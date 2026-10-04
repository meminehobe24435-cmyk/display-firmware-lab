/*
 * backlight.c -- PWM backlight control
 * ---------------------------------------------------------------------------
 * WHAT A MONITOR FIRMWARE ACTUALLY HAS TO GET RIGHT HERE
 *
 * A monitor backlight is driven by a PWM signal into an LED driver (or, on
 * older panels, a boost + dimming FET).  Four things bite people:
 *
 *   1. PERCEPTION.  Duty cycle is linear in light output, but the eye is not
 *      linear in light output.  Writing duty = brightness gives a control
 *      where the bottom half of the bar does almost nothing.  So the user
 *      value is gamma encoded:   duty = (brightness/100)^gamma
 *      with gamma ~2.2 (a plain power law standing in for sRGB-ish EOTF).
 *
 *   2. THE BOTTOM END.  duty = 0 means the panel is off; a 1 count duty on a
 *      20 kHz / 12-bit PWM is a visible flicker, not a dim image.  So there is
 *      a floor (min_duty_percent) that brightness 0 maps to, and the lowest
 *      usable step is reported as step_resolution_percent.
 *
 *   3. VISIBLE STEPS.  Slamming the duty register makes the panel flash and
 *      stresses the driver.  A change is therefore ramped over fade_ms, and
 *      last_delta_counts records how much the register moved on the last tick
 *      so a test can prove the ramp is gradual instead of a single jump.
 *
 *   4. HEAT.  Panels have a temperature limit; above it the firmware caps the
 *      brightness instead of letting the user cook the LEDs.  Two thresholds
 *      are modelled: derate_temp_c applies cap_warm_percent, and (if it keeps
 *      climbing) cutoff_temp_c applies cap_hot_percent.  A derate that
 *      actually limited the output is counted in derate_events.
 *
 * Everything is integer-count accurate: the LUT stores PWM counts, so what a
 * test reads back is exactly what would be written to the timer register.
 */
#include <math.h>
#include <string.h>

#include "dfw.h"

/* ---------------------------------------------------------------- helpers */

static unsigned dfw_bl_round_u(double v)
{
    return (unsigned)(v + 0.5);
}

/* ---------------------------------------------------------------- defaults */

void dfw_bl_cfg_default(dfw_bl_cfg *cfg)
{
    if (cfg == NULL) {
        return;
    }
    cfg->gamma            = 2.2;
    cfg->min_duty_percent = 1.0;
    cfg->pwm_counts       = 4095u;   /* 12-bit timer                      */
    cfg->pwm_hz           = 20000u;  /* 20 kHz: above audible, above CFF   */
    cfg->max_percent      = 100;
    cfg->fade_ms          = 300;
    cfg->temp_c           = 25;
    cfg->derate_temp_c    = 70;
    cfg->cutoff_temp_c    = 85;
    cfg->cap_warm_percent = 70;
    cfg->cap_hot_percent  = 40;
}

/* ---------------------------------------------------------------- mapping */

/*
 * gamma_encode: user brightness (0..100) -> duty (percent).
 *
 *   pct <= 0            -> min_duty_percent   (never a fully dark register)
 *   0 < pct <= 100      -> clamp(100*(pct/100)^gamma, min_duty, 100)
 *
 * The clamp is what makes the curve *not* a pure power law at the bottom; the
 * test asserts monotonicity of the resulting LUT rather than of the formula.
 */
double dfw_bl_gamma_encode(const dfw_bl_cfg *cfg, int brightness_percent)
{
    double g, duty, n;
    if (cfg == NULL) {
        return 0.0;
    }
    brightness_percent = dfw_clamp_i(brightness_percent, 0, 100);
    g = cfg->gamma > 0.0 ? cfg->gamma : 1.0;

    if (brightness_percent <= 0) {
        duty = 0.0;
    } else {
        n = (double)brightness_percent / 100.0;
        duty = 100.0 * pow(n, g);
    }
    if (duty < cfg->min_duty_percent) {
        duty = cfg->min_duty_percent;
    }
    if (duty > 100.0) {
        duty = 100.0;
    }
    return duty;
}

/* duty (percent) -> PWM counts, on the configured resolution. */
unsigned dfw_bl_duty_counts(const dfw_bl_cfg *cfg, double duty_percent)
{
    double counts;
    if (cfg == NULL || cfg->pwm_counts == 0u) {
        return 0u;
    }
    if (duty_percent < 0.0) {
        duty_percent = 0.0;
    }
    if (duty_percent > 100.0) {
        duty_percent = 100.0;
    }
    counts = duty_percent * (double)cfg->pwm_counts / 100.0;
    return dfw_bl_round_u(counts) > cfg->pwm_counts
               ? cfg->pwm_counts
               : dfw_bl_round_u(counts);
}

/* ---------------------------------------------------------------- thermal */

/* 0 = no cap, 1 = warm cap, 2 = hot cap. */
int dfw_bl_derate_level(const dfw_bl *b)
{
    if (b == NULL) {
        return 0;
    }
    if (b->cfg.temp_c >= b->cfg.cutoff_temp_c) {
        return 2;
    }
    if (b->cfg.temp_c >= b->cfg.derate_temp_c) {
        return 1;
    }
    return 0;
}

/*
 * Compute output_brightness = min(user brightness, ceiling, thermal cap).
 * The ceiling is cfg.max_percent (a service/limit setting).
 * derate_events counts only the case where the THERMAL cap is what bit.
 */
int dfw_bl_apply_thermal(dfw_bl *b)
{
    int cap, out, level;
    if (b == NULL) {
        return 0;
    }
    level = dfw_bl_derate_level(b);
    cap = (level == 2) ? b->cfg.cap_hot_percent
        : (level == 1) ? b->cfg.cap_warm_percent
                       : 100;
    if (cap > b->cfg.max_percent) {
        cap = b->cfg.max_percent;
    }
    if (cap < 0) {
        cap = 0;
    }
    out = b->brightness;
    if (out > cap) {
        out = cap;
        if (level != 0) {
            b->derate_events++;
        }
    }
    if (out < 0) {
        out = 0;
    }
    if (out > 100) {
        out = 100;
    }
    b->output_brightness = out;
    return out;
}

void dfw_bl_set_temp(dfw_bl *b, int temp_c)
{
    if (b == NULL) {
        return;
    }
    b->cfg.temp_c = temp_c;
    dfw_bl_apply_thermal(b);
    dfw_bl_set_brightness(b, b->brightness);
}

/* ---------------------------------------------------------------- init */

void dfw_bl_init(dfw_bl *b, const dfw_bl_cfg *cfg)
{
    int i;
    if (b == NULL) {
        return;
    }
    memset(b, 0, sizeof(*b));
    if (cfg != NULL) {
        b->cfg = *cfg;
    } else {
        dfw_bl_cfg_default(&b->cfg);
    }
    if (b->cfg.pwm_counts == 0u) {
        b->cfg.pwm_counts = 4095u;
    }
    if (b->cfg.pwm_hz == 0u) {
        b->cfg.pwm_hz = 20000u;
    }
    b->period_counts = b->cfg.pwm_counts;
    b->pwm_period_us = 1000000.0 / (double)b->cfg.pwm_hz;
    /* One count expressed as a percentage of the full scale. */
    b->step_resolution_percent = 100.0 / (double)b->cfg.pwm_counts;

    /* Build the user-brightness -> counts LUT once; runtime then costs a
     * table lookup, which is what a real firmware does. */
    for (i = 0; i <= 100; i++) {
        b->lut[i] = dfw_bl_duty_counts(&b->cfg, dfw_bl_gamma_encode(&b->cfg, i));
    }

    b->brightness        = 100;
    b->output_brightness = 100;
    b->duty_percent      = dfw_bl_gamma_encode(&b->cfg, 100);
    b->duty_counts       = b->lut[100];
    b->fading            = 0;
}

/* ---------------------------------------------------------------- runtime */

void dfw_bl_set_brightness(dfw_bl *b, int brightness)
{
    int out;
    if (b == NULL) {
        return;
    }
    b->brightness = dfw_clamp_i(brightness, 0, 100);
    out = dfw_bl_apply_thermal(b);

    b->duty_counts  = b->lut[out];
    b->duty_percent = 100.0 * (double)b->duty_counts / (double)b->cfg.pwm_counts;
    b->fading       = 0;
    b->transitions++;
}

int dfw_bl_fade_to(dfw_bl *b, int target_percent)
{
    int out;
    unsigned to;
    if (b == NULL) {
        return -1;
    }
    target_percent = dfw_clamp_i(target_percent, 0, 100);
    b->brightness = target_percent;
    out = dfw_bl_apply_thermal(b);
    to = b->lut[out];

    b->fade_from_counts = b->duty_counts;
    b->fade_to_counts   = to;
    b->fade_elapsed_ms  = 0;
    b->fade_total_ms    = b->cfg.fade_ms > 0 ? b->cfg.fade_ms : 1;
    b->fade_count++;
    b->transitions++;

    if (b->fade_from_counts == b->fade_to_counts || b->cfg.fade_ms <= 0) {
        b->duty_counts  = to;
        b->duty_percent = 100.0 * (double)to / (double)b->cfg.pwm_counts;
        b->fading       = 0;
        b->last_delta_counts = 0;
        return 0;
    }
    b->fading = 1;
    b->last_delta_counts = 0;
    return 0;
}

void dfw_bl_tick(dfw_bl *b, int dt_ms)
{
    double t;
    unsigned want;
    if (b == NULL || dt_ms <= 0) {
        return;
    }
    if (!b->fading) {
        b->last_delta_counts = 0;
        return;
    }
    b->fade_elapsed_ms += dt_ms;
    if (b->fade_elapsed_ms >= b->fade_total_ms) {
        /* last tick: report how far the register moved on THIS tick, not the
         * whole span -- last_delta_counts is what a test uses to prove the
         * ramp is gradual, so the final step must be an honest step. */
        b->last_delta_counts = (b->fade_to_counts > b->duty_counts)
                                   ? (int)(b->fade_to_counts - b->duty_counts)
                                   : (int)(b->duty_counts - b->fade_to_counts);
        b->duty_counts = b->fade_to_counts;
        b->fading      = 0;
    } else {
        t = (double)b->fade_elapsed_ms / (double)b->fade_total_ms;
        want = dfw_bl_round_u(dfw_lerp((double)b->fade_from_counts,
                                       (double)b->fade_to_counts, t));
        if (want > b->cfg.pwm_counts) {
            want = b->cfg.pwm_counts;
        }
        b->last_delta_counts = (want > b->duty_counts)
                                   ? (int)(want - b->duty_counts)
                                   : (int)(b->duty_counts - want);
        b->duty_counts = want;
    }
    b->duty_percent = 100.0 * (double)b->duty_counts / (double)b->cfg.pwm_counts;
}

/* ---------------------------------------------------------------- checks */

/* Non-zero when the LUT never decreases as brightness rises. */
int dfw_bl_is_monotonic(const dfw_bl *b)
{
    int i;
    if (b == NULL) {
        return 0;
    }
    for (i = 1; i <= 100; i++) {
        if (b->lut[i] < b->lut[i - 1]) {
            return 0;
        }
    }
    return 1;
}
