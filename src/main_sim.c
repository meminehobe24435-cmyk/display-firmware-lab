/*
 * main_sim.c -- end-to-end simulation of the five display-firmware modules
 * ---------------------------------------------------------------------------
 * Walks the same path a monitor firmware walks on power-up and during use:
 *
 *   power on -> read + validate EDID -> build the supported mode list
 *            -> HPD events pick an input source
 *            -> the user opens the OSD and changes brightness
 *            -> the backlight follows, with thermal derating
 *            -> the host (PC) reads the resulting values back over DDC/CI
 *
 * Every number printed here is computed by the code in src/; nothing is a
 * constant table.  Results are also written to <out_dir>/metrics.txt and to a
 * couple of CSVs so a reader can re-derive the figures.
 *
 * This is a simulation: there is no monitor, no EEPROM, no I2C bus and no
 * scaler.  See README.md "Boundaries".
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "dfw.h"

#if defined(_WIN32)
#include <direct.h>
#define DFW_MKDIR(p) _mkdir(p)
#else
#define DFW_MKDIR(p) mkdir((p), 0777)
#endif

/* ------------------------------------------------------------ utilities */

static void ensure_dir(const char *p)
{
    struct stat st;
    if (p == NULL || p[0] == '\0') {
        return;
    }
    if (stat(p, &st) == 0) {
        return;
    }
    (void)DFW_MKDIR(p);
}

static FILE *open_out(const char *dir, const char *name, char *pathbuf, size_t cap)
{
    FILE *f;
    if (dir == NULL || dir[0] == '\0') {
        dir = ".";
    }
    snprintf(pathbuf, cap, "%s/%s", dir, name);
    f = fopen(pathbuf, "wb");        /* binary: no 0x1A translation on Windows */
    return f;
}

/* --------------------------------------------------------- EDID fixture */

/* A 1920x1080@60 + 1280x720@60 monitor, assembled byte by byte. */
static void build_sim_edid(uint8_t *blk)
{
    unsigned pclk = 14850u;                 /* 148.50 MHz in 10 kHz units */
    unsigned ha = 1920, hb = 280, hso = 88, hsw = 44;
    unsigned va = 1080, vb = 45, vso = 4, vsw = 5;
    uint16_t mfg;

    memset(blk, 0, DFW_EDID_BLOCK_SIZE);
    blk[0] = 0x00;
    blk[1] = 0xFF; blk[2] = 0xFF; blk[3] = 0xFF;
    blk[4] = 0xFF; blk[5] = 0xFF; blk[6] = 0xFF; blk[7] = 0x00;

    mfg = (uint16_t)((((uint16_t)('A' - 'A' + 1)) << 10) |
                     (((uint16_t)('O' - 'A' + 1)) << 5) |
                     ((uint16_t)('C' - 'A' + 1)));
    blk[8] = (uint8_t)(mfg >> 8);
    blk[9] = (uint8_t)(mfg & 0xFFu);
    blk[10] = 0x2A; blk[11] = 0x24;         /* product 0x242A */
    blk[12] = 0x01; blk[13] = 0x00; blk[14] = 0x00; blk[15] = 0x00;
    blk[16] = 12;
    blk[17] = 36;                           /* 2026 */
    blk[18] = 1; blk[19] = 4;               /* EDID 1.4 */
    blk[20] = (uint8_t)(0x80u | (6u << 4) | 5u);   /* digital, 10 bpc, DP */
    blk[21] = 52; blk[22] = 29;
    blk[23] = 120;                          /* gamma 2.20 */
    blk[24] = (uint8_t)(0x80u | 0x40u | 0x04u | 0x02u);
    blk[25] = 0xEE; blk[26] = 0x91;
    blk[27] = 0xA3; blk[28] = 0x54;
    blk[29] = 0x4C; blk[30] = 0x99;
    blk[31] = 0x26; blk[32] = 0x0F;
    blk[33] = 0x50; blk[34] = 0x54;
    blk[35] = 0x21; blk[36] = 0x08; blk[37] = 0x00;
    {
        int i;
        for (i = 0; i < 8; i++) {
            blk[38 + 2 * i] = 0x01;
            blk[39 + 2 * i] = 0x01;
        }
    }
    /* DTD 1: 1920x1080@60 */
    blk[54] = (uint8_t)(pclk & 0xFFu);
    blk[55] = (uint8_t)((pclk >> 8) & 0xFFu);
    blk[56] = (uint8_t)(ha & 0xFFu);
    blk[57] = (uint8_t)(hb & 0xFFu);
    blk[58] = (uint8_t)(((ha >> 8) << 4) | ((hb >> 8) & 0x0Fu));
    blk[59] = (uint8_t)(va & 0xFFu);
    blk[60] = (uint8_t)(vb & 0xFFu);
    blk[61] = (uint8_t)(((va >> 8) << 4) | ((vb >> 8) & 0x0Fu));
    blk[62] = (uint8_t)hso; blk[63] = (uint8_t)hsw;
    blk[64] = (uint8_t)(((vso & 0x0Fu) << 4) | (vsw & 0x0Fu));
    blk[65] = 0; blk[66] = 0; blk[67] = 0;
    blk[68] = 52; blk[69] = 29; blk[70] = 0; blk[71] = 0x16;

    /* DTD 2: 1280x720@60 (74.25 MHz, 1650 x 750) */
    {
        unsigned p2 = 7425u, a2 = 1280, b2 = 370, so2 = 110, sw2 = 40;
        unsigned v2 = 720, vb2 = 30, vso2 = 5, vsw2 = 5;
        uint8_t *d = blk + 72;
        d[0] = (uint8_t)(p2 & 0xFFu);
        d[1] = (uint8_t)((p2 >> 8) & 0xFFu);
        d[2] = (uint8_t)(a2 & 0xFFu);
        d[3] = (uint8_t)(b2 & 0xFFu);
        d[4] = (uint8_t)(((a2 >> 8) << 4) | ((b2 >> 8) & 0x0Fu));
        d[5] = (uint8_t)(v2 & 0xFFu);
        d[6] = (uint8_t)(vb2 & 0xFFu);
        d[7] = (uint8_t)(((v2 >> 8) << 4) | ((vb2 >> 8) & 0x0Fu));
        d[8] = (uint8_t)so2; d[9] = (uint8_t)sw2;
        d[10] = (uint8_t)(((vso2 & 0x0Fu) << 4) | (vsw2 & 0x0Fu));
        d[17] = 0x16;
    }
    /* descriptor 3: product name, descriptor 4: range limits */
    {
        uint8_t name[18];
        uint8_t range[18];
        dfw_edid_build_desc_name(name, "AOC 24G2");
        dfw_edid_build_desc_range(range, 48.0, 75.0, 30.0, 83.0, 170.0);
        memcpy(blk + 90, name, 18);
        memcpy(blk + 108, range, 18);
    }
    blk[126] = 0;                           /* no extension blocks */
    dfw_edid_fix_checksum(blk);
}

/* ------------------------------------------------------------- sections */

static void section_edid(const uint8_t *blk, dfw_edid *e, FILE *out)
{
    dfw_mode modes[DFW_EDID_MAX_MODES];
    size_t n, i;
    int rc;

    rc = dfw_edid_parse(blk, DFW_EDID_BLOCK_SIZE, e);
    fprintf(out, "== 1. EDID ==\n");
    fprintf(out, "parse status            : %d (%s)\n", rc, dfw_edid_status_str(rc));
    fprintf(out, "checksum valid          : %s\n", dfw_edid_checksum_ok(blk) ? "yes" : "no");
    fprintf(out, "manufacturer            : %s (0x%04X)\n", e->mfg_letters, e->mfg_id);
    fprintf(out, "product code            : 0x%04X\n", e->product_code);
    fprintf(out, "EDID version            : %u.%u\n", e->version, e->revision);
    fprintf(out, "digital input           : %s, %u bpc, interface code %u\n",
            e->input_type ? "yes" : "no", e->bit_depth, e->interface_type);
    fprintf(out, "screen size             : %ux%u mm (%.1f inch diagonal)\n",
            e->image_w_mm, e->image_h_mm, e->diagonal_inch);
    fprintf(out, "gamma                   : %.2f\n", e->gamma);
    fprintf(out, "DTDs found              : %u\n", (unsigned)e->dtd_count);
    for (i = 0; i < e->dtd_count; i++) {
        const dfw_edid_dtd *d = &e->dtds[i];
        fprintf(out, "  mode %u                : %ux%u @ %.2f Hz  (pclk %.2f MHz, "
                     "h_total %u, v_total %u, %s sync)\n",
                (unsigned)(i + 1), d->h_active, d->v_active, d->refresh_hz,
                d->pixel_clock_mhz, d->h_total, d->v_total,
                d->interlaced ? "interlaced" : "progressive");
    }
    fprintf(out, "monitor descriptors     : name=\"%s\" serial-ish=\"%s\" range=%s\n",
            e->descs[2].text, e->descs[1].text,
            e->descs[3].has_range ? "yes" : "no");
    if (e->descs[3].has_range) {
        fprintf(out, "  range limits          : v %g..%g Hz, h %g..%g kHz, "
                     "max pclk %g MHz\n",
                e->descs[3].v_min_hz, e->descs[3].v_max_hz,
                e->descs[3].h_min_khz, e->descs[3].h_max_khz,
                e->descs[3].max_pixel_clock_mhz);
    }
    n = dfw_edid_expected_modes(e, modes, DFW_EDID_MAX_MODES);
    fprintf(out, "expected modes          : %u\n", (unsigned)n);
    for (i = 0; i < n; i++) {
        fprintf(out, "  %4ux%-4u @ %6.2f Hz\n", modes[i].h_active,
                modes[i].v_active, modes[i].refresh_hz);
    }
    /* corrupt one byte and show that the parser refuses the block */
    {
        uint8_t bad[DFW_EDID_BLOCK_SIZE];
        dfw_edid be;
        memcpy(bad, blk, sizeof bad);
        bad[100] ^= 0x08;
        rc = dfw_edid_parse(bad, sizeof bad, &be);
        fprintf(out, "corrupted byte 100      : parse=%d (%s)\n",
                rc, dfw_edid_status_str(rc));
    }
    fprintf(out, "\n");
}

static void section_ddc(dfw_ddc_slave *s, dfw_ddc_bus *bus, FILE *out)
{
    dfw_ddc_reply rep;
    int rc;

    dfw_ddc_slave_init(s);
    dfw_ddc_slave_add_vcp(s, DFW_VCP_BRIGHTNESS, 70, 100, DFW_DDC_MODE_SET_PARAM);
    dfw_ddc_slave_add_vcp(s, DFW_VCP_CONTRAST, 50, 100, DFW_DDC_MODE_SET_PARAM);
    dfw_ddc_slave_add_vcp(s, DFW_VCP_INPUT_SOURCE, 0x11, 0x1F, 0x00);
    dfw_ddc_slave_add_vcp(s, DFW_VCP_POWER_MODE, 0x01, 0x05, 0x00);
    dfw_ddc_slave_add_vcp(s, DFW_VCP_VERSION, 0x0100, 0xFFFF, 0x00);
    dfw_ddc_bus_init(bus, s);

    fprintf(out, "== 2. DDC/CI ==\n");
    fprintf(out, "VCP table               : %u controls\n", (unsigned)s->vcp_count);

    rc = dfw_ddc_get_vcp(bus, DFW_VCP_BRIGHTNESS, &rep);
    fprintf(out, "GET 0x10 brightness     : rc=%d value=%u max=%u result=0x%02X\n",
            rc, rep.value, rep.max_value, rep.result_code);

    /* the round trip that matters: Set, then read the value back */
    dfw_ddc_set_vcp(bus, DFW_VCP_BRIGHTNESS, 42);
    dfw_ddc_get_vcp(bus, DFW_VCP_BRIGHTNESS, &rep);
    fprintf(out, "SET 0x10=42 then GET    : value=%u (%s)\n", rep.value,
            rep.value == 42u ? "read back OK" : "MISMATCH");

    /* clamping: the monitor does not trust the host's range */
    dfw_ddc_set_vcp(bus, DFW_VCP_BRIGHTNESS, 250);
    dfw_ddc_get_vcp(bus, DFW_VCP_BRIGHTNESS, &rep);
    fprintf(out, "SET 0x10=250 (over max) : value=%u (clamped to max=%u)\n",
            rep.value, rep.max_value);

    /* a control the monitor does not have */
    dfw_ddc_get_vcp(bus, 0x7A, &rep);
    fprintf(out, "GET 0x7A (unsupported)  : result=0x%02X\n", rep.result_code);

    /* the retry policy, exercised with a one-shot silent monitor */
    {
        uint32_t r0 = bus->stats.retries;
        s->abort_next = 1;
        rc = dfw_ddc_get_vcp(bus, DFW_VCP_CONTRAST, &rep);
        fprintf(out, "one silent attempt      : rc=%d, retries used=%u, recovered=%s\n",
                rc, bus->stats.retries - r0, rc == DFW_DDC_OK ? "yes" : "no");
    }
    fprintf(out, "transactions            : %u ok, %u timeouts, %u retries, "
                 "%u unsupported\n",
            bus->stats.transactions, bus->stats.timeouts, bus->stats.retries,
            bus->stats.unsupported);
    fprintf(out, "slave counters          : %u requests, %u checksum errors, "
                 "%u malformed\n",
            s->transactions, s->checksum_errors, s->malformed_requests);
    fprintf(out, "\n");
}

static void section_mux(dfw_mux *m, FILE *out)
{
    dfw_mux_cfg cfg;
    int i;

    dfw_mux_cfg_default(&cfg);
    dfw_mux_init(m, &cfg);

    fprintf(out, "== 3. Input source switching ==\n");
    /* plug HDMI2 and DP, with HDMI2 arriving later so the mux must move */
    dfw_mux_hpd(m, DFW_SRC_HDMI1, 1, 0);
    for (i = 0; i < 40; i++) dfw_mux_tick(m, 20);
    dfw_mux_hpd(m, DFW_SRC_DP, 1, 0);
    for (i = 0; i < 200; i++) dfw_mux_tick(m, 20);
    fprintf(out, "after HPD up (HDMI1, DP): state=%s current=%s\n",
            dfw_mux_state_name(m->state), dfw_src_name(m->current));

    /* jitter: 10 fast HPD toggles on HDMI1 must not disturb the selection */
    {
        int before = m->current;
        uint32_t att = m->switch_attempts;
        for (i = 0; i < 10; i++) {
            dfw_mux_hpd(m, DFW_SRC_HDMI1, 0, 5);
            dfw_mux_hpd(m, DFW_SRC_HDMI1, 1, 5);
        }
        for (i = 0; i < 20; i++) dfw_mux_tick(m, 20);
        fprintf(out, "10 HPD toggles in 100 ms: current %s -> %s, extra switch "
                     "attempts=%u\n",
                dfw_src_name(before), dfw_src_name(m->current),
                m->switch_attempts - att);
    }
    /* a real unplug: hold HPD low past the debounce window */
    {
        dfw_mux_hpd(m, DFW_SRC_HDMI1, 0, 0);
        for (i = 0; i < 60; i++) dfw_mux_tick(m, 20);
        fprintf(out, "HDMI1 held low >120 ms  : present=%s\n",
                dfw_mux_is_present(m, DFW_SRC_HDMI1) ? "yes" : "no");
    }
    /* switch to a source that is not connected */
    {
        int before = m->current;
        dfw_mux_user_select(m, DFW_SRC_VGA, 0);
        for (i = 0; i < 400; i++) dfw_mux_tick(m, 20);
        fprintf(out, "select dead VGA         : current %s -> %s, showing an "
                     "absent input=%s\n",
                dfw_src_name(before), dfw_src_name(m->current),
                dfw_mux_is_present(m, m->current) ? "no" : "YES (bug)");
    }
    fprintf(out, "switch attempts=%u ok=%u timeout=%u dwell-blocked=%u\n",
            m->switch_attempts, m->switch_ok, m->switch_timeout,
            m->switch_aborted_by_dwell);
    fprintf(out, "switch log (%u entries):\n", (unsigned)m->log_count);
    for (i = 0; i < (int)m->log_count && i < 12; i++) {
        fprintf(out, "  t=%6u ms  %s -> %s  (%s)\n", m->logs[i][0],
                dfw_src_name(m->logs[i][1]), dfw_src_name(m->logs[i][2]),
                dfw_mux_reason_name(m->logs[i][3]));
    }
    fprintf(out, "\n");
}

static void section_osd(FILE *out)
{
    dfw_osd o;
    dfw_osd_cfg cfg;
    dfw_osd_settings set;
    dfw_osd_io io;
    int i;

    dfw_osd_cfg_default(&cfg);
    dfw_osd_settings_default(&set);
    memset(&io, 0, sizeof io);
    dfw_osd_init(&o, &cfg, &set, &io);
    dfw_osd_power(&o, DFW_POWER_ON);

    fprintf(out, "== 4. OSD ==\n");
    fprintf(out, "items                   : %u\n", (unsigned)o.item_count);
    fprintf(out, "idle timeout            : %d ms\n", cfg.idle_timeout_ms);

    dfw_osd_key(&o, DFW_KEY_OK, DFW_KEY_PRESS, 10);
    dfw_osd_tick(&o, 10);
    fprintf(out, "after OK                : state=%s index=%d\n",
            dfw_osd_state_name(o.state), o.menu_index);

    /* walk the whole menu once */
    for (i = 0; i < (int)o.item_count + 1; i++) {
        dfw_osd_key(&o, DFW_KEY_DOWN, DFW_KEY_PRESS, 200);
        dfw_osd_tick(&o, 200);
    }
    fprintf(out, "after walking the menu  : state=%s index=%d (in range=%s)\n",
            dfw_osd_state_name(o.state), o.menu_index,
            (o.menu_index >= 0 && o.menu_index < (int)o.item_count) ? "yes" : "NO");

    /* idle timeout */
    for (i = 0; i < 60; i++) {
        dfw_osd_tick(&o, 500);          /* 30 s, well past 15 s */
    }
    fprintf(out, "after 30 s idle         : state=%s timeouts=%u manual=%u\n",
            dfw_osd_state_name(o.state), o.close_timeout_count,
            o.close_manual_count);

    /* factory mode: hold OK while off, then press UP inside the window */
    {
        dfw_osd f;
        dfw_osd_init(&f, &cfg, &set, &io);
        dfw_osd_power(&f, DFW_POWER_OFF);
        dfw_osd_key(&f, DFW_KEY_OK, DFW_KEY_PRESS, 100);
        for (i = 0; i < 12; i++) dfw_osd_tick(&f, 500);   /* 6 s > 5 s hold */
        dfw_osd_key(&f, DFW_KEY_OK, DFW_KEY_RELEASE, 100);
        dfw_osd_key(&f, DFW_KEY_UP, DFW_KEY_PRESS, 100);
        fprintf(out, "factory sequence (correct order)  : entered=%s\n",
                f.factory_mode ? "yes" : "NO");
    }
    {
        dfw_osd f;
        dfw_osd_init(&f, &cfg, &set, &io);
        dfw_osd_power(&f, DFW_POWER_OFF);
        dfw_osd_key(&f, DFW_KEY_UP, DFW_KEY_PRESS, 100);   /* wrong order */
        dfw_osd_key(&f, DFW_KEY_OK, DFW_KEY_PRESS, 100);
        fprintf(out, "factory sequence (wrong order)    : entered=%s, rejects=%u\n",
                f.factory_mode ? "YES (bug)" : "no", f.factory_reject_count);
    }
    fprintf(out, "\n");
}

static void section_backlight(FILE *out, const char *dir)
{
    dfw_bl_cfg cfg;
    dfw_bl bl;
    int i;
    FILE *csv;
    char path[512];

    dfw_bl_cfg_default(&cfg);
    dfw_bl_init(&bl, &cfg);

    fprintf(out, "== 5. Backlight PWM ==\n");
    fprintf(out, "PWM                      : %u counts @ %u Hz -> period %.1f us, "
                 "one step = %.4f %%\n",
            bl.period_counts, cfg.pwm_hz, bl.pwm_period_us,
            bl.step_resolution_percent);
    fprintf(out, "gamma                    : %.2f, floor %.2f %%\n",
            cfg.gamma, cfg.min_duty_percent);
    fprintf(out, "LUT monotonic           : %s\n", dfw_bl_is_monotonic(&bl) ? "yes" : "NO");
    fprintf(out, "brightness 0/25/50/75/100 -> counts %u/%u/%u/%u/%u (of %u)\n",
            bl.lut[0], bl.lut[25], bl.lut[50], bl.lut[75], bl.lut[100],
            bl.period_counts);
    fprintf(out, "gamma notice            : 50 %% on the bar is only %u counts "
                 "(%.1f %% duty), which is the point of the curve\n",
            bl.lut[50], 100.0 * (double)bl.lut[50] / (double)bl.period_counts);

    /* fade from 100 % to 10 %: count the steps and the biggest single step */
    dfw_bl_set_brightness(&bl, 100);
    {
        unsigned start = bl.duty_counts;
        int max_delta = 0, steps = 0, ticks = 0;
        dfw_bl_fade_to(&bl, 10);
        while (bl.fading && ticks < 1000) {
            unsigned before = bl.duty_counts;
            dfw_bl_tick(&bl, 10);
            if (bl.duty_counts != before) steps++;
            if (bl.last_delta_counts > max_delta) max_delta = bl.last_delta_counts;
            ticks++;
        }
        fprintf(out, "fade 100%% -> 10%%         : %u -> %u counts in %d ticks "
                     "(%d ms), %d visible steps, largest step %d counts\n",
                start, bl.duty_counts, ticks, ticks * 10, steps, max_delta);
    }

    /* thermal derating */
    fprintf(out, "thermal thresholds      : derate at %d C -> %d %%, cutoff at "
                 "%d C -> %d %%\n",
            cfg.derate_temp_c, cfg.cap_warm_percent,
            cfg.cutoff_temp_c, cfg.cap_hot_percent);
    dfw_bl_set_brightness(&bl, 100);
    for (i = 0; i <= 90; i += 15) {
        dfw_bl_set_temp(&bl, i);
        fprintf(out, "  panel %2d C            : level %d, output %u %%, "
                     "%u counts%s\n",
                i, dfw_bl_derate_level(&bl), bl.output_brightness,
                bl.duty_counts,
                (bl.output_brightness < bl.brightness) ? "  (derated)" : "");
    }
    fprintf(out, "derate events           : %u\n", bl.derate_events);
    dfw_bl_set_temp(&bl, 25);
    fprintf(out, "cooled back to 25 C     : output %u %% (user asked %u %%)\n",
            bl.output_brightness, bl.brightness);

    /* the LUT as a CSV so the curve can be plotted */
    csv = open_out(dir, "backlight_lut.csv", path, sizeof path);
    if (csv) {
        fprintf(csv, "brightness_percent,duty_counts,duty_percent\n");
        for (i = 0; i <= 100; i++) {
            fprintf(csv, "%d,%u,%.4f\n", i, bl.lut[i],
                    100.0 * (double)bl.lut[i] / (double)bl.period_counts);
        }
        fclose(csv);
    }
    fprintf(out, "\n");
}

/* ------------------------------------------------------------------ run */

int dfw_sim_run(const char *out_dir)
{
    uint8_t blk[DFW_EDID_BLOCK_SIZE];
    dfw_edid edid;
    dfw_ddc_slave slave;
    dfw_ddc_bus bus;
    dfw_mux mux;
    FILE *out;
    char path[512];
    const char *dir = (out_dir && out_dir[0]) ? out_dir : "results";

    ensure_dir(dir);

    out = open_out(dir, "metrics.txt", path, sizeof path);
    if (out == NULL) {
        out = stdout;
    }

    fprintf(out, "display-firmware-lab -- end to end display firmware simulation\n");
    fprintf(out, "Every number below is computed by src/; see %s/*.csv for raw data.\n\n",
            dir);

    build_sim_edid(blk);
    section_edid(blk, &edid, out);
    section_ddc(&slave, &bus, out);
    section_mux(&mux, out);
    section_osd(out);
    section_backlight(out, dir);

    fprintf(out, "== summary ==\n");
    fprintf(out, "This is a pure logic/numerical simulation: no monitor, no EDID\n");
    fprintf(out, "EEPROM, no I2C/DDC bus, no scaler and no backlight driver were\n");
    fprintf(out, "involved.  See README.md \"Boundaries\" for what is not modelled.\n");

    if (out != stdout) {
        fclose(out);
        printf("wrote %s/metrics.txt\n", dir);
    }
    return 0;
}

#ifndef DFW_SIM_NO_MAIN
int main(int argc, char **argv)
{
    return dfw_sim_run(argc > 1 ? argv[1] : "results");
}
#endif
