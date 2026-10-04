/*
 * source_mux.c -- input source switching and hot-plug handling.
 *
 * ---------------------------------------------------------------------
 * WHAT A REAL SCALER DOES
 * ---------------------------------------------------------------------
 * A monitor has several receivers (HDMI x2, DisplayPort, USB Type-C, VGA)
 * feeding one scaler.  HDMI and DP carry a hot-plug detect line: the sink
 * pulls HPD high once it has read the source's EDID, and drops it when the
 * cable is pulled or the source sleeps.  Firmware must therefore:
 *
 *   1. debounce HPD     - a plug/unplug produces a burst of edges on real
 *                         hardware, and switching on every edge makes the
 *                         screen flash.  An edge only becomes real after
 *                         it has held `debounce_ms`.
 *   2. respect dwell    - never switch away from a source before
 *                         `min_dwell_ms` has passed, so a source that is
 *                         still training its link is not abandoned.
 *   3. break before make- turn the old receiver off, wait `settle_ms`, then
 *                         enable the new one.  Driving two receivers into
 *                         the same scaler input at once is what causes the
 *                         "two pictures" artefact on some silicon.
 *   4. verify           - after enabling, wait `enable_ms` for a valid
 *                         timing, then require `verify_ms` of stability.
 *   5. roll back        - if the target never stabilises within
 *                         `switch_timeout_ms`, return to the last source
 *                         that worked; if that is gone too, search.
 *   6. search / standby - a full pass over the priority order; if nothing
 *                         is present, go to standby rather than show a
 *                         frozen image.
 *
 * ---------------------------------------------------------------------
 * STATE MACHINE
 * ---------------------------------------------------------------------
 *   OFF --user power on--> STANDBY
 *   STANDBY --any source present--> SWITCHING --verify ok--> SHOWING
 *   SWITCHING --timeout--> ROLLBACK (previous source) --> SHOWING | SEARCH
 *   SHOWING --current HPD down--> SEARCH (priority order from current)
 *   SEARCH --candidate present--> SWITCHING
 *   SEARCH --full pass, nothing present--> STANDBY
 *
 *   Switch sequence (SWITCHING), one step per tick:
 *     step 0  disable current receiver + start settle timer
 *     step 1  settle_ms elapsed -> enable target receiver
 *     step 2  enable_ms elapsed -> if target HPD is up, start verify
 *     step 3  verify_ms elapsed with HPD still up -> commit, state SHOWING
 *
 * The search order and per-source priority are data (cfg.order), so
 * "which source wins" is testable without reading the source file.
 *
 * All numbers in dfw_mux_cfg are engineering defaults for a 27" 1080p
 * monitor, not measurements of any particular product.
 */
#include "dfw.h"

#include <string.h>

static const char *src_names[DFW_SRC_COUNT] = {
    "HDMI1", "HDMI2", "DP", "Type-C", "VGA"
};

static const char *state_names[] = {
    "OFF", "STANDBY", "SHOWING", "SWITCHING", "NO_SIGNAL"
};

static const char *reason_names[] = {
    "boot", "user", "hpd", "auto-search", "fallback", "no-signal", "rollback"
};

const char *dfw_src_name(int src)
{
    if (src < 0 || src >= DFW_SRC_COUNT) return "?";
    return src_names[src];
}

const char *dfw_mux_state_name(int state)
{
    if (state < 0 || state > DFW_MUX_NO_SIGNAL) return "?";
    return state_names[state];
}

const char *dfw_mux_reason_name(int reason)
{
    if (reason < 0 || reason > DFW_REASON_ROLLBACK) return "?";
    return reason_names[reason];
}

void dfw_mux_cfg_default(dfw_mux_cfg *cfg)
{
    int i;
    if (!cfg) return;
    memset(cfg, 0, sizeof *cfg);
    for (i = 0; i < DFW_SRC_COUNT; ++i)
        cfg->debounce_ms[i] = 120;
    cfg->min_dwell_ms      = 1000;
    cfg->settle_ms         = 200;
    cfg->enable_ms         = 300;
    cfg->verify_ms         = 200;
    cfg->switch_timeout_ms = 2500;
    cfg->search_dwell_ms   = 400;
    for (i = 0; i < DFW_SRC_COUNT; ++i)
        cfg->order[i] = i;      /* HDMI1, HDMI2, DP, Type-C, VGA */
}

static void mux_log(dfw_mux *m, int from, int to, int reason)
{
    if (m->log_count < sizeof m->logs / sizeof m->logs[0]) {
        m->logs[m->log_count][0] = (int)m->now_ms;
        m->logs[m->log_count][1] = from;
        m->logs[m->log_count][2] = to;
        m->logs[m->log_count][3] = reason;
        m->log_count++;
    }
}

void dfw_mux_init(dfw_mux *m, const dfw_mux_cfg *cfg)
{
    int i;
    if (!m) return;
    memset(m, 0, sizeof *m);
    if (cfg) m->cfg = *cfg;
    else     dfw_mux_cfg_default(&m->cfg);
    for (i = 0; i < DFW_SRC_COUNT; ++i) {
        m->port[i].hpd = 0;
        m->port[i].active = 0;
        m->port[i].level_pending = 0;
        m->port[i].timer_ms = 0;
        m->port[i].cap_score = 0;
    }
    m->state       = DFW_MUX_OFF;
    m->current     = -1;
    m->previous    = -1;
    m->target      = -1;
    m->search_from = 0;
    m->candidate   = -1;
    m->switch_step = 0;
    m->step_timer  = 0;
    m->now_ms      = 0;
}

int dfw_mux_is_present(const dfw_mux *m, int source)
{
    if (!m || source < 0 || source >= DFW_SRC_COUNT) return 0;
    return m->port[source].active;
}

int dfw_mux_first_available(const dfw_mux *m, int from)
{
    int k;
    if (!m || from < 0) from = 0;
    for (k = 0; k < DFW_SRC_COUNT; ++k) {
        int s = m->cfg.order[(from + k) % DFW_SRC_COUNT];
        if (m->port[s].active) return s;
    }
    return -1;
}

int dfw_mux_prev_in_order(const dfw_mux *m, int src)
{
    int k;
    if (!m) return -1;
    for (k = 0; k < DFW_SRC_COUNT; ++k) {
        if (m->cfg.order[k] == src)
            return m->cfg.order[(k + DFW_SRC_COUNT - 1) % DFW_SRC_COUNT];
    }
    return -1;
}

static int order_index(const dfw_mux *m, int src)
{
    int k;
    for (k = 0; k < DFW_SRC_COUNT; ++k)
        if (m->cfg.order[k] == src) return k;
    return 0;
}

/* Start the break-before-make sequence toward `target`. */
static int mux_begin_switch(dfw_mux *m, int target, int reason)
{
    if (target < 0 || target >= DFW_SRC_COUNT) return DFW_MUX_ERR_SOURCE;
    if (target == m->current && m->state == DFW_MUX_SHOWING) return DFW_MUX_OK;
    m->switch_attempts++;
    m->target         = target;
    m->switch_step    = 0;
    m->step_timer     = 0;
    m->switch_elapsed = 0;
    m->state          = DFW_MUX_SWITCHING;
    mux_log(m, m->current, target, reason);
    return DFW_MUX_OK;
}

/* No source is usable: go to standby (or NO_SIGNAL while already showing a
 * valid picture that the user may come back to). */
static void mux_enter_standby(dfw_mux *m, int reason)
{
    m->state = DFW_MUX_STANDBY;
    if (m->current >= 0) mux_log(m, m->current, -1, reason);
    m->current  = -1;
    m->target   = -1;
    m->switch_step = 0;
}

/* Walk the priority order once, starting after `from`. */
static void mux_start_search(dfw_mux *m, int from, int reason)
{
    int s;
    m->search_from = (from < 0) ? 0 : (order_index(m, from) + 1) % DFW_SRC_COUNT;
    m->candidate = -1;
    m->candidate_timer = 0;
    s = dfw_mux_first_available(m, m->search_from);
    if (s < 0) {
        mux_enter_standby(m, DFW_REASON_STANDBY);
        return;
    }
    m->state = DFW_MUX_NO_SIGNAL;
    m->candidate = s;
    mux_log(m, m->current, s, reason);
}

/* Try to show `src`; if it is not present, carry on down the priority
 * order from that point, and fall back to a full search if nothing is. */
static int mux_try_show(dfw_mux *m, int src, int reason)
{
    int k;
    if (src >= 0 && src < DFW_SRC_COUNT && m->port[src].active)
        return mux_begin_switch(m, src, reason);

    if (src >= 0 && src < DFW_SRC_COUNT) {
        for (k = 1; k < DFW_SRC_COUNT; ++k) {
            int idx  = (order_index(m, src) + k) % DFW_SRC_COUNT;
            int cand = m->cfg.order[idx];
            if (m->port[cand].active)
                return mux_begin_switch(m, cand, reason);
        }
    }
    mux_start_search(m, m->current, DFW_REASON_FALLBACK);
    return DFW_MUX_OK;
}

/* Current source lost its signal: fall back to the previous good source if
 * it is still present, otherwise run a full search. */
static void mux_handle_signal_loss(dfw_mux *m)
{
    int prev = m->previous;
    int lost = m->current;
    if (prev >= 0 && m->port[prev].active) {
        (void)mux_begin_switch(m, prev, DFW_REASON_FALLBACK);
        return;
    }
    mux_start_search(m, lost, DFW_REASON_FALLBACK);
}

/* ---------------------------------------------------------------- *
 * HPD debouncing
 * ---------------------------------------------------------------- */

static int mux_update_hpd(dfw_mux *m, int src, int dt_ms)
{
    dfw_mux_port *p = &m->port[src];
    int changed = 0;

    if (dt_ms <= 0) dt_ms = 1;

    if (p->hpd == p->active) {
        /* line agrees with the debounced state; drop any pending glitch */
        p->level_pending = p->hpd;
        p->timer_ms = 0;
        return 0;
    }

    if (p->hpd != p->level_pending) {
        /* The line moved again before the debounce window elapsed: this is
         * exactly the jitter the debouncer exists for.  Restart the clock. */
        p->level_pending = p->hpd;
        p->timer_ms = dt_ms;
        m->hpd_edges_ignored++;
        return 0;
    }

    p->timer_ms += dt_ms;
    if (p->timer_ms < m->cfg.debounce_ms[src]) return 0;

    /* Edge survived the debounce window. */
    p->active = p->level_pending;
    p->timer_ms = 0;
    changed = 1;

    if (p->active) {
        /* A source just became present.  Only switch to it if we are not
         * already showing a picture, or if nothing valid is displayed. */
        if (m->state == DFW_MUX_STANDBY || m->state == DFW_MUX_OFF ||
            m->state == DFW_MUX_NO_SIGNAL) {
            (void)mux_begin_switch(m, src, DFW_REASON_AUTO);
        }
        /* Otherwise remember it, but do not steal focus from a live input:
         * on real hardware that is how "the monitor jumped to HDMI while I
         * was working on DP" bugs happen. */
    } else {
        if (m->state == DFW_MUX_SHOWING && m->current == src)
            mux_handle_signal_loss(m);
        else if (m->state == DFW_MUX_SWITCHING && m->target == src) {
            /* target disappeared mid-switch; handled by the switch timeout */
        }
    }
    return changed;
}

/* ---------------------------------------------------------------- *
 * the switching sequence
 * ---------------------------------------------------------------- */

static void mux_advance_switch(dfw_mux *m, int dt_ms)
{
    /* `switch_elapsed` is the clock for the whole break-before-make
     * sequence and is never reset while switching, so the overall timeout
     * cannot be defeated by progressing through the stages.  `step_timer`
     * is only the clock for the current stage. */
    m->switch_elapsed += dt_ms;
    m->step_timer     += dt_ms;

    switch (m->switch_step) {
    case 0: /* break: old receiver off, wait for the sink to settle */
        if (m->step_timer >= m->cfg.settle_ms) {
            m->switch_step = 1;
            m->step_timer  = 0;
        }
        break;

    case 1: /* make: target receiver on, wait for a valid timing */
        /* Wait at least enable_ms before believing the presence detect:
         * right after the receiver is enabled the link has not trained
         * yet, and sampling too early reports a false "no signal". */
        if (m->step_timer >= m->cfg.enable_ms && m->port[m->target].active) {
            m->switch_step = 2;
            m->step_timer  = 0;
            break;
        }
        /* Still nothing.  Give the source a moment to retrain, but only up
         * to the overall switching timeout. */
        if (m->switch_elapsed >= m->cfg.switch_timeout_ms) {
            m->switch_timeout++;
            (void)mux_try_show(m, m->previous, DFW_REASON_ROLLBACK);
        }
        break;

    case 2: /* verify: the new signal must stay valid for verify_ms */
        if (!m->port[m->target].active) {
            m->switch_timeout++;
            (void)mux_try_show(m, m->previous, DFW_REASON_ROLLBACK);
            break;
        }
        if (m->step_timer >= m->cfg.verify_ms) {
            m->previous    = m->current;
            m->current     = m->target;
            m->target      = -1;
            m->state       = DFW_MUX_SHOWING;
            m->switch_step = 0;
            m->step_timer  = 0;   /* restarts the minimum-dwell window */
            m->switch_ok++;
        }
        break;

    default:
        m->switch_step = 0;
        m->step_timer  = 0;
        break;
    }
}

/* ---------------------------------------------------------------- *
 * public event entry points
 * ---------------------------------------------------------------- */

int dfw_mux_tick(dfw_mux *m, int dt_ms)
{
    int i;
    if (!m) return DFW_MUX_ERR_NULL;
    if (dt_ms < 0) return DFW_MUX_ERR_NULL;
    if (dt_ms == 0) return DFW_MUX_OK;

    m->now_ms += (uint32_t)dt_ms;

    /* debounce every port, in a fixed order so behaviour is deterministic */
    for (i = 0; i < DFW_SRC_COUNT; ++i)
        (void)mux_update_hpd(m, i, dt_ms);

    switch (m->state) {
    case DFW_MUX_SWITCHING:
        mux_advance_switch(m, dt_ms);
        break;

    case DFW_MUX_NO_SIGNAL:
        /* an automatic search lingers search_dwell_ms on each candidate
         * before moving on, which is what makes "no signal" take a
         * predictable amount of time instead of flickering through all
         * inputs instantly */
        if (m->candidate >= 0 && m->port[m->candidate].active) {
            (void)mux_begin_switch(m, m->candidate, DFW_REASON_AUTO);
        } else {
            m->candidate_timer += dt_ms;
            if (m->candidate_timer >= m->cfg.search_dwell_ms) {
                int next;
                m->candidate_timer = 0;
                m->search_from = (m->search_from + 1) % DFW_SRC_COUNT;
                next = dfw_mux_first_available(m, m->search_from);
                if (next < 0) {
                    if (m->search_from == 0) mux_enter_standby(m, DFW_REASON_STANDBY);
                } else {
                    m->candidate = next;
                }
            }
        }
        break;

    case DFW_MUX_OFF:
    case DFW_MUX_STANDBY:
    case DFW_MUX_SHOWING:
    default:
        break;
    }
    return DFW_MUX_OK;
}

int dfw_mux_event(dfw_mux *m, int event, int source, int dt_ms)
{
    if (!m) return DFW_MUX_ERR_NULL;
    if (dt_ms > 0) (void)dfw_mux_tick(m, dt_ms);
    if (dt_ms < 0) return DFW_MUX_ERR_NULL;

    switch (event) {
    case DFW_EV_HPD_UP:
    case DFW_EV_HPD_DOWN: {
        if (source < 0 || source >= DFW_SRC_COUNT) return DFW_MUX_ERR_SOURCE;
        m->hpd_events++;
        m->hpd_jitter_events++;
        m->port[source].hpd = (event == DFW_EV_HPD_UP) ? 1 : 0;
        /* The debounce clock starts now; the state only changes after the
         * level has held long enough, which the next tick evaluates. */
        m->port[source].level_pending = m->port[source].hpd;
        m->port[source].timer_ms = 0;
        break;
    }

    case DFW_EV_USER_SELECT: {
        if (source < 0 || source >= DFW_SRC_COUNT) return DFW_MUX_ERR_SOURCE;
        if (m->state == DFW_MUX_SHOWING) {
            /* Respect the minimum dwell time so a still-training link is
             * not abandoned, but count the rejection so tests can prove
             * the policy is active rather than accidental. */
            if (m->step_timer < m->cfg.min_dwell_ms) {
                m->switch_aborted_by_dwell++;
                return DFW_MUX_OK;
            }
        }
        if (m->state == DFW_MUX_SWITCHING) {
            m->switch_aborted_by_dwell++;
            return DFW_MUX_OK;
        }
        return mux_begin_switch(m, source, DFW_REASON_USER);
    }

    case DFW_EV_TICK:
        break;

    default:
        return DFW_MUX_ERR_SOURCE;
    }
    return DFW_MUX_OK;
}

int dfw_mux_hpd(dfw_mux *m, int source, int up, int dt_ms)
{
    return dfw_mux_event(m, up ? DFW_EV_HPD_UP : DFW_EV_HPD_DOWN, source, dt_ms);
}

int dfw_mux_user_select(dfw_mux *m, int source, int dt_ms)
{
    return dfw_mux_event(m, DFW_EV_USER_SELECT, source, dt_ms);
}
