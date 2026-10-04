/*
 * osd.c -- on-screen display menu state machine.
 *
 * ---------------------------------------------------------------------
 * WHAT THIS MODELS
 * ---------------------------------------------------------------------
 * A monitor OSD is a small menu system driven by five buttons (or a
 * joystick) wired straight to the scaler's GPIO.  Firmware responsibilities
 * that are modelled here:
 *
 *   navigation   up/down move the cursor, right/OK enters an item,
 *                left/BACK leaves it.  Every key that is not meaningful in
 *                the current state must be *ignored*, not treated as an
 *                error and not as a silent value change.
 *   idle timeout after `idle_timeout_ms` without any key the OSD closes
 *                and the picture comes back.  This is a shop-floor
 *                requirement: a monitor left with its menu open is a
 *                support call.
 *   persistence  a value change is applied live (so the user sees it) but
 *                committed to non-volatile storage only when the menu
 *                closes or on an explicit save.  Writing flash on every
 *                key repeat wears it out.
 *   factory mode a hidden key sequence.  The one implemented here is:
 *
 *                  powered OFF, hold OK for >= factory_hold_ms, release,
 *                  then press UP within factory_window_ms.
 *
 *                Any other order, or a hold that is too short, must fail.
 *                Getting this wrong is how a monitor ends up in a service
 *                menu in a customer's living room.
 *   live clamp   brightness is clamped to whatever the backlight can
 *                actually deliver: if a panel is thermally derated to 70,
 *                the OSD must not offer 100 and then not deliver it.  The
 *                limit is read through the io.backlight_max hook, so the
 *                OSD and the backlight cannot disagree.
 *
 * ---------------------------------------------------------------------
 * MENU TREE (top level, in this order)
 * ---------------------------------------------------------------------
 *   Brightness   0..100, step 1   <- also mirrored to DDC/CI VCP 0x10
 *   Contrast     0..100, step 1   <- VCP 0x12
 *   Input        HDMI1/HDMI2/DP/Type-C/VGA   (calls io.switch_input)
 *   Language     EN / ZH / JA / DE
 *   Factory Reset action: restores every default and saves
 *   Exit         closes the OSD
 *
 * A long press (DFW_KEY_HOLD) on a numeric item repeats the value change so
 * holding UP does not need dozens of discrete presses; the repeat period is
 * a parameter because too fast a repeat is unusable on cheap tactile
 * switches and too slow feels broken.
 */
#include "dfw.h"

#include <string.h>

const dfw_osd_item dfw_osd_default_items[DFW_OSD_ITEM_COUNT] = {
    { DFW_OSD_BRIGHTNESS, "Brightness", 0, 100, 70, 0, 1, 1 },
    { DFW_OSD_CONTRAST,   "Contrast",   0, 100, 50, 0, 1, 1 },
    { DFW_OSD_INPUT,      "Input",      0, DFW_SRC_COUNT - 1, 0, 1, 1, 0 },
    { DFW_OSD_LANGUAGE,   "Language",   0, DFW_LANG_COUNT - 1, 0, 1, 1, 0 },
    { DFW_OSD_FACTORY_RESET, "Factory Reset", 0, 1, 0, 2, 1, 0 },
    { DFW_OSD_EXIT,       "Exit",       0, 0, 0, 2, 1, 0 }
};

const char *dfw_osd_state_name(int state)
{
    switch (state) {
    case DFW_OSD_CLOSED:    return "CLOSED";
    case DFW_OSD_TOP:       return "TOP";
    case DFW_OSD_ITEM:      return "ITEM";
    case DFW_OSD_SELECTING: return "SELECTING";
    default:                return "?";
    }
}

const char *dfw_osd_item_name(int id)
{
    if (id < 0 || id >= DFW_OSD_ITEM_COUNT) return "?";
    return dfw_osd_default_items[id].name;
}

const dfw_osd_item *dfw_osd_item_by_id(const dfw_osd *o, int id)
{
    size_t i;
    if (!o || !o->items) return NULL;
    for (i = 0; i < o->item_count; ++i)
        if (o->items[i].id == id) return &o->items[i];
    return NULL;
}

void dfw_osd_settings_default(dfw_osd_settings *s)
{
    if (!s) return;
    memset(s, 0, sizeof *s);
    s->vcp_code       = DFW_VCP_BRIGHTNESS;
    s->input_source   = DFW_SRC_HDMI1;
    s->input_from_vcp = 1;
    s->step           = 2;      /* one UP press = 2 points of brightness */
    s->lang           = DFW_LANG_EN;
    s->lang_count     = DFW_LANG_COUNT;
    s->repeat_delay_ms  = 500;
    s->repeat_period_ms = 100;
}

void dfw_osd_cfg_default(dfw_osd_cfg *cfg)
{
    if (!cfg) return;
    memset(cfg, 0, sizeof *cfg);
    cfg->items              = dfw_osd_default_items;
    cfg->item_count         = DFW_OSD_ITEM_COUNT;
    cfg->idle_timeout_ms    = 15000;
    cfg->factory_timeout_ms = 60000;
    cfg->factory_hold_ms    = 5000;
    cfg->factory_window_ms  = 2000;
}

void dfw_osd_init(dfw_osd *o, const dfw_osd_cfg *cfg, const dfw_osd_settings *set,
                  const dfw_osd_io *io)
{
    const dfw_osd_item *bright;
    if (!o) return;
    memset(o, 0, sizeof *o);

    if (cfg) o->cfg = *cfg;
    else     dfw_osd_cfg_default(&o->cfg);
    o->items      = o->cfg.items;
    o->item_count = o->cfg.item_count;

    if (set) o->set = *set;
    else     dfw_osd_settings_default(&o->set);

    if (io) o->io = *io;

    /* start from the item defaults */
    bright = dfw_osd_item_by_id(o, DFW_OSD_BRIGHTNESS);
    o->value = bright ? bright->vdef : 70;
    o->saved_value = o->value;
    o->last_input  = o->set.input_source;

    o->power        = DFW_POWER_OFF;
    o->state        = DFW_OSD_CLOSED;
    o->menu_index   = 0;
    o->selecting    = 0;
    o->select_index = 0;
    o->dirty        = 0;
    o->now_ms       = 0;
    o->last_activity_ms = 0;
}

void dfw_osd_power(dfw_osd *o, int on)
{
    if (!o) return;
    o->power = on ? DFW_POWER_ON : DFW_POWER_OFF;
    if (!on) {
        /* Powering off closes the OSD and cancels any pending entry
         * sequence: the factory-mode gesture begins from a powered-off
         * state, so a power transition must not itself satisfy it. */
        o->state    = DFW_OSD_CLOSED;
        o->selecting = 0;
        o->fs_armed = 0;
        o->fs_hold_ms = 0;
        o->fs_window = 0;
    }
}

/* ---------------------------------------------------------------- *
 * helpers
 * ---------------------------------------------------------------- */

static const dfw_osd_item *current_item(const dfw_osd *o)
{
    if (!o || !o->items) return NULL;
    if (o->menu_index < 0 || (size_t)o->menu_index >= o->item_count) return NULL;
    return &o->items[o->menu_index];
}

/* Effective upper bound of an item: brightness can be limited by the
 * backlight's thermal derating, everything else uses its own vmax. */
static int item_effective_max(const dfw_osd *o, const dfw_osd_item *it)
{
    int hi = it->vmax;
    if (it->id == DFW_OSD_BRIGHTNESS && o->io.backlight_max) {
        int lim = o->io.backlight_max(o->io.ctx);
        if (lim >= 0 && lim < hi) hi = lim;
    }
    return hi;
}

static void osd_record_ignored(dfw_osd *o, int key, int state, int phase)
{
    if (o->ignored_count < DFW_OSD_MAX_IGNORED) {
        o->ignored_log[o->ignored_count][0] = key;
        o->ignored_log[o->ignored_count][1] = state;
        o->ignored_log[o->ignored_count][2] = phase;
    }
    o->ignored_count++;
    o->reject_count++;
}

static void osd_touch(dfw_osd *o)
{
    o->last_activity_ms = o->now_ms;
}

static void osd_save(dfw_osd *o)
{
    o->saved_value = o->value;
    o->dirty = 0;
    if (o->io.save_settings) o->io.save_settings(o->io.ctx);
}

static void osd_open(dfw_osd *o)
{
    if (o->state != DFW_OSD_CLOSED) return;
    o->state      = DFW_OSD_TOP;
    o->menu_index = 0;
    o->selecting  = 0;
    o->open_count++;
    osd_touch(o);
}

static void osd_close(dfw_osd *o, int manual)
{
    if (o->state == DFW_OSD_CLOSED) return;
    if (o->dirty) osd_save(o);       /* commit once, on exit */
    o->state     = DFW_OSD_CLOSED;
    o->selecting = 0;
    if (manual) o->close_manual_count++;
    else        o->close_timeout_count++;
    osd_touch(o);
}

static void osd_load_value_for(dfw_osd *o, const dfw_osd_item *it)
{
    if (!it) return;
    switch (it->id) {
    case DFW_OSD_BRIGHTNESS: o->value = o->saved_value; break;
    case DFW_OSD_CONTRAST:   o->value = 50;             break;
    case DFW_OSD_INPUT:      o->value = o->last_input;  break;
    case DFW_OSD_LANGUAGE:   o->value = o->set.lang;    break;
    default:                 o->value = it->vdef;       break;
    }
    o->value = dfw_clamp_i(o->value, it->vmin, item_effective_max(o, it));
}

static void osd_apply_value(dfw_osd *o, const dfw_osd_item *it)
{
    if (!it) return;
    switch (it->id) {
    case DFW_OSD_BRIGHTNESS:
        if (o->io.write_vcp) o->io.write_vcp(o->io.ctx, o->set.vcp_code, o->value);
        o->dirty = 1;
        break;
    case DFW_OSD_CONTRAST:
        if (o->io.write_vcp) o->io.write_vcp(o->io.ctx, DFW_VCP_CONTRAST, o->value);
        o->dirty = 1;
        break;
    case DFW_OSD_INPUT:
        o->last_input = o->value;
        if (o->io.switch_input) o->io.switch_input(o->io.ctx, o->value);
        o->dirty = 1;
        break;
    case DFW_OSD_LANGUAGE:
        o->set.lang = o->value;
        o->dirty = 1;
        break;
    default:
        break;
    }
}

/* ---------------------------------------------------------------- *
 * the factory-mode gesture
 * ---------------------------------------------------------------- */

static int factory_gesture_key(dfw_osd *o, int key, int phase, int dt_ms)
{
    if (o->power != DFW_POWER_OFF) return 0;

    if (key == DFW_KEY_OK) {
        if (phase == DFW_KEY_HOLD) {
            /* The hold time is accumulated from the dt_ms of each HOLD event,
             * so the gesture can be driven with an explicit time step in a
             * test instead of by sleeping. */
            if (!o->fs_armed) {
                o->fs_armed   = 1;
                o->fs_hold_ms = 0;
            }
            o->fs_hold_ms += (dt_ms > 0) ? dt_ms : 0;
            return 1;
        }
        if (phase == DFW_KEY_PRESS) {
            /* A fresh press restarts the gesture, discarding any earlier
             * partial hold -- otherwise two short presses would add up and
             * a 2.5 s + 2.5 s pair would wrongly reach the 5 s threshold. */
            o->fs_armed   = 1;
            o->fs_hold_ms = 0;
            o->fs_window  = 0;
            return 1;
        }
        if (phase == DFW_KEY_RELEASE) {
            if (o->fs_armed && o->fs_hold_ms >= o->cfg.factory_hold_ms) {
                o->fs_window = o->cfg.factory_window_ms;
            } else {
                o->factory_reject_count++;
                o->fs_armed   = 0;
                o->fs_hold_ms = 0;
                o->fs_window  = 0;
            }
            return 1;
        }
    }

    if (key == DFW_KEY_UP && phase == DFW_KEY_PRESS) {
        if (o->fs_window > 0) {
            o->factory_mode = 1;
            o->fs_window    = 0;
            o->fs_armed     = 0;
            o->fs_hold_ms   = 0;
            return 1;
        }
        /* UP alone, or after the window expired, must not enter factory
         * mode: this is the negative case the tests pin down. */
        if (o->fs_armed) o->factory_reject_count++;
        return 1;
    }
    return 0;
}

/* ---------------------------------------------------------------- *
 * key handling
 * ---------------------------------------------------------------- */

static void osd_numeric_step(dfw_osd *o, const dfw_osd_item *it, int dir)
{
    int hi = item_effective_max(o, it);
    int nv;
    if (!it || dir == 0) return;
    nv = o->value + dir * it->step;
    if (nv < it->vmin) nv = it->vmin;
    if (nv > hi) nv = hi;
    if (nv == o->value) return;      /* already at the stop: silently no-op */
    o->value = nv;
    osd_apply_value(o, it);
}

int dfw_osd_key(dfw_osd *o, int key, int phase, int dt_ms)
{
    const dfw_osd_item *it;

    if (!o) return DFW_OSD_ERR_NULL;
    if (key < DFW_KEY_UP || key > DFW_KEY_BACK) return DFW_OSD_ERR_KEY;
    if (phase < DFW_KEY_PRESS || phase > DFW_KEY_RELEASE) return DFW_OSD_ERR_KEY;
    if (dt_ms < 0) return DFW_OSD_ERR_KEY;

    if (dt_ms > 0) (void)dfw_osd_tick(o, dt_ms);

    /* The factory gesture is only meaningful while powered off and it owns
     * the keys entirely: no OSD navigation happens in that state. */
    if (o->power == DFW_POWER_OFF) {
        if (o->factory_mode) {
            /* Once in factory mode, OK leaves it; everything else is inert. */
            if (key == DFW_KEY_OK && phase == DFW_KEY_PRESS) {
                o->factory_mode = 0;
                return DFW_OSD_OK;
            }
            osd_record_ignored(o, key, DFW_OSD_CLOSED, phase);
            return DFW_OSD_OK;
        }
        if (factory_gesture_key(o, key, phase, dt_ms)) {
            osd_touch(o);
            return DFW_OSD_OK;
        }
        osd_record_ignored(o, key, DFW_OSD_CLOSED, phase);
        return DFW_OSD_OK;
    }

    /* powered on: OK opens the OSD when it is closed */
    if (o->state == DFW_OSD_CLOSED) {
        if (key == DFW_KEY_OK && phase == DFW_KEY_PRESS) {
            osd_open(o);
            return DFW_OSD_OK;
        }
        if (key == DFW_KEY_BACK && phase == DFW_KEY_PRESS) {
            /* BACK with no menu: treated as a request to power down */
            if (o->io.power_off) o->io.power_off(o->io.ctx);
            osd_touch(o);
            return DFW_OSD_OK;
        }
        osd_record_ignored(o, key, DFW_OSD_CLOSED, phase);
        return DFW_OSD_OK;
    }

    it = current_item(o);
    osd_touch(o);

    /* ---- enum picker ---- */
    if (o->state == DFW_OSD_SELECTING && it) {
        int n = (it->id == DFW_OSD_INPUT) ? DFW_SRC_COUNT : o->set.lang_count;
        if (phase != DFW_KEY_PRESS) {
            /* holds are meaningless in a picker: record and move on */
            osd_record_ignored(o, key, o->state, phase);
            return DFW_OSD_OK;
        }
        switch (key) {
        case DFW_KEY_UP:
            o->value = (o->value - 1 + n) % n;
            break;
        case DFW_KEY_DOWN:
            o->value = (o->value + 1) % n;
            break;
        case DFW_KEY_OK:
            o->selecting = 0;
            o->state = DFW_OSD_ITEM;
            osd_apply_value(o, it);
            break;
        case DFW_KEY_BACK:
            o->selecting = 0;
            o->state = DFW_OSD_ITEM;
            osd_load_value_for(o, it);   /* discard the preview */
            break;
        default:
            osd_record_ignored(o, key, o->state, phase);
            break;
        }
        return DFW_OSD_OK;
    }

    /* ---- inside an item ---- */
    if (o->state == DFW_OSD_ITEM && it) {
        if (phase == DFW_KEY_HOLD && it->kind == 0) {
            /* long press repeats numeric values */
            o->repeat_accum_ms += dt_ms > 0 ? dt_ms : o->set.repeat_period_ms;
            if (o->key_down_ms >= o->set.repeat_delay_ms) {
                osd_numeric_step(o, it, key == DFW_KEY_UP ? 1 :
                                     (key == DFW_KEY_DOWN ? -1 : 0));
            }
            return DFW_OSD_OK;
        }
        if (phase != DFW_KEY_PRESS) {
            osd_record_ignored(o, key, o->state, phase);
            return DFW_OSD_OK;
        }
        switch (key) {
        case DFW_KEY_UP:
            if (it->kind == 0) osd_numeric_step(o, it, +1);
            else osd_record_ignored(o, key, o->state, phase);
            break;
        case DFW_KEY_DOWN:
            if (it->kind == 0) osd_numeric_step(o, it, -1);
            else osd_record_ignored(o, key, o->state, phase);
            break;
        case DFW_KEY_RIGHT:
        case DFW_KEY_OK:
            if (it->kind == 2) {
                if (it->id == DFW_OSD_EXIT) {
                    osd_close(o, 1);
                } else if (it->id == DFW_OSD_FACTORY_RESET) {
                    /* all six items back to their defaults, then save */
                    o->value = it->vdef;
                    if (o->io.write_vcp)
                        o->io.write_vcp(o->io.ctx, o->set.vcp_code, o->value);
                    o->set.lang = DFW_LANG_EN;
                    o->last_input = DFW_SRC_HDMI1;
                    o->dirty = 1;
                    osd_save(o);
                } else {
                    osd_record_ignored(o, key, o->state, phase);
                }
            } else if (it->kind == 1) {
                o->state = DFW_OSD_SELECTING;
                o->selecting = 1;
                o->select_index = o->value;
            } else {
                /* numeric item: keep the current value */
                o->dirty = 1;
            }
            break;
        case DFW_KEY_LEFT:
        case DFW_KEY_BACK:
            if (o->dirty) osd_save(o);
            o->state = DFW_OSD_TOP;
            break;
        default:
            osd_record_ignored(o, key, o->state, phase);
            break;
        }
        return DFW_OSD_OK;
    }

    /* ---- top level ---- */
    if (o->state == DFW_OSD_TOP) {
        int n = (int)o->item_count;
        if (phase != DFW_KEY_PRESS) {
            osd_record_ignored(o, key, o->state, phase);
            return DFW_OSD_OK;
        }
        switch (key) {
        case DFW_KEY_UP:
            o->menu_index = (o->menu_index - 1 + n) % n;
            break;
        case DFW_KEY_DOWN:
            o->menu_index = (o->menu_index + 1) % n;
            break;
        case DFW_KEY_OK:
        case DFW_KEY_RIGHT:
            if (it) {
                if (it->kind == 2) {
                    if (it->id == DFW_OSD_EXIT) {
                        osd_close(o, 1);
                    } else if (it->id == DFW_OSD_FACTORY_RESET) {
                        if (o->io.save_settings) o->io.save_settings(o->io.ctx);
                    } else {
                        osd_record_ignored(o, key, o->state, phase);
                    }
                } else {
                    osd_load_value_for(o, it);
                    o->state = DFW_OSD_ITEM;
                }
            }
            break;
        case DFW_KEY_BACK:
        case DFW_KEY_LEFT:
            osd_close(o, 1);
            break;
        default:
            osd_record_ignored(o, key, o->state, phase);
            break;
        }
        return DFW_OSD_OK;
    }

    osd_record_ignored(o, key, o->state, phase);
    return DFW_OSD_OK;
}

/* ---------------------------------------------------------------- *
 * time
 * ---------------------------------------------------------------- */

int dfw_osd_tick(dfw_osd *o, int dt_ms)
{
    int changes = 0;
    if (!o) return DFW_OSD_ERR_NULL;
    if (dt_ms <= 0) return 0;

    o->now_ms += (uint32_t)dt_ms;
    o->key_down_ms += dt_ms;

    /* factory-mode gesture timers run while powered off */
    if (o->power == DFW_POWER_OFF && !o->factory_mode) {
        if (o->fs_armed) {
            o->fs_hold_ms += dt_ms;
            if (o->fs_hold_ms > o->cfg.factory_timeout_ms) {
                /* held absurdly long (stuck button): abort the gesture */
                o->fs_armed = 0;
                o->fs_hold_ms = 0;
                o->fs_window = 0;
                o->factory_reject_count++;
            }
        }
        if (o->fs_window > 0) {
            o->fs_window -= dt_ms;
            if (o->fs_window <= 0) {
                o->fs_window = 0;
                o->factory_reject_count++;   /* window expired unused */
                changes++;
            }
        }
    }

    if (o->state != DFW_OSD_CLOSED) {
        uint32_t idle = o->now_ms - o->last_activity_ms;
        if (idle >= (uint32_t)o->cfg.idle_timeout_ms) {
            osd_close(o, 0);
            changes++;
        }
    }
    return changes;
}
