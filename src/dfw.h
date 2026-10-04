/*
 * dfw.h -- display-firmware-lab public interface
 *
 * Five host-testable modules that mirror the day-to-day jobs of a monitor
 * firmware (scaler) MCU:
 *
 *   edid.c        EDID 1.4 base-block parsing + checksum + DTD -> timing maths
 *   ddc_ci.c      DDC/CI message layer: host (PC) side and monitor (slave) side
 *   source_mux.c  input-source switching state machine with HPD / debounce
 *   osd.c         OSD menu tree state machine with idle timeout + factory mode
 *   backlight.c   PWM backlight: gamma mapping, fades, thermal derating
 *
 * Everything here is pure C99 + libc.  No hardware is touched; see README.md
 * for the honest boundary statement.
 */
#ifndef DFW_H
#define DFW_H

#include <stddef.h>
#include <stdint.h>

/* ------------------------------------------------------------------ *
 * Shared helpers
 * ------------------------------------------------------------------ */

/* Clamp v into [lo, hi].  Requires lo <= hi. */
int      dfw_clamp_i(int v, int lo, int hi);
uint32_t dfw_clamp_u32(uint32_t v, uint32_t lo, uint32_t hi);
/* Monotone linear interpolation used only for control (not timing) values. */
double   dfw_lerp(double a, double b, double t);

/* ================================================================== *
 * 1. EDID
 * ================================================================== */

#define DFW_EDID_BLOCK_SIZE 128u
#define DFW_EDID_BASE_DTDS  4u   /* 4 x 18-byte descriptor slots          */
#define DFW_EDID_STD_TIMINGS 8u  /* 8 x 2-byte standard timing entries    */
#define DFW_EDID_MAX_DTDS   64u  /* room for base + one extension block   */
#define DFW_EDID_MAX_MODES  96u

/* EDID parser status codes.  Negative = failure, 0 = success. */
enum {
    DFW_EDID_OK                =   0,
    DFW_EDID_E_NULL            = -1,
    DFW_EDID_E_TOO_SHORT       = -2,  /* fewer than 128 bytes supplied        */
    DFW_EDID_E_HEADER          = -3,  /* magic 00 FF FF FF FF FF FF 00 broken */
    DFW_EDID_E_CHECKSUM        = -4,  /* byte sum not congruent 0 mod 256     */
    DFW_EDID_E_VERSION         = -5,  /* revision < 3                         */
    DFW_EDID_E_MFG_ID          = -6,  /* manufacturer code letters out of range*/
    DFW_EDID_E_DTD_PCLK_ZERO   = -7,  /* all four slots are non-timing        */
    DFW_EDID_E_DTD_RANGE       = -8,  /* DTD produced an impossible geometry  */
    DFW_EDID_E_DTD_DUPLICATE   = -9   /* same mode advertised twice           */
};

const char *dfw_edid_status_str(int status);

/* One Detailed Timing Descriptor, already decoded into engineering units. */
typedef struct {
    double   pixel_clock_mhz;
    uint32_t h_active, h_blank, h_sync_off, h_sync_width;
    uint32_t v_active, v_blank, v_sync_off, v_sync_width;
    uint32_t h_size_mm, v_size_mm, h_border, v_border;
    uint32_t h_total, v_total;
    double   refresh_hz;      /* pclk / (htotal * vtotal)                    */
    double   h_freq_khz;      /* pclk / htotal                               */
    double   v_freq_hz;       /* pclk / (htotal * vtotal), recomputed        */
    int      interlaced;
    int      h_sync_positive, v_sync_positive;
    /* bits 7..6 of DTD byte 17: 00 analog, 01 bipolar analog,
     * 10 digital separate, 11 digital composite                        */
    int      sync_type;
    int      sync_serration, sync_on_green;
} dfw_edid_dtd;

/* Monitor descriptor (18-byte descriptor whose pixel clock is 0). */
enum {
    DFW_DESC_NONE = 0,
    DFW_DESC_SERIAL,
    DFW_DESC_ASCII,
    DFW_DESC_RANGE_LIMITS,
    DFW_DESC_NAME,
    DFW_DESC_COLOR_POINT,
    DFW_DESC_STD_TIMING_IDS,
    DFW_DESC_DUMMY,
    DFW_DESC_OTHER
};

typedef struct {
    int      type;
    char     text[14];        /* serial / name / generic ASCII string         */
    /* used by DFW_DESC_RANGE_LIMITS */
    double   v_min_hz, v_max_hz, h_min_khz, h_max_khz, max_pixel_clock_mhz;
    int      has_range;
} dfw_edid_desc;

/* Standard timing entry (2 bytes) as advertised in bytes 38..53. */
typedef struct {
    uint32_t h_active;
    uint32_t v_active;
    double   refresh_hz;
    int      ratio_code;      /* bits 7..6 of byte 1                          */
} dfw_edid_std_timing;

typedef struct {
    int      valid;
    int      status;          /* DFW_EDID_OK or DFW_EDID_E_*                  */
    uint8_t  block[DFW_EDID_BLOCK_SIZE];

    uint16_t mfg_id;          /* packed 5-bit triplet                         */
    char     mfg_letters[4];  /* "DEL", "AOC", ... + NUL                      */
    uint16_t product_code;
    uint32_t serial_number;
    uint8_t  week, year;      /* year is "year since 1990"                    */

    uint8_t  version, revision;
    uint8_t  input_type;      /* 1 digital, 0 analog                          */
    uint8_t  bit_depth;       /* 6/8/10/12/14/16 (0 if analog)                */
    uint8_t  interface_type;  /* 0 undefined,1 DVI,2 HDMI-a,3 HDMI-b,4 MDDI,5 DP */
    uint8_t  video_input_byte;

    int      supports_dpms_standby, supports_dpms_suspend,
             supports_dpms_active_off;
    int      srgb_default, preferred_timing_is_native,
             continuous_frequency, has_extended_timings;

    double   gamma;           /* 0.0 when "defined in extension block"        */

    /* chromaticity in xy, 0..1 */
    double   red_x, red_y, green_x, green_y, blue_x, blue_y, white_x, white_y;

    uint8_t  established_timings[3];
    uint32_t image_w_mm, image_h_mm;
    double   aspect_ratio;         /* 0.0 when undefined                     */
    double   diagonal_inch;        /* 0.0 when both dimensions are 0         */

    dfw_edid_desc      descs[DFW_EDID_BASE_DTDS];
    dfw_edid_dtd       dtds[DFW_EDID_MAX_DTDS];
    size_t             dtd_count;

    dfw_edid_std_timing std_timings[DFW_EDID_STD_TIMINGS];
    size_t              std_count;

    uint8_t  extensions;
    int      has_cea861;      /* extension tag 0x02                           */
    int      has_displayid;   /* extension tag 0x70                           */
} dfw_edid;

/* A mode the firmware is expected to support, e.g. 1920x1080 @ 60 Hz. */
typedef struct {
    uint32_t h_active, v_active;
    double   refresh_hz;
} dfw_mode;

/* Cross-check the parsed EDID against a candidate pool of common modes and
 * return the ones the EDID really advertises: a candidate counts only if it
 * matches a DTD, a standard timing, or a set bit in the established-timings
 * bitmap.  The result is a function of the parsed bytes, not a constant
 * table, so a corrupted EDID shrinks the list.
 * Returns the number of modes written to `expected`.
 */
size_t dfw_edid_expected_modes(const dfw_edid *e, dfw_mode *expected, size_t cap);

/* Decode one 18-byte DTD.  Returns DFW_EDID_OK, or a negative status:
 * DFW_EDID_E_DTD_PCLK_ZERO when the slot is really a monitor descriptor. */
int dfw_edid_decode_dtd(const uint8_t *block18, dfw_edid_dtd *out);

/* Parse a base block.  Accepts DFW_EDID_BLOCK_SIZE bytes and, when the
 * extension count is non-zero, optional follow-on 128-byte blocks so that
 * extension-block DTDs also land in edid->dtds[].
 *
 * `len` must be >= 128.  The base block is fully validated (header, version,
 * checksum); extension blocks are only checksum-tested and mined for DTDs.
 */
int dfw_edid_parse(const uint8_t *data, size_t len, dfw_edid *out);

/* Recompute the 128-byte checksum (byte 127 = 256 - sum(bytes 0..126)). */
uint8_t dfw_edid_compute_checksum(const uint8_t *block);
/* Non-zero when sum(block[0..127]) % 256 == 0. */
int     dfw_edid_checksum_ok(const uint8_t *block);
/* Write the correct checksum into block[127] in place. */
void    dfw_edid_fix_checksum(uint8_t *block);

/* --- synthetic EDID construction, used by tests and the simulator ------ */
/* These are deliberately plain builders: the test suite assembles blocks
 * byte by byte so that "the parser found it" is never the same code that
 * "wrote it". */
void dfw_edid_build_dtd(uint8_t *block18, const dfw_edid_dtd *spec);
void dfw_edid_build_desc_name(uint8_t *block18, const char *name);
void dfw_edid_build_desc_serial(uint8_t *block18, const char *serial);
void dfw_edid_build_desc_range(uint8_t *block18, double v_min, double v_max,
                               double h_min_khz, double h_max_khz,
                               double max_pclk_mhz);

/* ================================================================== *
 * 2. DDC/CI
 * ================================================================== */

enum {
    DFW_DDC_SRC_HOST      = 0x51,  /* host (PC) source address, 7-bit << 1 */
    DFW_DDC_DST_DISPLAY   = 0x6E,  /* display destination address          */
    DFW_DDC_SRC_DISPLAY   = 0x6E,  /* display source (for replies)         */
    DFW_DDC_DST_HOST      = 0x51
};

enum {
    DFW_DDC_GET_VCP        = 0x01,
    DFW_DDC_GET_VCP_REPLY  = 0x02,
    DFW_DDC_SET_VCP        = 0x03,
    DFW_DDC_SAVE_SETTINGS  = 0x0C,
    DFW_DDC_GET_CAPABILITIES = 0xF3,
    DFW_DDC_CAP_REPLY      = 0xE3
};

enum {
    DFW_DDC_OP_OK             = 0x00,
    DFW_DDC_OP_UNSUPPORTED    = 0x01,
    DFW_DDC_OP_CHECKSUM_ERROR = 0x81
};

enum {
    DFW_DDC_MODE_SET_PARAM  = 0x00,
    DFW_DDC_MODE_MOMENTARY  = 0x01,
    DFW_DDC_MODE_READ_ONLY  = 0x02
};

/* Well-known VCP codes. */
enum {
    DFW_VCP_BRIGHTNESS   = 0x10,
    DFW_VCP_CONTRAST     = 0x12,
    DFW_VCP_COLOR_PRESET = 0x14,
    DFW_VCP_INPUT_SOURCE = 0x60,
    DFW_VCP_POWER_MODE   = 0xD6,
    DFW_VCP_VERSION      = 0xDF,
    DFW_VCP_MUTE         = 0x8D
};

/* VCP values 0x0000..0x00FF are "table values" (one byte of meaning);
 * 0x0100..0xFFFF are continuous values.  Getting this wrong makes the
 * message-type nibble wrong, so the constant is spelled out here. */
#define DFW_VCP_CONTINUOUS_BASE 0x0100u

/* Transaction / bus level error codes. */
enum {
    DFW_DDC_OK             =  0,
    DFW_DDC_E_NULL         = -1,
    DFW_DDC_E_BAD_DST      = -2,
    DFW_DDC_E_BAD_LENGTH   = -3,
    DFW_DDC_E_CHECKSUM     = -4,
    DFW_DDC_E_TIMEOUT      = -5,
    DFW_DDC_E_TRUNCATED    = -6,
    DFW_DDC_E_NO_RESPONSE  = -7,
    DFW_DDC_E_IO           = -8,
    DFW_DDC_E_BAD_OPCODE   = -9,
    DFW_DDC_E_NO_SPACE     = -10
};

const char *dfw_ddc_status_str(int status);

/* --- monitor (slave) side --------------------------------------------- */

typedef struct {
    uint16_t code;
    uint16_t value;
    uint16_t max_value;
    uint8_t  type;      /* DFW_DDC_MODE_*                                  */
    int      supported;
} dfw_vcp_entry;

typedef struct {
    uint8_t      address;      /* expected destination address, default 0x6E */
    uint8_t      host_address; /* replies are sent here, default 0x51       */
    dfw_vcp_entry vcp[16];
    size_t       vcp_count;
    int          powered_on;
    int          reply_delay_ticks;   /* simulated slave latency             */
    int          nak_next;            /* force a NAK on the next request      */
    int          drop_next_checksum;  /* mangle the next reply checksum       */
    int          abort_next;          /* abort the transaction (host timeout)*/
    uint32_t     transactions;        /* well-formed requests seen            */
    uint32_t     checksum_errors;     /* requests rejected for bad checksum   */
    uint32_t     unsupported_requests;
    uint32_t     malformed_requests;
    uint32_t     saves;
    char         capabilities[128];
} dfw_ddc_slave;

void dfw_ddc_slave_init(dfw_ddc_slave *s);
/* Add a control to the monitor's VCP table.  Pass `type` = 0xFF to have the
 * MCCS default for that well-known code applied (continuous vs table vs
 * read-only); any other value is stored verbatim. */
int  dfw_ddc_slave_add_vcp(dfw_ddc_slave *s, uint16_t code, uint16_t value,
                           uint16_t max_value, uint8_t type);
dfw_vcp_entry *dfw_ddc_slave_find(dfw_ddc_slave *s, uint16_t code);

/* Build a DDC/CI packet: [dst][src][len][payload...][xor].
 * `len` = 2 + payload_len.  Returns DFW_DDC_OK or a negative status. */
int  dfw_ddc_build(uint8_t *out, size_t cap, size_t *out_len,
                   uint8_t dst, uint8_t src,
                   const uint8_t *payload, size_t payload_len);

/* XOR of bytes [0..len-2] inclusive must equal byte[len-1]. */
uint8_t dfw_ddc_xor_checksum(const uint8_t *buf, size_t len);
int     dfw_ddc_verify_checksum(const uint8_t *buf, size_t len);

/* Slave request handler.  Writes the reply packet (may be zero bytes for
 * NAK / aborted transactions) into `reply`.
 * `reply_delay_ticks` is surfaced through `*delay_ticks_out` so the host's
 * retry logic can be exercised without real time passing. */
int dfw_ddc_slave_handle(dfw_ddc_slave *s,
                         const uint8_t *req, size_t req_len,
                         uint8_t *reply, size_t reply_cap, size_t *reply_len,
                         int *delay_ticks_out);

/* --- host (PC) side ---------------------------------------------------- */

typedef struct {
    uint16_t vcp_code;
    uint8_t  result_code;    /* DFW_DDC_OP_*                                */
    uint8_t  message_type;
    uint16_t value;
    uint16_t max_value;
} dfw_ddc_reply;

typedef struct {
    uint32_t transactions;    /* replies successfully parsed                 */
    uint32_t checksum_errors; /* replies (or requests) that failed XOR       */
    uint32_t timeouts;        /* attempts that got no reply in time          */
    uint32_t retries;         /* extra attempts after the first              */
    uint32_t unsupported;     /* VCP codes the monitor said it does not have */
    uint32_t saves;
} dfw_ddc_stats;

/* A simulated DDC bus.  There is no I2C here: a call is synchronous, and
 * `max_retries` / `timeout_ticks` exist so the retry policy is real code
 * rather than a comment. */
typedef struct {
    dfw_ddc_slave *slave;
    int            max_retries;    /* extra attempts, default 2             */
    int            timeout_ticks;  /* default 40 (DDC/CI allows ~40 ms)     */
    dfw_ddc_stats  stats;
} dfw_ddc_bus;

void dfw_ddc_bus_init(dfw_ddc_bus *bus, dfw_ddc_slave *slave);

int dfw_ddc_get_vcp(dfw_ddc_bus *bus, uint16_t code, dfw_ddc_reply *out);
int dfw_ddc_set_vcp(dfw_ddc_bus *bus, uint16_t code, uint16_t value);
int dfw_ddc_save_settings(dfw_ddc_bus *bus);
/* One raw request/response round trip, including the retry policy. */
int dfw_ddc_host_transact(dfw_ddc_bus *bus,
                          const uint8_t *payload, size_t payload_len,
                          uint8_t *reply, size_t reply_cap, size_t *reply_len);

/* ================================================================== *
 * 3. Input source multiplexer
 * ================================================================== */

enum {
    DFW_SRC_HDMI1 = 0, DFW_SRC_HDMI2, DFW_SRC_DP, DFW_SRC_TYPEC, DFW_SRC_VGA,
    DFW_SRC_COUNT
};

enum {
    DFW_MUX_OFF = 0, DFW_MUX_STANDBY, DFW_MUX_SHOWING,
    DFW_MUX_SWITCHING, DFW_MUX_NO_SIGNAL
};

enum { DFW_EV_HPD_UP = 0, DFW_EV_HPD_DOWN, DFW_EV_USER_SELECT, DFW_EV_TICK };

enum { DFW_MUX_OK = 0, DFW_MUX_ERR_NULL = -1, DFW_MUX_ERR_SOURCE = -2 };

typedef struct {
    int      debounce_ms[DFW_SRC_COUNT];   /* default 120                     */
    int      min_dwell_ms;                 /* 1000: no re-switch before this  */
    int      settle_ms;                    /* 200: sink blank + PLL settle    */
    int      enable_ms;                    /* 300: wait for valid timing      */
    int      verify_ms;                    /* 200: confirm signal is stable   */
    int      switch_timeout_ms;            /* 2500: give up, roll back        */
    int      search_dwell_ms;              /* 400: linger per candidate       */
    int      order[DFW_SRC_COUNT];         /* search priority, default HDMI1,HDMI2,DP,TYPEC,VGA */
} dfw_mux_cfg;

typedef struct {
    int      hpd;                 /* instantaneous level (may be jittering) */
    int      active;              /* debounced, stable level                */
    int      level_pending;       /* level seen during a pending debounce   */
    int      timer_ms;            /* ms the pending level has held          */
    int      cap_score;           /* 0..100 capability/link quality score   */
} dfw_mux_port;

typedef struct {
    dfw_mux_cfg  cfg;
    dfw_mux_port port[DFW_SRC_COUNT];
    int          state;
    int          current;         /* valid when state == SHOWING/NO_SIGNAL    */
    int          previous;        /* rollback target                          */
    int          target;
    int          search_from;
    int          candidate;       /* source being probed while searching      */
    int          candidate_timer;
    uint32_t     now_ms;

    /* bookkeeping / metrics */
    uint32_t     switch_attempts;
    uint32_t     switch_ok;
    uint32_t     switch_timeout;
    uint32_t     switch_aborted_by_dwell;
    uint32_t     hpd_edges_ignored;
    uint32_t     hpd_events;
    int          switch_step;     /* 0 idle, 1 disable, 2 settle, 3 enable, 4 verify */
    int          step_timer;
    int          switch_elapsed;  /* clock for the whole switch sequence        */
    int          logs[64][4];     /* {time, from, to, reason}                 */
    size_t       log_count;

    /* observed sequence of (source, hpd_level) used to prove debouncing */
    uint32_t     hpd_jitter_events;
} dfw_mux;

/* reason codes recorded in the switch log */
enum { DFW_REASON_BOOT=0, DFW_REASON_USER, DFW_REASON_HPD, DFW_REASON_AUTO,
       DFW_REASON_FALLBACK, DFW_REASON_STANDBY, DFW_REASON_ROLLBACK };

const char *dfw_src_name(int src);
const char *dfw_mux_state_name(int state);
const char *dfw_mux_reason_name(int reason);

void dfw_mux_cfg_default(dfw_mux_cfg *cfg);
void dfw_mux_init(dfw_mux *m, const dfw_mux_cfg *cfg);

/* Feed one event with a millisecond delta.  Returns DFW_MUX_OK or negative. */
int  dfw_mux_event(dfw_mux *m, int event, int source, int dt_ms);

/* Convenience wrappers (they just emit the event above). */
int  dfw_mux_hpd(dfw_mux *m, int source, int up, int dt_ms);
int  dfw_mux_user_select(dfw_mux *m, int source, int dt_ms);
int  dfw_mux_tick(dfw_mux *m, int dt_ms);

int  dfw_mux_is_present(const dfw_mux *m, int source);
int  dfw_mux_first_available(const dfw_mux *m, int from);
int  dfw_mux_prev_in_order(const dfw_mux *m, int src);

/* ================================================================== *
 * 4. OSD
 * ================================================================== */

enum { DFW_KEY_UP=0, DFW_KEY_DOWN, DFW_KEY_LEFT, DFW_KEY_RIGHT,
       DFW_KEY_OK, DFW_KEY_BACK };

enum { DFW_KEY_PRESS=0, DFW_KEY_HOLD, DFW_KEY_RELEASE };

enum { DFW_OSD_NONE=0, DFW_OSD_BRIGHTNESS, DFW_OSD_CONTRAST, DFW_OSD_INPUT,
       DFW_OSD_LANGUAGE, DFW_OSD_FACTORY_RESET, DFW_OSD_EXIT, DFW_OSD_ITEM_COUNT };

enum { DFW_OSD_CLOSED=0, DFW_OSD_TOP, DFW_OSD_ITEM, DFW_OSD_SELECTING };

enum { DFW_POWER_OFF=0, DFW_POWER_ON };

enum { DFW_LANG_EN=0, DFW_LANG_ZH, DFW_LANG_JA, DFW_LANG_DE, DFW_LANG_COUNT };

#define DFW_OSD_MAX_IGNORED 16

typedef struct {
    int id;
    const char *name;
    int vmin, vmax, vdef;
    int kind;          /* 0 = numeric, 1 = enum, 2 = action               */
    int step;          /* for numeric items                               */
    int unit_permille; /* display-only granularity, 0 for enums/actions   */
} dfw_osd_item;

typedef struct {
    int vcp_code;
    int input_source;      /* DFW_SRC_*; ignored when input_from_vcp == 0 */
    int input_from_vcp;
    int step;
    int lang;
    int lang_count;
    /* long-press repeat policy, used for numeric items */
    int repeat_delay_ms;
    int repeat_period_ms;
} dfw_osd_settings;

typedef struct {
    const dfw_osd_item *items;
    size_t    item_count;
    int       idle_timeout_ms;      /* default 15000                        */
    int       factory_timeout_ms;   /* default 60000                        */
    int       factory_hold_ms;      /* default 5000                         */
    int       factory_window_ms;    /* default 2000                         */
} dfw_osd_cfg;

/* Optional hooks so the OSD can talk to the rest of the firmware. */
typedef struct {
    void (*write_vcp)(void *ctx, int code, int value);
    void (*save_settings)(void *ctx);
    int  (*backlight_max)(void *ctx);   /* dynamic clamp for brightness     */
    void (*switch_input)(void *ctx, int source);
    void (*power_off)(void *ctx);
    void *ctx;
} dfw_osd_io;

typedef struct {
    int key;
    int source;     /* for the BACK/OK "source" overload, unused otherwise */
    int dt_ms;      /* time since the previous event                      */
    int phase;      /* DFW_KEY_PRESS / HOLD / RELEASE                     */
} dfw_osd_key_event;

typedef struct {
    /* ---- configuration copy ---- */
    const dfw_osd_item *items;
    size_t      item_count;
    dfw_osd_cfg cfg;
    dfw_osd_settings set;
    dfw_osd_io  io;

    /* ---- runtime ---- */
    int      power;
    int      state;
    int      menu_index;      /* cursor in the top level                     */
    int      value;
    int      saved_value;
    int      dirty;
    int      selecting;       /* enum picker open                            */
    int      select_index;
    int      last_input;

    /* ---- factory mode entry sequence ---- */
    int      fs_armed;        /* OK is being held while powered off          */
    int      fs_hold_ms;
    int      fs_window;       /* ms left after the OK release                */
    int      factory_mode;

    /* ---- timing / diagnostics ---- */
    uint32_t now_ms;
    uint32_t last_activity_ms;
    uint32_t open_count;
    uint32_t close_timeout_count;
    uint32_t close_manual_count;
    uint32_t ignored_count;
    uint32_t reject_count;    /* key presses rejected in the current state   */
    int      ignored_log[DFW_OSD_MAX_IGNORED][3];
    uint32_t factory_reject_count;
    int      key_down_ms;     /* for repeat handling                         */
    int      repeat_accum_ms;
} dfw_osd;

extern const dfw_osd_item dfw_osd_default_items[DFW_OSD_ITEM_COUNT];

void dfw_osd_settings_default(dfw_osd_settings *s);
void dfw_osd_cfg_default(dfw_osd_cfg *cfg);
void dfw_osd_init(dfw_osd *o, const dfw_osd_cfg *cfg, const dfw_osd_settings *set,
                  const dfw_osd_io *io);
void dfw_osd_power(dfw_osd *o, int on);

/* Deliver one key event.  Returns DFW_OSD_OK or negative. */
int  dfw_osd_key(dfw_osd *o, int key, int phase, int dt_ms);

/* Advance time without any key.  Handles the idle timeout and the factory
 * entry window.  Returns the number of state changes applied. */
int  dfw_osd_tick(dfw_osd *o, int dt_ms);

const dfw_osd_item *dfw_osd_item_by_id(const dfw_osd *o, int id);
const char *dfw_osd_item_name(int id);
const char *dfw_osd_state_name(int state);
#define DFW_OSD_OK 0
#define DFW_OSD_ERR_NULL (-1)
#define DFW_OSD_ERR_KEY  (-2)

/* ================================================================== *
 * 5. Backlight PWM
 * ================================================================== */

typedef struct {
    double   gamma;            /* default 2.2                               */
    double   min_duty_percent; /* default 1.0: never fully off at bri=0     */
    unsigned pwm_counts;       /* default 4095 -> 12-bit resolution         */
    unsigned pwm_hz;           /* default 20000                             */
    int      max_percent;      /* user brightness ceiling, default 100      */
    int      fade_ms;          /* default 300                               */
    /* thermal derating: apply cap[level] to the user brightness */
    int      temp_c;           /* current panel temperature                 */
    int      derate_temp_c;    /* default 70                                */
    int      cutoff_temp_c;    /* default 85                                */
    int      cap_warm_percent; /* default 70                                */
    int      cap_hot_percent;  /* default 40                                */
} dfw_bl_cfg;

typedef struct {
    dfw_bl_cfg cfg;
    int        brightness;      /* user request 0..100                      */
    int        output_brightness;/* after thermal derating                  */
    double     duty_percent;
    unsigned   duty_counts;
    unsigned   period_counts;
    double     step_resolution_percent;
    double     pwm_period_us;
    unsigned   lut[101];
    int        fading;
    unsigned   fade_from_counts;
    unsigned   fade_to_counts;
    int        fade_elapsed_ms;
    int        fade_total_ms;
    uint32_t   fade_count;
    uint32_t   transitions;
    uint32_t   derate_events;
    int        last_delta_counts;
} dfw_bl;

void   dfw_bl_cfg_default(dfw_bl_cfg *cfg);
double dfw_bl_gamma_encode(const dfw_bl_cfg *cfg, int brightness_percent);
unsigned dfw_bl_duty_counts(const dfw_bl_cfg *cfg, double duty_percent);
void   dfw_bl_init(dfw_bl *b, const dfw_bl_cfg *cfg);
void   dfw_bl_set_brightness(dfw_bl *b, int brightness);   /* instant, LUT based */
int    dfw_bl_fade_to(dfw_bl *b, int target_percent);      /* ramped over fade_ms */
void   dfw_bl_tick(dfw_bl *b, int dt_ms);
void   dfw_bl_set_temp(dfw_bl *b, int temp_c);
int    dfw_bl_derate_level(const dfw_bl *b);
/* Returns the percentage actually applied, and records a derate event when
 * the thermal cap is what limited the output. */
int    dfw_bl_apply_thermal(dfw_bl *b);
int    dfw_bl_is_monotonic(const dfw_bl *b);

/* ================================================================== *
 * Simulator entry point (main_sim.c)
 * ================================================================== */
int dfw_sim_run(const char *out_dir);

#endif /* DFW_H */
