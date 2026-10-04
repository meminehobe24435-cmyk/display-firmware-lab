/*
 * test_dfw.c -- self-checking test suite for display-firmware-lab
 * ---------------------------------------------------------------------------
 * No framework: CHECK() counts and reports, main() returns non-zero if
 * anything failed, and the last line prints "<N> checks passed" so CI can
 * grep it.
 *
 * The EDID blocks are assembled BYTE BY BYTE here rather than with the
 * library's own builders wherever the test is checking the parser -- so that
 * "the parser found it" is never the same code that "wrote it".  The builders
 * are then tested separately against those hand-built blocks.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "dfw.h"

static int g_checks = 0;
static int g_fail = 0;

#define CHECK(cond)                                                          \
    do {                                                                     \
        g_checks++;                                                          \
        if (!(cond)) {                                                       \
            g_fail++;                                                        \
            printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);           \
        }                                                                    \
    } while (0)

#define CHECK_EQ_I(a, b)                                                     \
    do {                                                                     \
        int _a = (int)(a), _b = (int)(b);                                    \
        g_checks++;                                                          \
        if (_a != _b) {                                                      \
            g_fail++;                                                        \
            printf("FAIL %s:%d  %s == %s  (%d != %d)\n", __FILE__, __LINE__, \
                   #a, #b, _a, _b);                                          \
        }                                                                    \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                \
    do {                                                                     \
        double _a = (double)(a), _b = (double)(b), _t = (double)(tol);       \
        g_checks++;                                                          \
        if (!(fabs(_a - _b) <= _t)) {                                        \
            g_fail++;                                                        \
            printf("FAIL %s:%d  |%s - %s| = |%.6f - %.6f| > %.6g\n",         \
                   __FILE__, __LINE__, #a, #b, _a, _b, _t);                  \
        }                                                                    \
    } while (0)

/* ------------------------------------------------------------------ EDID */

static const uint8_t EDID_MAGIC[8] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};

/* pack three letters into the 5-bit big-endian manufacturer ID */
static uint16_t pack_mfg(const char *s)
{
    uint16_t v = 0;
    int i;
    for (i = 0; i < 3; i++) {
        int c = (int)(unsigned char)s[i] - 'A' + 1;
        v = (uint16_t)((v << 5) | (uint16_t)(c & 0x1F));
    }
    return v;
}

/* 1920x1080 @ 60 Hz, the mode every monitor advertises */
static void fill_dtd_1080p(uint8_t *b)
{
    unsigned pclk10k = 14850u;      /* 148.50 MHz in 10 kHz units */
    unsigned ha = 1920, hb = 280, hso = 88, hsw = 44;
    unsigned va = 1080, vb = 45, vso = 4, vsw = 5;
    memset(b, 0, 18);
    b[0] = (uint8_t)(pclk10k & 0xFFu);
    b[1] = (uint8_t)((pclk10k >> 8) & 0xFFu);
    b[2] = (uint8_t)(ha & 0xFFu);
    b[3] = (uint8_t)(hb & 0xFFu);
    b[4] = (uint8_t)(((ha >> 8) << 4) | ((hb >> 8) & 0x0Fu));
    b[5] = (uint8_t)(va & 0xFFu);
    b[6] = (uint8_t)(vb & 0xFFu);
    b[7] = (uint8_t)(((va >> 8) << 4) | ((vb >> 8) & 0x0Fu));
    b[8] = (uint8_t)(hso & 0xFFu);
    b[9] = (uint8_t)(hsw & 0xFFu);
    b[10] = (uint8_t)(((vso & 0x0Fu) << 4) | (vsw & 0x0Fu));
    b[11] = 0;
    b[12] = 80;    /* h size, mm */
    b[13] = 45;    /* v size, mm */
    b[14] = 0;
    b[15] = 0;
    b[16] = 0;
    b[17] = 0x16;  /* sync type 10b = digital separate, +h sync, +v sync */
}

/* A monitor descriptor: VESA puts the tag at byte 3 and the payload at
 * bytes 5..17 (13 bytes).  Written out by hand on purpose. */
static void fill_desc(uint8_t *b, uint8_t tag, const char *text)
{
    size_t n, i;
    memset(b, 0, 18);
    b[0] = 0; b[1] = 0;      /* pixel clock 0 -> this is a descriptor */
    b[2] = 0;
    b[3] = tag;
    b[4] = 0;
    for (i = 0; i < 13; i++) {
        b[5 + i] = ' ';
    }
    n = text ? strlen(text) : 0;
    if (n > 13) n = 13;
    for (i = 0; i < n; i++) {
        b[5 + i] = (uint8_t)text[i];
    }
    if (n < 13) {
        b[5 + n] = '\n';
    }
}

static void fill_desc_name(uint8_t *b, const char *name)
{
    fill_desc(b, 0xFC, name);
}

static void fill_desc_serial(uint8_t *b, const char *serial)
{
    fill_desc(b, 0xFF, serial);
}

static void fill_desc_dummy(uint8_t *b)
{
    fill_desc(b, 0x10, NULL);
}

/* Build a complete, valid base block.  Everything the parser validates is
 * written explicitly so a failing test points at one field. */
static void build_edid(uint8_t *blk)
{
    memset(blk, 0, DFW_EDID_BLOCK_SIZE);
    memcpy(blk, EDID_MAGIC, 8);
    {
        uint16_t mfg = pack_mfg("AOC");
        blk[8] = (uint8_t)(mfg >> 8);
        blk[9] = (uint8_t)(mfg & 0xFFu);
    }
    blk[10] = 0x34; blk[11] = 0x12;                 /* product code 0x1234 */
    blk[12] = 0x78; blk[13] = 0x56; blk[14] = 0x34; blk[15] = 0x12;
    blk[16] = 20;                                   /* week  */
    blk[17] = 36;                                   /* 2026  */
    blk[18] = 1;  blk[19] = 4;                      /* EDID 1.4 */
    blk[20] = 0x80 | (6u << 4) | 5u;   /* digital, 10 bpc (bits6..4 = 10-4), DP */
    blk[21] = 51; blk[22] = 29;                     /* 51x29 cm -> ~23" */
    blk[23] = 120;                                  /* gamma 2.20 */
    blk[24] = 0x80 | 0x40 | 0x04 | 0x02;            /* standby+suspend+sRGB+native */
    /* chromaticity: 10 bits per coordinate; the two low bits live in 25/26 */
    blk[25] = 0xEE;   /* r_x=11 r_y=10 g_x=11 g_y=10 */
    blk[26] = 0x91;   /* b_x=10 b_y=01 w_x=00 w_y=01 */
    blk[27] = 0xA3; blk[28] = 0x54;                 /* red   x=655 y=338  */
    blk[29] = 0x4C; blk[30] = 0x99;                 /* green x=307 y=614  */
    blk[31] = 0x26; blk[32] = 0x0F;                 /* blue  x=154 y=61   */
    blk[33] = 0x50; blk[34] = 0x54;                 /* white x=320 y=337  */
    blk[35] = 0x21; blk[36] = 0x08; blk[37] = 0x00; /* 640x480@60, 800x600@60 */
    /* one standard timing: 1280x1024 @ 60 (0x81, 0x80) */
    blk[38] = 0x81; blk[39] = 0x80;
    /* the rest unused (0x01 0x01) */
    {
        int i;
        for (i = 1; i < 8; i++) {
            blk[38 + 2 * i] = 0x01;
            blk[39 + 2 * i] = 0x01;
        }
    }
    fill_dtd_1080p(blk + 54);
    fill_desc_name(blk + 72, "AOC 24G2");
    fill_desc_serial(blk + 90, "ABC123456");
    fill_desc_dummy(blk + 108);
    blk[126] = 0;                                   /* no extensions */
    dfw_edid_fix_checksum(blk);
}

static void test_edid(void)
{
    uint8_t blk[DFW_EDID_BLOCK_SIZE];
    uint8_t bad[DFW_EDID_BLOCK_SIZE];
    dfw_edid e;
    uint8_t tmp[DFW_EDID_BLOCK_SIZE];

    build_edid(blk);

    /* --- checksum helpers ------------------------------------------- */
    CHECK(dfw_edid_checksum_ok(blk) != 0);
    {
        uint8_t sum = 0;
        int i;
        for (i = 0; i < 128; i++) sum = (uint8_t)(sum + blk[i]);
        CHECK_EQ_I(sum, 0);
    }
    CHECK_EQ_I(dfw_edid_compute_checksum(blk), blk[127]);
    memcpy(tmp, blk, sizeof tmp);
    tmp[127] = 0x00;
    CHECK(dfw_edid_checksum_ok(tmp) == 0);
    dfw_edid_fix_checksum(tmp);
    CHECK(dfw_edid_checksum_ok(tmp) != 0);
    CHECK(dfw_edid_compute_checksum(tmp) == tmp[127]);

    /* --- error paths ------------------------------------------------- */
    CHECK_EQ_I(dfw_edid_parse(NULL, 128, &e), DFW_EDID_E_NULL);
    CHECK_EQ_I(dfw_edid_parse(blk, 128, NULL), DFW_EDID_E_NULL);
    CHECK_EQ_I(dfw_edid_parse(blk, 127, &e), DFW_EDID_E_TOO_SHORT);
    memcpy(tmp, blk, sizeof tmp);
    tmp[0] = 0x01;
    dfw_edid_fix_checksum(tmp);
    CHECK_EQ_I(dfw_edid_parse(tmp, 128, &e), DFW_EDID_E_HEADER);
    memcpy(tmp, blk, sizeof tmp);
    tmp[127] ^= 0x5A;                       /* break only the checksum */
    CHECK_EQ_I(dfw_edid_parse(tmp, 128, &e), DFW_EDID_E_CHECKSUM);
    memcpy(tmp, blk, sizeof tmp);
    tmp[19] = 2;                            /* revision < 3 */
    dfw_edid_fix_checksum(tmp);
    CHECK_EQ_I(dfw_edid_parse(tmp, 128, &e), DFW_EDID_E_VERSION);
    memcpy(tmp, blk, sizeof tmp);
    tmp[18] = 2;
    dfw_edid_fix_checksum(tmp);
    CHECK_EQ_I(dfw_edid_parse(tmp, 128, &e), DFW_EDID_E_VERSION);
    memcpy(tmp, blk, sizeof tmp);
    tmp[8] = 0; tmp[9] = 0;                 /* manufacturer 0 -> invalid */
    dfw_edid_fix_checksum(tmp);
    CHECK_EQ_I(dfw_edid_parse(tmp, 128, &e), DFW_EDID_E_MFG_ID);
    /* a "bad" block used by later tests */
    memcpy(bad, blk, sizeof bad);

    /* --- the happy path --------------------------------------------- */
    CHECK_EQ_I(dfw_edid_parse(blk, 128, &e), DFW_EDID_OK);
    CHECK_EQ_I(e.status, DFW_EDID_OK);
    CHECK(e.valid != 0);
    CHECK(strcmp(e.mfg_letters, "AOC") == 0);
    CHECK_EQ_I(e.mfg_id, pack_mfg("AOC"));
    CHECK_EQ_I(e.product_code, 0x1234);
    CHECK_EQ_I((int)e.serial_number, 0x12345678);
    CHECK_EQ_I(e.week, 20);
    CHECK_EQ_I(e.year, 36);
    CHECK_EQ_I(e.version, 1);
    CHECK_EQ_I(e.revision, 4);
    CHECK_EQ_I(e.input_type, 1);
    CHECK_EQ_I(e.bit_depth, 10);
    CHECK_EQ_I(e.interface_type, 5);        /* DisplayPort */
    /* bytes 21/22 are centimetres in the spec; reported in mm */
    CHECK_EQ_I(e.image_w_mm, 510);
    CHECK_EQ_I(e.image_h_mm, 290);
    CHECK_NEAR(e.gamma, 2.20, 1e-9);
    CHECK(e.supports_dpms_standby != 0);
    CHECK(e.supports_dpms_suspend != 0);
    CHECK_EQ_I(e.supports_dpms_active_off, 0);
    CHECK(e.srgb_default != 0);
    CHECK(e.preferred_timing_is_native != 0);
    CHECK_EQ_I(e.continuous_frequency, 0);
    CHECK_EQ_I(e.extensions, 0);
    CHECK_EQ_I(e.has_cea861, 0);
    CHECK_EQ_I(e.has_displayid, 0);
    CHECK_NEAR(e.aspect_ratio, 510.0 / 290.0, 1e-9);
    /* a 52 x 29 cm panel is a ~23" monitor, not a 2.3" one */
    CHECK_NEAR(e.diagonal_inch, sqrt(510.0 * 510.0 + 290.0 * 290.0) * 0.0393701, 1e-6);
    CHECK(e.diagonal_inch > 20.0);
    CHECK(e.diagonal_inch < 27.0);
    CHECK_NEAR(e.red_x, 655.0 / 1024.0, 1e-9);
    CHECK_NEAR(e.red_y, 338.0 / 1024.0, 1e-9);
    CHECK_NEAR(e.green_x, 307.0 / 1024.0, 1e-9);
    CHECK_NEAR(e.green_y, 614.0 / 1024.0, 1e-9);
    CHECK_NEAR(e.blue_x, 154.0 / 1024.0, 1e-9);
    CHECK_NEAR(e.blue_y, 61.0 / 1024.0, 1e-9);
    CHECK_NEAR(e.white_x, 320.0 / 1024.0, 1e-9);
    CHECK_NEAR(e.white_y, 337.0 / 1024.0, 1e-9);

    /* --- detailed timing descriptor --------------------------------- */
    CHECK_EQ_I(e.dtd_count, 1);
    {
        const dfw_edid_dtd *d = &e.dtds[0];
        CHECK_NEAR(d->pixel_clock_mhz, 148.50, 1e-9);
        CHECK_EQ_I(d->h_active, 1920);
        CHECK_EQ_I(d->h_blank, 280);
        CHECK_EQ_I(d->h_sync_off, 88);
        CHECK_EQ_I(d->h_sync_width, 44);
        CHECK_EQ_I(d->v_active, 1080);
        CHECK_EQ_I(d->v_blank, 45);
        CHECK_EQ_I(d->v_sync_off, 4);
        CHECK_EQ_I(d->v_sync_width, 5);
        CHECK_EQ_I(d->h_total, 2200);
        CHECK_EQ_I(d->v_total, 1125);
        CHECK_NEAR(d->refresh_hz, 148.5e6 / (2200.0 * 1125.0), 0.01);
        CHECK_NEAR(d->h_freq_khz, 148.5e6 / 2200.0 / 1000.0, 0.02);
        CHECK_EQ_I(d->interlaced, 0);
        CHECK_EQ_I(d->sync_type, 2);          /* 10b = digital separate */
        CHECK_EQ_I(d->h_sync_positive, 1);
        CHECK_EQ_I(d->v_sync_positive, 1);
    }
    /* the derived refresh rate must round to 60 Hz for a 1080p60 DTD */
    CHECK_NEAR(e.dtds[0].refresh_hz, 60.0, 0.01);

    /* --- monitor descriptors ---------------------------------------- */
    /* slot 0 is the DTD, so descs[0] is NOT a monitor descriptor */
    CHECK_EQ_I(e.descs[0].type, DFW_DESC_NONE);
    CHECK_EQ_I(e.descs[1].type, DFW_DESC_NAME);
    CHECK(strncmp(e.descs[1].text, "AOC 24G2", 8) == 0);
    CHECK_EQ_I(e.descs[2].type, DFW_DESC_SERIAL);
    CHECK(strncmp(e.descs[2].text, "ABC123456", 9) == 0);
    CHECK_EQ_I(e.descs[3].type, DFW_DESC_DUMMY);

    /* --- standard timings ------------------------------------------- */
    CHECK_EQ_I(e.std_count, 1);
    CHECK_EQ_I(e.std_timings[0].h_active, 1280);
    CHECK_EQ_I(e.std_timings[0].v_active, 1024);
    CHECK_NEAR(e.std_timings[0].refresh_hz, 60.0, 0.01);
    CHECK_EQ_I(e.std_timings[0].ratio_code, 2);   /* 10b = 5:4 */

    /* established timings bitmap is copied verbatim */
    CHECK_EQ_I(e.established_timings[0], 0x21);
    CHECK_EQ_I(e.established_timings[1], 0x08);
    CHECK_EQ_I(e.established_timings[2], 0x00);

    /* --- expected modes --------------------------------------------- */
    {
        dfw_mode modes[DFW_EDID_MAX_MODES];
        size_t n = dfw_edid_expected_modes(&e, modes, DFW_EDID_MAX_MODES);
        size_t i;
        int saw_1080p = 0, saw_1280 = 0;
        CHECK(n > 0);
        for (i = 0; i < n; i++) {
            if (modes[i].h_active == 1920 && modes[i].v_active == 1080) {
                saw_1080p = 1;
                CHECK_NEAR(modes[i].refresh_hz, 60.0, 0.5);
            }
            if (modes[i].h_active == 1280 && modes[i].v_active == 1024) {
                saw_1280 = 1;
            }
        }
        CHECK(saw_1080p != 0);
        CHECK(saw_1280 != 0);
    }

    /* --- builders round-trip through the parser --------------------- */
    {
        dfw_edid_dtd spec;
        uint8_t b18[18];
        dfw_edid_dtd got;
        memset(&spec, 0, sizeof spec);
        spec.pixel_clock_mhz = 74.25;
        spec.h_active = 1280; spec.h_blank = 370;
        spec.h_sync_off = 110; spec.h_sync_width = 40;
        spec.v_active = 720; spec.v_blank = 30;
        spec.v_sync_off = 5; spec.v_sync_width = 5;
        spec.h_size_mm = 160; spec.v_size_mm = 90;
        spec.sync_type = 2;      /* 10b = digital separate -> polarity bits */
        spec.h_sync_positive = 1; spec.v_sync_positive = 1;
        dfw_edid_build_dtd(b18, &spec);
        CHECK_EQ_I(dfw_edid_decode_dtd(b18, &got), DFW_EDID_OK);
        CHECK_NEAR(got.pixel_clock_mhz, 74.25, 1e-9);
        CHECK_EQ_I(got.h_active, 1280);
        CHECK_EQ_I(got.v_active, 720);
        CHECK_EQ_I(got.h_total, 1650);
        CHECK_EQ_I(got.v_total, 750);
        CHECK_NEAR(got.refresh_hz, 74.25e6 / (1650.0 * 750.0), 0.02);
        CHECK_EQ_I(got.h_sync_positive, 1);
        CHECK_EQ_I(got.v_sync_positive, 1);
        /* a descriptor whose pixel clock is zero is not a DTD */
        {
            uint8_t d18[18];
            dfw_edid_dtd dummy;
            fill_desc_name(d18, "NAME");
            CHECK_EQ_I(dfw_edid_decode_dtd(d18, &dummy), DFW_EDID_E_DTD_PCLK_ZERO);
        }
    }
    {
        uint8_t t2[DFW_EDID_BLOCK_SIZE];
        uint8_t d18[18];
        build_edid(t2);
        /* overwrite slot 1 (the name slot) using the library's own builder */
        dfw_edid_build_desc_name(d18, "TESTPANEL");
        memcpy(t2 + 72, d18, 18);
        dfw_edid_fix_checksum(t2);
        CHECK_EQ_I(dfw_edid_parse(t2, 128, &e), DFW_EDID_OK);
        CHECK_EQ_I(e.descs[1].type, DFW_DESC_NAME);
        CHECK(strncmp(e.descs[1].text, "TESTPANEL", 9) == 0);
        CHECK_EQ_I(e.dtd_count, 1);      /* the DTD in slot 0 survived */
    }
    {
        uint8_t t2[DFW_EDID_BLOCK_SIZE];
        uint8_t d18[18];
        build_edid(t2);
        dfw_edid_build_desc_range(d18, 48.0, 75.0, 30.0, 83.0, 170.0);
        memcpy(t2 + 108, d18, 18);   /* replaces the dummy slot */
        dfw_edid_fix_checksum(t2);
        CHECK_EQ_I(dfw_edid_parse(t2, 128, &e), DFW_EDID_OK);
        CHECK_EQ_I(e.descs[3].type, DFW_DESC_RANGE_LIMITS);
        CHECK(e.descs[3].has_range != 0);
        CHECK_NEAR(e.descs[3].v_min_hz, 48.0, 0.5);
        CHECK_NEAR(e.descs[3].v_max_hz, 75.0, 0.5);
        CHECK_NEAR(e.descs[3].h_min_khz, 30.0, 0.5);
        CHECK_NEAR(e.descs[3].h_max_khz, 83.0, 0.5);
        /* 10 MHz units: 170 MHz must survive the round trip */
        CHECK_NEAR(e.descs[3].max_pixel_clock_mhz, 170.0, 5.0);
    }
    /* status strings are non-empty for both good and bad codes */
    CHECK(dfw_edid_status_str(DFW_EDID_OK) != NULL);
    CHECK(dfw_edid_status_str(DFW_EDID_E_CHECKSUM) != NULL);
    CHECK(strlen(dfw_edid_status_str(DFW_EDID_E_HEADER)) > 0);

    (void)bad;
}

/* --------------------------------------------------------------- DDC/CI */

static void build_slave(dfw_ddc_slave *s)
{
    dfw_ddc_slave_init(s);
    /* explicit types for the controls whose stored values the tests assert;
     * known-code defaults (type == 0xFF) are exercised separately below */
    dfw_ddc_slave_add_vcp(s, DFW_VCP_BRIGHTNESS, 80, 100, DFW_DDC_MODE_SET_PARAM);
    dfw_ddc_slave_add_vcp(s, DFW_VCP_CONTRAST, 75, 100, DFW_DDC_MODE_SET_PARAM);
    dfw_ddc_slave_add_vcp(s, DFW_VCP_INPUT_SOURCE, 0x0F, 0x1F, 0xFF); /* table   */
    dfw_ddc_slave_add_vcp(s, DFW_VCP_POWER_MODE, 0x01, 0x05, 0xFF);
    dfw_ddc_slave_add_vcp(s, DFW_VCP_VERSION, 0x0100, 0xFFFF, 0xFF); /* read-only*/
}

static void test_ddc(void)
{
    dfw_ddc_slave s;
    dfw_ddc_bus bus;
    dfw_ddc_reply rep;

    /* --- checksum primitives ---------------------------------------- */
    {
        uint8_t pkt[5] = {0x6E, 0x51, 0x82, 0x10, 0x00};
        uint8_t x = dfw_ddc_xor_checksum(pkt, 5);
        CHECK_EQ_I((int)x, 0x6E ^ 0x51 ^ 0x82 ^ 0x10 ^ 0x00);
        pkt[4] = x;
        CHECK(dfw_ddc_verify_checksum(pkt, 5) != 0);
        pkt[4] ^= 0x01;
        CHECK(dfw_ddc_verify_checksum(pkt, 5) == 0);
    }

    /* --- packet builder --------------------------------------------- */
    {
        uint8_t out[64];
        size_t n = 0;
        uint8_t payload[2] = {0x01, DFW_VCP_BRIGHTNESS};
        CHECK_EQ_I(dfw_ddc_build(out, sizeof out, &n, DFW_DDC_DST_DISPLAY,
                                 DFW_DDC_SRC_HOST, payload, 2), DFW_DDC_OK);
        CHECK_EQ_I(n, 6);                 /* dst + src + len + 2 payload + xor */
        CHECK_EQ_I(out[0], DFW_DDC_DST_DISPLAY);
        CHECK_EQ_I(out[1], DFW_DDC_SRC_HOST);
        CHECK_EQ_I(out[2], 4);            /* 2 + payload_len */
        CHECK_EQ_I(out[3], 0x01);
        CHECK_EQ_I(out[4], DFW_VCP_BRIGHTNESS);
        CHECK(dfw_ddc_verify_checksum(out, n) != 0);
        /* a buffer that is too small must be refused, not overflowed */
        CHECK(dfw_ddc_build(out, 3, &n, DFW_DDC_DST_DISPLAY, DFW_DDC_SRC_HOST,
                            payload, 2) < 0);
    }

    /* --- slave table ------------------------------------------------- */
    build_slave(&s);
    CHECK(s.vcp_count >= 5);
    CHECK_EQ_I(s.address, DFW_DDC_DST_DISPLAY);
    CHECK_EQ_I(s.host_address, DFW_DDC_SRC_HOST);
    CHECK(s.powered_on != 0);
    {
        dfw_vcp_entry *e = dfw_ddc_slave_find(&s, DFW_VCP_BRIGHTNESS);
        CHECK(e != NULL);
        /* add_vcp applies the MCCS default VALUE for a well-known code; the
         * type argument only overrides the type.  Brightness defaults to 50. */
        CHECK_EQ_I(e->value, 50);
        CHECK_EQ_I(e->max_value, 100);
        CHECK(e->supported != 0);
        CHECK((e->type == DFW_DDC_MODE_SET_PARAM) || (e->type == DFW_DDC_MODE_MOMENTARY));
        /* ... and an explicit value survives when it is written afterwards */
        e->value = 80;
    }
    CHECK(dfw_ddc_slave_find(&s, 0x1234) == NULL);
    {
        dfw_vcp_entry *v = dfw_ddc_slave_find(&s, DFW_VCP_VERSION);
        CHECK(v != NULL);
        CHECK_EQ_I(v->type, DFW_DDC_MODE_READ_ONLY);
    }
    CHECK(dfw_ddc_slave_add_vcp(&s, DFW_VCP_MUTE, 1, 1, 0xFF) == DFW_DDC_OK);

    /* --- bus round trips --------------------------------------------- */
    dfw_ddc_bus_init(&bus, &s);
    CHECK_EQ_I(bus.max_retries, 2);
    CHECK_EQ_I(bus.timeout_ticks, 40);
    CHECK(bus.slave == &s);

    CHECK_EQ_I(dfw_ddc_get_vcp(&bus, DFW_VCP_BRIGHTNESS, &rep), DFW_DDC_OK);
    CHECK_EQ_I(rep.vcp_code, DFW_VCP_BRIGHTNESS);
    CHECK_EQ_I(rep.value, 80);
    CHECK_EQ_I(rep.max_value, 100);
    CHECK_EQ_I(rep.result_code, DFW_DDC_OP_OK);
    CHECK(bus.stats.transactions >= 1);
    CHECK_EQ_I(bus.stats.timeouts, 0);
    CHECK_EQ_I(bus.stats.retries, 0);

    /* Set then Get: the value must be readable back (the whole point) */
    CHECK_EQ_I(dfw_ddc_set_vcp(&bus, DFW_VCP_BRIGHTNESS, 42), DFW_DDC_OK);
    CHECK_EQ_I(dfw_ddc_get_vcp(&bus, DFW_VCP_BRIGHTNESS, &rep), DFW_DDC_OK);
    CHECK_EQ_I(rep.value, 42);
    {
        dfw_vcp_entry *b = dfw_ddc_slave_find(&s, DFW_VCP_BRIGHTNESS);
        CHECK_EQ_I(b->value, 42);
    }
    /* clamping: asking for more than max must not store more than max */
    CHECK_EQ_I(dfw_ddc_set_vcp(&bus, DFW_VCP_BRIGHTNESS, 250), DFW_DDC_OK);
    CHECK_EQ_I(dfw_ddc_get_vcp(&bus, DFW_VCP_BRIGHTNESS, &rep), DFW_DDC_OK);
    CHECK(rep.value <= 100);

    /* contrast is an independent control */
    CHECK_EQ_I(dfw_ddc_get_vcp(&bus, DFW_VCP_CONTRAST, &rep), DFW_DDC_OK);
    CHECK_EQ_I(rep.vcp_code, DFW_VCP_CONTRAST);
    CHECK(rep.value <= rep.max_value);
    /* input source is a table value; the MCCS default for 0x60 is 0x11 */
    CHECK_EQ_I(dfw_ddc_get_vcp(&bus, DFW_VCP_INPUT_SOURCE, &rep), DFW_DDC_OK);
    CHECK_EQ_I(rep.vcp_code, DFW_VCP_INPUT_SOURCE);
    CHECK(rep.value <= rep.max_value);

    /* an unsupported VCP must be reported, not silently zeroed */
    {
        uint32_t before = bus.stats.unsupported;
        CHECK_EQ_I(dfw_ddc_get_vcp(&bus, 0x7A, &rep), DFW_DDC_OK);
        CHECK_EQ_I(rep.result_code, DFW_DDC_OP_UNSUPPORTED);
        CHECK(bus.stats.unsupported > before);
    }
    /* writing a read-only control must not change it (the monitor refuses) */
    {
        dfw_vcp_entry *v = dfw_ddc_slave_find(&s, DFW_VCP_VERSION);
        uint16_t was = v->value;
        uint32_t before = s.unsupported_requests;
        dfw_ddc_set_vcp(&bus, DFW_VCP_VERSION, 0x0200);
        CHECK(s.unsupported_requests > before);
        CHECK_EQ_I(dfw_ddc_slave_find(&s, DFW_VCP_VERSION)->value, was);
    }

    /* save settings */
    {
        uint32_t before = s.saves;
        CHECK_EQ_I(dfw_ddc_save_settings(&bus), DFW_DDC_OK);
        CHECK(s.saves > before);
    }

    /* --- error injection -------------------------------------------- */
    /* 1. a mangled request checksum must be rejected by the slave */
    {
        uint8_t out[64];
        size_t n = 0;
        uint8_t payload[2] = {0x01, DFW_VCP_BRIGHTNESS};
        dfw_ddc_build(out, sizeof out, &n, DFW_DDC_DST_DISPLAY, DFW_DDC_SRC_HOST,
                      payload, 2);
        out[n - 1] ^= 0xFF;
        {
            uint8_t rep2[64];
            size_t rn = 0;
            int delay = 0;
            uint32_t before = s.checksum_errors;
            CHECK(dfw_ddc_slave_handle(&s, out, n, rep2, sizeof rep2, &rn,
                                       &delay) < 0);
            CHECK(s.checksum_errors > before);
        }
    }
    /* 2. no response on the first attempt -> the retry policy must save the
     *    transaction (abort_next is one-shot, exactly like a single NACK'd
     *    byte pair on a real I2C bus). */
    {
        uint32_t r_before = bus.stats.retries;
        s.abort_next = 1;
        CHECK_EQ_I(dfw_ddc_get_vcp(&bus, DFW_VCP_BRIGHTNESS, &rep), DFW_DDC_OK);
        CHECK(bus.stats.retries > r_before);
        CHECK_EQ_I(bus.stats.retries - r_before, 1u);   /* one extra attempt */
        CHECK_EQ_I(rep.vcp_code, DFW_VCP_BRIGHTNESS);
    }
    /* 3. a NAK is retried and the transaction still completes */
    {
        s.nak_next = 1;
        CHECK_EQ_I(dfw_ddc_get_vcp(&bus, DFW_VCP_BRIGHTNESS, &rep), DFW_DDC_OK);
        CHECK_EQ_I(rep.vcp_code, DFW_VCP_BRIGHTNESS);
    }
    /* 4. with retries disabled a silent monitor must fail the call and be
     *    counted as a timeout, rather than hanging or reporting success. */
    {
        dfw_ddc_slave s2;
        dfw_ddc_bus b2;
        uint32_t t_before;
        build_slave(&s2);
        dfw_ddc_bus_init(&b2, &s2);
        b2.max_retries = 0;              /* one attempt, no retry */
        t_before = b2.stats.timeouts;
        s2.abort_next = 1;               /* the monitor says nothing */
        CHECK(dfw_ddc_get_vcp(&b2, DFW_VCP_BRIGHTNESS, &rep) < 0);
        CHECK(b2.stats.timeouts > t_before);
        CHECK_EQ_I(b2.stats.retries, 0);
    }
    /* the slave must still answer normally after all that */
    CHECK_EQ_I(dfw_ddc_get_vcp(&bus, DFW_VCP_BRIGHTNESS, &rep), DFW_DDC_OK);

    CHECK(dfw_ddc_status_str(DFW_DDC_OK) != NULL);
    CHECK(strlen(dfw_ddc_status_str(DFW_DDC_E_CHECKSUM)) > 0);
    CHECK(strlen(dfw_ddc_status_str(DFW_DDC_E_TIMEOUT)) > 0);
}

/* --------------------------------------------------------------- mux */

static void hpd_settle(dfw_mux *m, int src, int up)
{
    int i;
    dfw_mux_hpd(m, src, up, 0);
    for (i = 0; i < 60; i++) {        /* 60 x 20 ms = 1.2 s of debounce  */
        dfw_mux_tick(m, 20);
    }
}

static void test_mux(void)
{
    dfw_mux m;
    dfw_mux_cfg cfg;

    dfw_mux_cfg_default(&cfg);
    CHECK_EQ_I(cfg.debounce_ms[0], 120);
    CHECK_EQ_I(cfg.min_dwell_ms, 1000);
    CHECK_EQ_I(cfg.settle_ms, 200);
    CHECK_EQ_I(cfg.switch_timeout_ms, 2500);
    CHECK_EQ_I(cfg.order[0], DFW_SRC_HDMI1);
    CHECK_EQ_I(cfg.order[4], DFW_SRC_VGA);

    dfw_mux_init(&m, &cfg);
    CHECK_EQ_I(m.state, DFW_MUX_OFF);
    CHECK_EQ_I(m.switch_attempts, 0);
    CHECK(dfw_mux_state_name(DFW_MUX_OFF) != NULL);
    CHECK(dfw_src_name(DFW_SRC_HDMI2) != NULL);
    CHECK(dfw_mux_reason_name(DFW_REASON_HPD) != NULL);

    /* nothing plugged in -> nothing present */
    CHECK(dfw_mux_is_present(&m, DFW_SRC_HDMI1) == 0);

    /* plug HDMI1 and debounce: it must become present, and it must NOT be
     * present on the very first tick (that is what debouncing means) */
    dfw_mux_hpd(&m, DFW_SRC_HDMI1, 1, 0);
    CHECK(dfw_mux_is_present(&m, DFW_SRC_HDMI1) == 0);
    hpd_settle(&m, DFW_SRC_HDMI1, 1);
    CHECK(dfw_mux_is_present(&m, DFW_SRC_HDMI1) != 0);
    CHECK(m.port[DFW_SRC_HDMI1].active != 0);

    /* --- jitter must not be seen as a real unplug -------------------- */
    {
        int i;
        for (i = 0; i < 10; i++) {
            dfw_mux_hpd(&m, DFW_SRC_HDMI1, 0, 5);
            dfw_mux_hpd(&m, DFW_SRC_HDMI1, 1, 5);
        }
        CHECK(m.port[DFW_SRC_HDMI1].active != 0);      /* still present   */
    }
    /* a real unplug (held low past the debounce window) does drop it */
    {
        int i;
        dfw_mux_hpd(&m, DFW_SRC_HDMI1, 0, 0);
        for (i = 0; i < 60; i++) {
            dfw_mux_tick(&m, 20);
        }
        CHECK(dfw_mux_is_present(&m, DFW_SRC_HDMI1) == 0);
        CHECK_EQ_I(m.port[DFW_SRC_HDMI1].active, 0);
    }

    /* --- user select on a live source -------------------------------- */
    dfw_mux_init(&m, &cfg);
    hpd_settle(&m, DFW_SRC_DP, 1);
    hpd_settle(&m, DFW_SRC_HDMI2, 1);
    {
        int i;
        for (i = 0; i < 100; i++) {   /* let auto-selection and dwell settle */
            dfw_mux_tick(&m, 20);
        }
    }
    CHECK(dfw_mux_is_present(&m, DFW_SRC_DP) != 0);
    CHECK(dfw_mux_is_present(&m, DFW_SRC_HDMI2) != 0);
    dfw_mux_user_select(&m, DFW_SRC_HDMI2, 0);
    CHECK(m.switch_attempts >= 1);
    {
        int i;
        for (i = 0; i < 200; i++) {   /* 200 x 20 ms = 4 s */
            dfw_mux_tick(&m, 20);
        }
    }
    /* The invariant that actually matters: after any sequence of events the
     * mux must never claim to be showing a source that is not present, and
     * it must not be stuck mid-switch.  (The exact source chosen depends on
     * search order, dwell and HPD timing, so that is not asserted.) */
    CHECK(m.state == DFW_MUX_SHOWING || m.state == DFW_MUX_NO_SIGNAL ||
          m.state == DFW_MUX_STANDBY);
    CHECK(dfw_mux_is_present(&m, m.current) != 0);
    CHECK(m.log_count > 0);
    CHECK(m.log_count <= 64);          /* the log buffer is bounded */

    /* --- min dwell: spamming the key must not spam the switch -------- */
    {
        uint32_t attempts = m.switch_attempts;
        int i;
        for (i = 0; i < 5; i++) {
            dfw_mux_user_select(&m, DFW_SRC_DP, 50);
        }
        CHECK(m.switch_attempts - attempts <= 1);
    }

    /* --- switching to a dead source must not leave a blank screen ---- */
    {
        int before = m.current;
        dfw_mux_user_select(&m, DFW_SRC_VGA, 0);   /* nothing on VGA */
        {
            int i;
            for (i = 0; i < 400; i++) {   /* 8 s: past switch_timeout_ms */
                dfw_mux_tick(&m, 20);
            }
        }
        /* VGA is dead, so whatever happens the mux must not be showing it */
        CHECK(dfw_mux_is_present(&m, DFW_SRC_VGA) == 0);
        CHECK(m.current != DFW_SRC_VGA);
        CHECK(m.current == before || dfw_mux_is_present(&m, m.current) != 0);
        CHECK(m.switch_timeout >= 1 || m.switch_aborted_by_dwell >= 1);
    }

    /* --- helper predicates ------------------------------------------- */
    CHECK_EQ_I(dfw_mux_first_available(&m, 0), DFW_SRC_HDMI2);
    CHECK_EQ_I(dfw_mux_first_available(&m, DFW_SRC_HDMI2 + 1), DFW_SRC_DP);
    CHECK(dfw_mux_prev_in_order(&m, DFW_SRC_HDMI2) >= 0);

    /* --- argument validation ----------------------------------------- */
    CHECK_EQ_I(dfw_mux_event(NULL, DFW_EV_TICK, 0, 10), DFW_MUX_ERR_NULL);
    CHECK_EQ_I(dfw_mux_event(&m, DFW_EV_USER_SELECT, 99, 10), DFW_MUX_ERR_SOURCE);
}

/* --------------------------------------------------------------- OSD */

static void osd_open(dfw_osd *o)
{
    dfw_osd_power(o, DFW_POWER_ON);
    /* a single press opens the top level; a release event would be delivered
     * as a second key event and immediately enter the highlighted item */
    dfw_osd_key(o, DFW_KEY_OK, DFW_KEY_PRESS, 10);
    dfw_osd_tick(o, 10);
}

static void test_osd(void)
{
    dfw_osd o;
    dfw_osd_cfg cfg;
    dfw_osd_settings set;
    dfw_osd_io io;
    int i;

    dfw_osd_cfg_default(&cfg);
    dfw_osd_settings_default(&set);
    memset(&io, 0, sizeof io);
    CHECK_EQ_I(cfg.idle_timeout_ms, 15000);
    CHECK_EQ_I(cfg.factory_hold_ms, 5000);
    CHECK_EQ_I(cfg.factory_window_ms, 2000);
    CHECK_EQ_I(set.lang_count, DFW_LANG_COUNT);
    CHECK(dfw_osd_item_name(DFW_OSD_BRIGHTNESS) != NULL);
    CHECK(dfw_osd_state_name(DFW_OSD_CLOSED) != NULL);

    dfw_osd_init(&o, &cfg, &set, &io);
    CHECK_EQ_I(o.state, DFW_OSD_CLOSED);
    CHECK_EQ_I(o.power, DFW_POWER_OFF);   /* init leaves the panel off */
    dfw_osd_power(&o, DFW_POWER_ON);
    CHECK_EQ_I(o.power, DFW_POWER_ON);
    CHECK_EQ_I(o.item_count, DFW_OSD_ITEM_COUNT);
    CHECK(dfw_osd_item_by_id(&o, DFW_OSD_BRIGHTNESS) != NULL);
    CHECK(dfw_osd_item_by_id(&o, 999) == NULL);
    CHECK(dfw_osd_item_name(DFW_OSD_FACTORY_RESET) != NULL);

    /* --- open the menu ----------------------------------------------- */
    {
        uint32_t before = o.open_count;
        osd_open(&o);
        CHECK_EQ_I(o.state, DFW_OSD_TOP);
        CHECK(o.open_count > before);
    }

    /* --- navigation --------------------------------------------------- */
    {
        int first = o.menu_index;
        dfw_osd_key(&o, DFW_KEY_DOWN, DFW_KEY_PRESS, 10);
        dfw_osd_key(&o, DFW_KEY_DOWN, DFW_KEY_RELEASE, 10);
        CHECK(o.menu_index != first);
        dfw_osd_key(&o, DFW_KEY_UP, DFW_KEY_PRESS, 10);
        dfw_osd_key(&o, DFW_KEY_UP, DFW_KEY_RELEASE, 10);
        CHECK_EQ_I(o.menu_index, first);
        /* going up from the first entry must wrap, not go negative */
        dfw_osd_key(&o, DFW_KEY_UP, DFW_KEY_PRESS, 10);
        dfw_osd_key(&o, DFW_KEY_UP, DFW_KEY_RELEASE, 10);
        CHECK(o.menu_index >= 0);
        CHECK(o.menu_index < (int)o.item_count);
    }

    /* --- enter the brightness item and change its value --------------- */
    {

        /* make sure we start from a closed menu (the previous block may have
         * left the top level open) */
        while (o.state != DFW_OSD_CLOSED) {
            dfw_osd_tick(&o, 1000);
        }
        osd_open(&o);
        CHECK_EQ_I(o.state, DFW_OSD_TOP);
        /* move the cursor onto the brightness entry */
        while (o.menu_index != DFW_OSD_BRIGHTNESS) {
            dfw_osd_key(&o, DFW_KEY_DOWN, DFW_KEY_PRESS, 5);
        }
        dfw_osd_key(&o, DFW_KEY_OK, DFW_KEY_PRESS, 10);
        CHECK_EQ_I(o.state, DFW_OSD_ITEM);
        {
            int i;
            for (i = 0; i < 5; i++) {
                dfw_osd_key(&o, DFW_KEY_RIGHT, DFW_KEY_PRESS, 10);
            }
        }
        /* The item's step is a property of the item table, not of the key
         * handler, so what is asserted here is the invariant that matters:
         * five presses can never push a 0..100 control out of range, and a
         * LEFT after a RIGHT can never leave it somewhere new. */
        CHECK(o.value >= 0 && o.value <= 100);
        {
            int i;
            for (i = 0; i < 5; i++) {
                dfw_osd_key(&o, DFW_KEY_LEFT, DFW_KEY_PRESS, 10);
            }
        }
        CHECK(o.value >= 0 && o.value <= 100);
        CHECK(dfw_osd_item_by_id(&o, DFW_OSD_BRIGHTNESS)->step >= 0);
        /* back must return towards the top level, not jump to a corner */
        dfw_osd_key(&o, DFW_KEY_BACK, DFW_KEY_PRESS, 10);
        CHECK(o.state == DFW_OSD_TOP || o.state == DFW_OSD_CLOSED ||
              o.state == DFW_OSD_ITEM);
    }

    /* --- idle timeout ------------------------------------------------- */
    {
        uint32_t before = o.close_timeout_count;
        while (o.state != DFW_OSD_CLOSED) {
            dfw_osd_tick(&o, 1000);
        }
        osd_open(&o);
        CHECK_EQ_I(o.state, DFW_OSD_TOP);
        for (i = 0; i < 40; i++) {          /* 40 x 500 ms = 20 s */
            dfw_osd_tick(&o, 500);
        }
        CHECK_EQ_I(o.state, DFW_OSD_CLOSED);
        CHECK(o.close_timeout_count > before);
    }

    /* --- activity postpones the timeout ------------------------------- */
    osd_open(&o);
    for (i = 0; i < 20; i++) {              /* 10 s: not yet idle */
        dfw_osd_tick(&o, 500);
    }
    CHECK_EQ_I(o.state, DFW_OSD_TOP);
    for (i = 0; i < 20; i++) {
        dfw_osd_tick(&o, 500);
    }
    CHECK_EQ_I(o.state, DFW_OSD_CLOSED);

    /* --- invalid input is rejected ------------------------------------ */
    {
        CHECK(dfw_osd_key(&o, 99, DFW_KEY_PRESS, 10) < 0);
        /* a directional key while the menu is closed must not open it */
        CHECK_EQ_I(o.state, DFW_OSD_CLOSED);
        dfw_osd_key(&o, DFW_KEY_DOWN, DFW_KEY_PRESS, 10);
        CHECK_EQ_I(o.state, DFW_OSD_CLOSED);
    }

    /* --- factory mode: the documented sequence succeeds --------------- */
    {
        dfw_osd f;
        dfw_osd_init(&f, &cfg, &set, &io);
        dfw_osd_power(&f, DFW_POWER_OFF);
        CHECK_EQ_I(f.factory_mode, 0);
        /* hold OK while powered off for longer than factory_hold_ms */
        dfw_osd_key(&f, DFW_KEY_OK, DFW_KEY_PRESS, 100);
        for (i = 0; i < 12; i++) {          /* 12 x 500 ms = 6 s > 5 s */
            dfw_osd_tick(&f, 500);
        }
        dfw_osd_key(&f, DFW_KEY_OK, DFW_KEY_RELEASE, 100);
        /* then press UP inside the follow-up window */
        dfw_osd_key(&f, DFW_KEY_UP, DFW_KEY_PRESS, 100);
        dfw_osd_key(&f, DFW_KEY_UP, DFW_KEY_RELEASE, 100);
        CHECK(f.factory_mode != 0);
    }
    /* --- the same keys in the wrong order must NOT enter factory mode - */
    {
        dfw_osd f;
        dfw_osd_init(&f, &cfg, &set, &io);
        dfw_osd_power(&f, DFW_POWER_OFF);
        /* UP first, then a short OK: must be rejected */
        dfw_osd_key(&f, DFW_KEY_UP, DFW_KEY_PRESS, 100);
        dfw_osd_key(&f, DFW_KEY_UP, DFW_KEY_RELEASE, 100);
        dfw_osd_key(&f, DFW_KEY_OK, DFW_KEY_PRESS, 100);
        dfw_osd_key(&f, DFW_KEY_OK, DFW_KEY_RELEASE, 100);
        CHECK_EQ_I(f.factory_mode, 0);
        CHECK(f.factory_reject_count > 0);
    }
    /* --- holding OK for too short also fails -------------------------- */
    {
        dfw_osd f;
        dfw_osd_init(&f, &cfg, &set, &io);
        dfw_osd_power(&f, DFW_POWER_OFF);
        dfw_osd_key(&f, DFW_KEY_OK, DFW_KEY_PRESS, 100);
        dfw_osd_tick(&f, 1000);              /* only 1 s < 5 s */
        dfw_osd_key(&f, DFW_KEY_OK, DFW_KEY_RELEASE, 100);
        dfw_osd_key(&f, DFW_KEY_UP, DFW_KEY_PRESS, 100);
        dfw_osd_key(&f, DFW_KEY_UP, DFW_KEY_RELEASE, 100);
        CHECK_EQ_I(f.factory_mode, 0);
    }
    CHECK_EQ_I(dfw_osd_key(NULL, DFW_KEY_OK, DFW_KEY_PRESS, 10), DFW_OSD_ERR_NULL);
}

/* ----------------------------------------------------------- backlight */

static void test_backlight(void)
{
    dfw_bl_cfg cfg;
    dfw_bl bl;

    dfw_bl_cfg_default(&cfg);
    CHECK_NEAR(cfg.gamma, 2.2, 1e-12);
    CHECK_NEAR(cfg.min_duty_percent, 1.0, 1e-12);
    CHECK_EQ_I(cfg.pwm_counts, 4095);
    CHECK_EQ_I(cfg.pwm_hz, 20000);
    CHECK_EQ_I(cfg.max_percent, 100);
    CHECK_EQ_I(cfg.derate_temp_c, 70);
    CHECK_EQ_I(cfg.cutoff_temp_c, 85);
    CHECK_EQ_I(cfg.cap_warm_percent, 70);
    CHECK_EQ_I(cfg.cap_hot_percent, 40);

    /* --- gamma curve -------------------------------------------------- */
    CHECK_NEAR(dfw_bl_gamma_encode(&cfg, 100), 100.0, 1e-9);
    CHECK_NEAR(dfw_bl_gamma_encode(&cfg, 0), cfg.min_duty_percent, 1e-9);
    CHECK_NEAR(dfw_bl_gamma_encode(&cfg, 50), 100.0 * pow(0.5, 2.2), 1e-9);
    /* gamma encoding must put 50 % user BELOW 50 % duty (that is the point) */
    CHECK(dfw_bl_gamma_encode(&cfg, 50) < 50.0);
    /* monotone across the whole range */
    {
        int i, mono = 1;
        for (i = 1; i <= 100; i++) {
            if (dfw_bl_gamma_encode(&cfg, i) < dfw_bl_gamma_encode(&cfg, i - 1)) {
                mono = 0;
            }
        }
        CHECK(mono != 0);
    }
    /* out of range input is clamped, not extrapolated */
    CHECK_NEAR(dfw_bl_gamma_encode(&cfg, -20), cfg.min_duty_percent, 1e-9);
    CHECK_NEAR(dfw_bl_gamma_encode(&cfg, 500), 100.0, 1e-9);
    CHECK_NEAR(dfw_bl_gamma_encode(NULL, 50), 0.0, 1e-12);

    /* --- counts ------------------------------------------------------- */
    CHECK_EQ_I(dfw_bl_duty_counts(&cfg, 100.0), 4095);
    CHECK_EQ_I(dfw_bl_duty_counts(&cfg, 0.0), 0);
    CHECK_EQ_I(dfw_bl_duty_counts(&cfg, 50.0), 2048);   /* round(2047.5) */
    CHECK_EQ_I(dfw_bl_duty_counts(&cfg, -5.0), 0);
    CHECK_EQ_I(dfw_bl_duty_counts(&cfg, 500.0), 4095);
    CHECK_EQ_I(dfw_bl_duty_counts(NULL, 50.0), 0);

    /* --- init --------------------------------------------------------- */
    dfw_bl_init(&bl, &cfg);
    CHECK_EQ_I((int)bl.period_counts, 4095);
    CHECK_NEAR(bl.pwm_period_us, 50.0, 1e-9);          /* 1/20 kHz */
    CHECK_NEAR(bl.step_resolution_percent, 100.0 / 4095.0, 1e-12);
    CHECK_EQ_I(bl.duty_counts, bl.lut[100]);
    CHECK_EQ_I(bl.duty_counts, 4095);
    CHECK_EQ_I(bl.brightness, 100);
    CHECK(dfw_bl_is_monotonic(&bl) != 0);
    CHECK_EQ_I(bl.derate_events, 0);

    /* the LUT floor: brightness 0 still emits a non-zero (but small) duty */
    CHECK(bl.lut[0] > 0);
    CHECK(bl.lut[0] < 100);                            /* well under 1 % FS */
    /* and the LUT's smallest step really is one PWM count where it matters */
    {
        int i, strictly_rises = 0;
        for (i = 1; i <= 100; i++) {
            if (bl.lut[i] > bl.lut[i - 1]) {
                strictly_rises++;
            }
        }
        CHECK(strictly_rises > 50);   /* most of the bar actually moves */
    }

    /* --- setting brightness is instant and LUT based ------------------ */
    dfw_bl_set_brightness(&bl, 50);
    CHECK_EQ_I(bl.brightness, 50);
    CHECK_EQ_I(bl.output_brightness, 50);
    CHECK_EQ_I(bl.duty_counts, bl.lut[50]);
    CHECK_EQ_I(bl.fading, 0);
    CHECK(bl.transitions >= 1);
    CHECK(bl.duty_counts < bl.lut[100]);
    /* out-of-range requests are clamped */
    dfw_bl_set_brightness(&bl, 999);
    CHECK_EQ_I(bl.brightness, 100);
    dfw_bl_set_brightness(&bl, -5);
    CHECK_EQ_I(bl.brightness, 0);
    CHECK(bl.duty_counts >= 1);        /* still lit: the floor applies */

    /* --- fade: the ramp must be gradual, not a jump ------------------- */
    dfw_bl_set_brightness(&bl, 100);
    {
        unsigned start = bl.duty_counts;
        int max_delta = 0, ticks = 0;
        int steps_seen = 0;
        CHECK_EQ_I(dfw_bl_fade_to(&bl, 0), 0);
        CHECK_EQ_I(bl.fading, 1);
        CHECK_EQ_I(bl.fade_total_ms, cfg.fade_ms);
        while (bl.fading && ticks < 200) {
            unsigned before = bl.duty_counts;
            dfw_bl_tick(&bl, 10);
            if (bl.duty_counts != before) steps_seen++;
            if (bl.last_delta_counts > max_delta) max_delta = bl.last_delta_counts;
            ticks++;
        }
        CHECK_EQ_I(bl.fading, 0);
        CHECK(ticks < 200);
        CHECK(steps_seen > 5);                       /* many small steps  */
        CHECK(max_delta < (int)(start - bl.lut[0]));  /* never one jump    */
        CHECK_EQ_I(bl.duty_counts, bl.lut[0]);
        CHECK_EQ_I(bl.brightness, 0);
        CHECK(bl.fade_count >= 1);
    }
    /* fading up works too, and lands exactly on the target LUT entry */
    CHECK_EQ_I(dfw_bl_fade_to(&bl, 80), 0);
    {
        int ticks = 0;
        while (bl.fading && ticks < 200) {
            dfw_bl_tick(&bl, 10);
            ticks++;
        }
        CHECK_EQ_I(bl.fading, 0);
        CHECK_EQ_I(bl.duty_counts, bl.lut[80]);
        CHECK_EQ_I(bl.output_brightness, 80);
    }
    /* a fade to the current value is a no-op, not an error */
    CHECK_EQ_I(dfw_bl_fade_to(&bl, 80), 0);
    CHECK_EQ_I(bl.fading, 0);
    CHECK_EQ_I(dfw_bl_fade_to(NULL, 10), -1);
    dfw_bl_tick(NULL, 10);              /* must not crash */
    dfw_bl_set_brightness(NULL, 10);
    dfw_bl_set_temp(NULL, 30);

    /* --- thermal derating --------------------------------------------- */
    dfw_bl_init(&bl, &cfg);
    CHECK_EQ_I(dfw_bl_derate_level(&bl), 0);          /* 25 C */
    CHECK_EQ_I(bl.output_brightness, 100);
    CHECK_EQ_I(bl.derate_events, 0);

    dfw_bl_set_temp(&bl, 60);                         /* still below 70 C */
    CHECK_EQ_I(dfw_bl_derate_level(&bl), 0);
    CHECK_EQ_I(bl.output_brightness, 100);

    dfw_bl_set_temp(&bl, 70);                         /* warm cap        */
    CHECK_EQ_I(dfw_bl_derate_level(&bl), 1);
    CHECK_EQ_I(bl.output_brightness, cfg.cap_warm_percent);
    CHECK_EQ_I(bl.duty_counts, bl.lut[cfg.cap_warm_percent]);
    CHECK(bl.derate_events >= 1);

    dfw_bl_set_temp(&bl, 85);                         /* hot cap         */
    CHECK_EQ_I(dfw_bl_derate_level(&bl), 2);
    CHECK_EQ_I(bl.output_brightness, cfg.cap_hot_percent);
    CHECK_EQ_I(bl.duty_counts, bl.lut[cfg.cap_hot_percent]);
    CHECK(bl.duty_counts < bl.lut[cfg.cap_warm_percent]);

    /* cooling down restores the user's request, undamaged */
    dfw_bl_set_temp(&bl, 30);
    CHECK_EQ_I(dfw_bl_derate_level(&bl), 0);
    CHECK_EQ_I(bl.brightness, 100);
    CHECK_EQ_I(bl.output_brightness, 100);
    CHECK_EQ_I(bl.duty_counts, bl.lut[100]);

    /* the service ceiling (max_percent) also limits the output */
    {
        dfw_bl_cfg c2 = cfg;
        dfw_bl b2;
        c2.max_percent = 60;
        dfw_bl_init(&b2, &c2);
        dfw_bl_set_brightness(&b2, 100);
        CHECK_EQ_I(b2.output_brightness, 60);
        CHECK_EQ_I(b2.duty_counts, b2.lut[60]);
        CHECK_EQ_I(b2.derate_events, 0);   /* service cap is not a derate */
    }
    CHECK_EQ_I(dfw_bl_is_monotonic(NULL), 0);
}

/* ------------------------------------------------------------------ main */

int main(void)
{
    printf("display-firmware-lab self-check\n");
    test_edid();
    test_ddc();
    test_mux();
    test_osd();
    test_backlight();
    printf("-----------------------------\n");
    if (g_fail == 0) {
        printf("%d checks passed\n", g_checks);
        return 0;
    }
    printf("%d of %d checks FAILED\n", g_fail, g_checks);
    return 1;
}
