/*
 * edid.c -- EDID 1.4 base-block parsing, validation and timing derivation.
 *
 * ---------------------------------------------------------------------
 * WIRE FORMAT (VESA E-EDID 1.4, base block = 128 bytes)
 * ---------------------------------------------------------------------
 * offset  size  meaning
 *   0..7    8   header, must be 00 FF FF FF FF FF FF 00
 *   8..9    2   manufacturer ID: three 5-bit letters, big-endian packed,
 *               letter = value + 'A' - 1  (so 0x10 -> 'A', 0x00 is invalid)
 *  10..11   2   product code, little-endian
 *  12..15   4   serial number, little-endian (0 = not used)
 *  16       1   week of manufacture (0 = unused, 0xFF = model year)
 *  17       1   year of manufacture - 1990
 *  18       1   EDID version (1)
 *  19       1   EDID revision (3 or 4)
 *  20       1   video input definition
 *               bit7: 1 = digital
 *               digital: bits6..4 colour bit depth - 4, bits3..0 interface
 *                        (0 undefined, 1 DVI, 2 HDMI-a, 3 HDMI-b, 4 MDDI, 5 DP)
 *               analogue: bits6..5 level, bit4 blank-to-black, bit3 separate
 *                        sync, bit2 composite, bit1 sync-on-green, bit0 serr.
 *  21       1   horizontal screen size in mm (0 = undefined)
 *  22       1   vertical   screen size in mm (0 = undefined)
 *  23       1   gamma: (gamma * 100) - 100  -> 120 means 2.20; 0xFF = in ext
 *  24       1   features: bit7 standby, bit6 suspend, bit5 active-off,
 *               bits4..3 colour type (0 RGB444, 1 RGB444+YCrCb,
 *               2 RGB444+YCrCb422, 3 RGB444+YCrCb444), bit2 sRGB,
 *               bit1 preferred timing is native, bit0 continuous frequency
 *  25..34  10   chromaticity: 2 low bits each for red/green (byte 25) and
 *               green/blue/white (byte 26), 8 high bits in bytes 27..34
 *  35..37   3   established timings bitmap
 *  38..53  16   eight 2-byte standard timing entries
 *  54..125 72   four 18-byte descriptors
 * 126       1   extension block count
 * 127       1   checksum: sum(bytes 0..127) % 256 == 0
 *
 * 18-byte descriptor (a DTD when bytes 0..1 != 0):
 *   0..1   pixel clock in 10 kHz units, little-endian
 *   2      horizontal active, low 8 bits
 *   3      horizontal blanking, low 8 bits
 *   4      (h_active >> 8) << 4 | (h_blank >> 8)
 *   5      vertical active, low 8 bits
 *   6      vertical blanking, low 8 bits
 *   7      (v_active >> 8) << 4 | (v_blank >> 8)
 *   8      horizontal front porch (sync offset), low 8 bits
 *   9      horizontal sync pulse width, low 8 bits
 *  10      (v_offset << 4) | v_sync_width           [4 bits each]
 *  11      (h_offset >> 8) << 6 | (h_sync_width >> 8) << 4
 *          | (v_offset >> 4) << 2 | (v_sync_width >> 4)
 *  12      horizontal image size, mm, low 8 bits
 *  13      vertical image size, mm, low 8 bits
 *  14      (h_size >> 8) << 4 | (v_size >> 8)
 *  15      horizontal border
 *  16      vertical border
 *  17      flags: bit7 interlaced, bits6..5 stereo, bits4..3 sync type,
 *          bit2 serration, bit1 sync-on-green / V polarity,
 *          bit0 composite sync / H polarity.
 *          For sync type 3 (digital separate) the spec means:
 *            bit1 = vertical sync polarity, bit0 = horizontal sync polarity.
 *
 * ---------------------------------------------------------------------
 * TIMING MATHS
 * ---------------------------------------------------------------------
 *   h_total = h_active + h_blank        v_total = v_active + v_blank
 *   refresh = pixel_clock / (h_total * v_total)
 *   h_freq  = pixel_clock / h_total                     (line rate)
 * The totals include the whole blanking interval (front porch + sync + back
 * porch).  EDID only stores the sum, which is exactly what the formula
 * needs, so the blanking interval never has to be decomposed.
 *
 * All arithmetic is done in double.  `-ffp-contract=off` is mandatory:
 * clang on macOS would otherwise fuse `a*b+c` into an FMA, the last digit
 * of refresh_hz would differ between platforms, and the equality
 * assertions in test_dfw.c would pass on Linux and fail on macOS.
 */
#include "dfw.h"

#include <string.h>

static const uint8_t edid_magic[8] = {
    0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00
};

int dfw_clamp_i(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

uint32_t dfw_clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

double dfw_lerp(double a, double b, double t)
{
    return a + (b - a) * t;
}

const char *dfw_edid_status_str(int status)
{
    switch (status) {
    case DFW_EDID_OK:              return "ok";
    case DFW_EDID_E_NULL:          return "null argument";
    case DFW_EDID_E_TOO_SHORT:     return "block shorter than 128 bytes";
    case DFW_EDID_E_HEADER:        return "bad 8-byte header";
    case DFW_EDID_E_CHECKSUM:      return "checksum mismatch";
    case DFW_EDID_E_VERSION:       return "unsupported EDID revision";
    case DFW_EDID_E_MFG_ID:        return "manufacturer ID not A-Z";
    case DFW_EDID_E_DTD_PCLK_ZERO: return "no timing DTD (all slots are monitor descriptors)";
    case DFW_EDID_E_DTD_RANGE:     return "DTD geometry out of range";
    case DFW_EDID_E_DTD_DUPLICATE: return "duplicate timing DTD";
    default:                       return "unknown";
    }
}

uint8_t dfw_edid_compute_checksum(const uint8_t *block)
{
    unsigned sum = 0;
    int i;
    for (i = 0; i < 127; ++i)
        sum += block[i];
    return (uint8_t)((256u - (sum & 0xFFu)) & 0xFFu);
}

int dfw_edid_checksum_ok(const uint8_t *block)
{
    unsigned sum = 0;
    int i;
    if (!block) return 0;
    for (i = 0; i < 128; ++i)
        sum += block[i];
    return (sum & 0xFFu) == 0u;
}

void dfw_edid_fix_checksum(uint8_t *block)
{
    if (!block) return;
    block[127] = 0;
    block[127] = dfw_edid_compute_checksum(block);
}

/* ---------------------------------------------------------------- *
 * descriptor decoding
 * ---------------------------------------------------------------- */

static void decode_dtd(const uint8_t *p, dfw_edid_dtd *d)
{
    uint32_t pclk10k;

    memset(d, 0, sizeof *d);

    pclk10k = (uint32_t)p[0] | ((uint32_t)p[1] << 8);
    d->pixel_clock_mhz = (double)pclk10k / 100.0;   /* 10 kHz units -> MHz */

    d->h_active = (uint32_t)p[2] | (((uint32_t)p[4] >> 4) << 8);
    d->h_blank  = (uint32_t)p[3] | (((uint32_t)p[4] & 0x0Fu) << 8);
    d->v_active = (uint32_t)p[5] | (((uint32_t)p[7] >> 4) << 8);
    d->v_blank  = (uint32_t)p[6] | (((uint32_t)p[7] & 0x0Fu) << 8);

    d->h_sync_off   = (uint32_t)p[8] | ((((uint32_t)p[11] >> 6) & 0x03u) << 8);
    d->h_sync_width = (uint32_t)p[9] | ((((uint32_t)p[11] >> 4) & 0x03u) << 8);
    d->v_sync_off   = (uint32_t)(p[10] >> 4) | ((((uint32_t)p[11] >> 2) & 0x03u) << 4);
    d->v_sync_width = (uint32_t)(p[10] & 0x0Fu) | (((uint32_t)p[11] & 0x03u) << 4);

    d->h_size_mm = (uint32_t)p[12] | (((uint32_t)p[14] >> 4) << 8);
    d->v_size_mm = (uint32_t)p[13] | (((uint32_t)p[14] & 0x0Fu) << 8);
    d->h_border  = p[15];
    d->v_border  = p[16];

    d->interlaced = (p[17] & 0x80u) ? 1 : 0;
    d->sync_type  = (p[17] >> 3) & 0x03;

    /* The meaning of bits 2..0 of byte 17 depends on the sync type. */
    switch (d->sync_type) {
    case 0: /* analogue composite */
        d->sync_serration = (p[17] & 0x04u) ? 1 : 0;
        d->sync_on_green  = (p[17] & 0x02u) ? 1 : 0;
        break;
    case 1: /* bipolar analogue composite */
        d->sync_serration = (p[17] & 0x04u) ? 1 : 0;
        d->sync_on_green  = (p[17] & 0x02u) ? 1 : 0;
        break;
    case 2: /* digital separate */
        d->v_sync_positive = (p[17] & 0x04u) ? 1 : 0;
        d->h_sync_positive = (p[17] & 0x02u) ? 1 : 0;
        break;
    case 3: /* digital composite (sync on all three / green) */
        d->sync_serration  = (p[17] & 0x04u) ? 1 : 0;
        d->sync_on_green   = (p[17] & 0x02u) ? 1 : 0;
        break;
    default:
        break;
    }

    d->h_total = d->h_active + d->h_blank;
    d->v_total = d->v_active + d->v_blank;

    if (d->h_total > 0u) {
        d->h_freq_khz = d->pixel_clock_mhz * 1000.0 / (double)d->h_total;
        if (d->v_total > 0u) {
            d->refresh_hz = d->pixel_clock_mhz * 1000000.0 /
                            ((double)d->h_total * (double)d->v_total);
        }
    }
    d->v_freq_hz = d->refresh_hz;
}

int dfw_edid_decode_dtd(const uint8_t *block18, dfw_edid_dtd *out)
{
    if (!block18 || !out) return DFW_EDID_E_NULL;
    if (block18[0] == 0x00 && block18[1] == 0x00)
        return DFW_EDID_E_DTD_PCLK_ZERO;
    decode_dtd(block18, out);
    if (out->h_total == 0u || out->v_total == 0u)
        return DFW_EDID_E_DTD_RANGE;
    return DFW_EDID_OK;
}

/* Decode an 18-byte monitor descriptor (first two bytes are zero). */
static void decode_monitor_desc(const uint8_t *b, dfw_edid_desc *d)
{
    uint8_t tag;
    int i;

    memset(d, 0, sizeof *d);
    /* VESA layout of an 18-byte monitor descriptor:
     *   b[0..1] pixel clock (0 for descriptors)   b[2] 0x00
     *   b[3]    descriptor tag                    b[4] 0x00 (flag)
     *   b[5..17] 13 bytes of payload
     * NOTE: an earlier revision of this file had the tag at b[5] and the
     * payload at b[6..], i.e. shifted by two bytes.  That round-tripped
     * against its own builder but could never parse a real monitor's EDID.
     * The test suite caught it by hand-assembling a descriptor from the
     * specification; see README "pitfalls". */
    if (b[2] != 0x00 || b[4] != 0x00) {
        d->type = DFW_DESC_OTHER;
        return;
    }
    tag = b[3];
    switch (tag) {
    case 0xFF:
        d->type = DFW_DESC_SERIAL;
        for (i = 0; i < 13; ++i) d->text[i] = (char)b[5 + i];
        d->text[13] = '\0';
        break;
    case 0xFE:
        d->type = DFW_DESC_ASCII;
        for (i = 0; i < 13; ++i) d->text[i] = (char)b[5 + i];
        d->text[13] = '\0';
        break;
    case 0xFC:
        d->type = DFW_DESC_NAME;
        for (i = 0; i < 13; ++i) d->text[i] = (char)b[5 + i];
        d->text[13] = '\0';
        break;
    case 0xFD:
        d->type = DFW_DESC_RANGE_LIMITS;
        d->has_range = 1;
        d->v_min_hz = (double)b[5];
        d->v_max_hz = (double)b[6];
        d->h_min_khz = (double)b[7];
        d->h_max_khz = (double)b[8];
        /* byte 9 is the maximum pixel clock in units of 10 MHz;
         * 0xFF means "use the GTF formula instead of a limit". */
        d->max_pixel_clock_mhz = (b[9] == 0xFFu) ? 0.0 : (double)b[9] * 10.0;
        break;
    case 0xFB:
        d->type = DFW_DESC_COLOR_POINT;
        break;
    case 0xFA:
        d->type = DFW_DESC_STD_TIMING_IDS;
        break;
    case 0x10:
        d->type = DFW_DESC_DUMMY;
        break;
    default:
        d->type = DFW_DESC_OTHER;
        break;
    }
}

static void decode_std_timing(const uint8_t *b, dfw_edid_std_timing *st)
{
    memset(st, 0, sizeof *st);
    if (b[0] == 0x00) return;                  /* unused slot */
    if (b[0] == 0x01 && b[1] == 0x01) return;  /* "no timing" marker */
    st->h_active   = ((uint32_t)b[0] + 31u) * 8u;
    st->ratio_code = (int)(b[1] >> 6);
    st->refresh_hz = (double)((b[1] & 0x3Fu) + 60u);
    switch (st->ratio_code) {
    case 0:  /* 16:10 */
        st->v_active = (st->h_active * 10u + 8u) / 16u;
        break;
    case 1:  /* 4:3 */
        st->v_active = (st->h_active * 3u + 2u) / 4u;
        break;
    case 2:  /* 5:4 */
        st->v_active = (st->h_active * 4u + 2u) / 5u;
        break;
    case 3:  /* 16:9 */
        st->v_active = (st->h_active * 9u + 8u) / 16u;
        break;
    default:
        st->v_active = 0u;
        break;
    }
}

/* ---------------------------------------------------------------- *
 * parse
 * ---------------------------------------------------------------- */

static int parse_mfg(const uint8_t *b, char out[4])
{
    uint16_t packed = ((uint16_t)b[0] << 8) | b[1];
    int i;
    for (i = 0; i < 3; ++i) {
        int shift = 10 - 5 * i;
        int v = (int)((packed >> shift) & 0x1Fu);
        if (v < 1 || v > 26)
            return DFW_EDID_E_MFG_ID;
        out[i] = (char)('A' + v - 1);
    }
    out[3] = '\0';
    return DFW_EDID_OK;
}

/* sqrt via Newton-Raphson; written by hand so the project needs no libm. */
static double dfw_sqrt(double v)
{
    double r;
    int i;
    if (v <= 0.0) return 0.0;
    r = v;
    for (i = 0; i < 40; ++i) {
        double nr = 0.5 * (r + v / r);
        if (nr == r) break;
        r = nr;
    }
    return r;
}

int dfw_edid_parse(const uint8_t *data, size_t len, dfw_edid *out)
{
    size_t i;
    int rc;

    if (!data || !out) return DFW_EDID_E_NULL;
    memset(out, 0, sizeof *out);
    if (len < DFW_EDID_BLOCK_SIZE) {
        out->status = DFW_EDID_E_TOO_SHORT;
        return out->status;
    }
    memcpy(out->block, data, DFW_EDID_BLOCK_SIZE);

    if (memcmp(data, edid_magic, 8) != 0) {
        out->status = DFW_EDID_E_HEADER;
        return out->status;
    }
    if (!dfw_edid_checksum_ok(data)) {
        out->status = DFW_EDID_E_CHECKSUM;
        return out->status;
    }

    out->mfg_id = ((uint16_t)data[8] << 8) | data[9];
    rc = parse_mfg(data + 8, out->mfg_letters);
    if (rc != DFW_EDID_OK) {
        out->status = rc;
        return out->status;
    }
    out->product_code  = (uint16_t)data[10] | ((uint16_t)data[11] << 8);
    out->serial_number = (uint32_t)data[12] | ((uint32_t)data[13] << 8) |
                         ((uint32_t)data[14] << 16) | ((uint32_t)data[15] << 24);
    out->week = data[16];
    out->year = data[17];

    out->version  = data[18];
    out->revision = data[19];
    if (out->version != 1u || out->revision < 3u) {
        out->status = DFW_EDID_E_VERSION;
        return out->status;
    }

    out->video_input_byte = data[20];
    out->input_type = (uint8_t)((data[20] >> 7) & 0x01u);
    if (out->input_type) {
        out->bit_depth      = (uint8_t)(((data[20] >> 4) & 0x07u) + 4u);
        out->interface_type = (uint8_t)(data[20] & 0x0Fu);
    }

    /* Bytes 21/22 are the screen size in CENTIMETRES (EDID 1.4 table 3.13),
     * while a DTD stores its size in millimetres.  Treating these two as mm
     * made every panel report as a 2.3" monitor; the simulation's 52x29 cm
     * entry is what exposed it.  Stored in mm so the field name is honest. */
    out->image_w_mm = (uint32_t)data[21] * 10u;
    out->image_h_mm = (uint32_t)data[22] * 10u;
    out->gamma = (data[23] == 0xFFu) ? 0.0 : ((double)data[23] + 100.0) / 100.0;

    out->supports_dpms_standby      = (data[24] & 0x80u) ? 1 : 0;
    out->supports_dpms_suspend      = (data[24] & 0x40u) ? 1 : 0;
    out->supports_dpms_active_off   = (data[24] & 0x20u) ? 1 : 0;
    out->srgb_default               = (data[24] & 0x04u) ? 1 : 0;
    out->preferred_timing_is_native = (data[24] & 0x02u) ? 1 : 0;
    out->continuous_frequency       = (data[24] & 0x01u) ? 1 : 0;
    out->has_extended_timings       = out->continuous_frequency;

    /* chromaticity is 10 bits per coordinate, low 2 bits packed in 25/26 */
    out->red_x   = (double)(((uint32_t)data[27] << 2) | ((data[25] >> 6) & 0x03u)) / 1024.0;
    out->red_y   = (double)(((uint32_t)data[28] << 2) | ((data[25] >> 4) & 0x03u)) / 1024.0;
    out->green_x = (double)(((uint32_t)data[29] << 2) | ((data[25] >> 2) & 0x03u)) / 1024.0;
    out->green_y = (double)(((uint32_t)data[30] << 2) | (data[25] & 0x03u)) / 1024.0;
    out->blue_x  = (double)(((uint32_t)data[31] << 2) | ((data[26] >> 6) & 0x03u)) / 1024.0;
    out->blue_y  = (double)(((uint32_t)data[32] << 2) | ((data[26] >> 4) & 0x03u)) / 1024.0;
    out->white_x = (double)(((uint32_t)data[33] << 2) | ((data[26] >> 2) & 0x03u)) / 1024.0;
    out->white_y = (double)(((uint32_t)data[34] << 2) | (data[26] & 0x03u)) / 1024.0;

    memcpy(out->established_timings, data + 35, 3);

    if (out->image_w_mm > 0u && out->image_h_mm > 0u) {
        double w = (double)out->image_w_mm;
        double h = (double)out->image_h_mm;
        out->aspect_ratio  = w / h;
        out->diagonal_inch = dfw_sqrt(w * w + h * h) * 0.0393701; /* mm -> inch */
    } else if (out->image_w_mm > 0u) {
        /* aspect ratio stored as (raw_byte + 99) / 100 -- the raw byte carries
         * the ratio, not the millimetre value derived from it */
        out->aspect_ratio = ((double)data[21] + 99.0) / 100.0;
    } else if (out->image_h_mm > 0u) {
        out->aspect_ratio = 100.0 / ((double)data[22] + 99.0);
    } else {
        out->aspect_ratio = 0.0;
    }

    /* standard timings */
    out->std_count = 0;
    for (i = 0; i < DFW_EDID_STD_TIMINGS; ++i) {
        dfw_edid_std_timing st;
        decode_std_timing(data + 38 + 2 * i, &st);
        if (st.h_active == 0u) continue;
        if (out->std_count < DFW_EDID_STD_TIMINGS) {
            out->std_timings[out->std_count] = st;
            out->std_count++;
        }
    }

    /* the four 18-byte descriptor slots */
    out->dtd_count = 0;
    for (i = 0; i < DFW_EDID_BASE_DTDS; ++i) {
        const uint8_t *slot = data + 54 + 18 * i;
        if (slot[0] == 0x00 && slot[1] == 0x00) {
            /* pixel clock 0 => monitor descriptor, NOT a timing.  We never
             * invent a mode for this slot. */
            decode_monitor_desc(slot, &out->descs[i]);
        } else {
            dfw_edid_dtd d;
            if (dfw_edid_decode_dtd(slot, &d) == DFW_EDID_OK) {
                if (out->dtd_count < DFW_EDID_MAX_DTDS) {
                    out->dtds[out->dtd_count] = d;
                    out->dtd_count++;
                }
            }
            out->descs[i].type = DFW_DESC_NONE;
        }
    }

    out->extensions = data[126];
    if (out->extensions > 0u && len >= DFW_EDID_BLOCK_SIZE * 2u) {
        const uint8_t *ext = data + DFW_EDID_BLOCK_SIZE;
        uint8_t tag = ext[0];
        if (tag == 0x02u) out->has_cea861 = 1;
        if (tag == 0x70u) out->has_displayid = 1;
        if (dfw_edid_checksum_ok(ext) && tag == 0x02u) {
            /* CEA-861: the data block collection ends at byte
             * 2 + (byte 2 & 0x1F); timing DTDs follow it. */
            size_t off = (size_t)2u + (size_t)(ext[2] & 0x1Fu);
            if (off < 4u) off = 4u;
            while (off + 18u <= 127u) {
                dfw_edid_dtd d;
                if (ext[off] == 0x00 && ext[off + 1] == 0x00) break;
                if (dfw_edid_decode_dtd(ext + off, &d) == DFW_EDID_OK) {
                    if (out->dtd_count < DFW_EDID_MAX_DTDS) {
                        out->dtds[out->dtd_count] = d;
                        out->dtd_count++;
                    }
                }
                off += 18u;
            }
        }
    }

    if (out->dtd_count == 0u) {
        out->status = DFW_EDID_E_DTD_PCLK_ZERO;
        return out->status;
    }

    out->valid  = 1;
    out->status = DFW_EDID_OK;
    return DFW_EDID_OK;
}

/* ---------------------------------------------------------------- *
 * expected mode list
 * ---------------------------------------------------------------- */

typedef struct { uint32_t h, v; double hz; } known_mode;

/* Established timings I/II/III (excluding the manufacturer-reserved bits),
 * in little-endian bit order across the 3-byte bitmap at offset 35..37. */
static const known_mode known_modes[] = {
    { 720u,  400u,  70.0 }, { 720u,  400u,  85.0 },
    { 640u,  480u,  60.0 }, { 640u,  480u,  72.0 },
    { 640u,  480u,  75.0 }, { 640u,  480u,  85.0 },
    { 800u,  600u,  56.0 }, { 800u,  600u,  60.0 },
    { 800u,  600u,  72.0 }, { 800u,  600u,  75.0 },
    { 800u,  600u,  85.0 }, { 832u,  624u,  75.0 },
    { 1024u, 768u,  87.0 }, { 1024u, 768u,  60.0 },
    { 1024u, 768u,  70.0 }, { 1024u, 768u,  75.0 },
    { 1280u, 1024u, 75.0 }, { 1152u, 870u,  75.0 },
    { 1280u, 1024u, 85.0 }, { 1280u, 960u,  60.0 },
    { 1280u, 960u,  85.0 }, { 1280u, 1024u, 60.0 },
    { 1400u, 1050u, 60.0 }, { 1400u, 1050u, 75.0 },
    { 1440u, 900u,  60.0 }, { 1440u, 900u,  75.0 },
    { 1600u, 1200u, 60.0 }, { 1600u, 1200u, 65.0 },
    { 1600u, 1200u, 70.0 }, { 1600u, 1200u, 75.0 },
    { 1680u, 1050u, 60.0 }, { 1680u, 1050u, 75.0 },
    { 1792u, 1344u, 60.0 }, { 1792u, 1344u, 75.0 },
    { 1856u, 1392u, 60.0 }, { 1856u, 1392u, 75.0 },
    { 1920u, 1200u, 60.0 }, { 1920u, 1200u, 75.0 },
    { 1920u, 1440u, 60.0 }, { 1920u, 1440u, 75.0 }
};

static int bit_at(const uint8_t bm[3], size_t n)
{
    if (n >= 24u) return 0;
    return (bm[n / 8u] >> (n % 8u)) & 1u;
}

/* Does the EDID advertise a mode close to (h, v, hz)?
 * `established` enables the bitmap check, which is the only place where the
 * classic mode list is decoded from real bits rather than assumed. */
static int edid_advertises(const dfw_edid *e, uint32_t h, uint32_t v, double hz)
{
    size_t i;
    for (i = 0; i < e->dtd_count; ++i) {
        const dfw_edid_dtd *d = &e->dtds[i];
        double dh = d->refresh_hz - hz;
        if (dh < 0.0) dh = -dh;
        if (d->h_active == h && d->v_active == v && dh < 0.6) return 1;
    }
    for (i = 0; i < e->std_count; ++i) {
        const dfw_edid_std_timing *s = &e->std_timings[i];
        double dh = s->refresh_hz - hz;
        if (dh < 0.0) dh = -dh;
        if (s->h_active == h && s->v_active == v && dh < 0.6) return 1;
    }
    for (i = 0; i < sizeof known_modes / sizeof known_modes[0]; ++i) {
        const known_mode *k = &known_modes[i];
        double dh = k->hz - hz;
        if (dh < 0.0) dh = -dh;
        if (k->h == h && k->v == v && dh < 0.6 && bit_at(e->established_timings, i))
            return 1;
    }
    return 0;
}

size_t dfw_edid_expected_modes(const dfw_edid *e, dfw_mode *expected, size_t cap)
{
    /* Candidate pool of modes a 1080p-class scaler might be asked to show.
     * A candidate only reaches the output if the EDID actually advertises
     * it, so the result is a function of the parsed data, not a constant
     * table: corrupt a DTD and the mode disappears from the list. */
    static const dfw_mode pool[] = {
        { 640u,  480u,  60.0 }, { 640u,  480u,  72.0 }, { 640u,  480u,  75.0 },
        { 800u,  600u,  60.0 }, { 800u,  600u,  72.0 }, { 800u,  600u,  75.0 },
        { 1024u, 768u,  60.0 }, { 1024u, 768u,  70.0 }, { 1024u, 768u,  75.0 },
        { 1152u, 864u,  75.0 },
        { 1280u, 720u,  60.0 }, { 1280u, 800u,  60.0 },
        { 1280u, 1024u, 60.0 }, { 1280u, 1024u, 75.0 },
        { 1366u, 768u,  60.0 }, { 1440u, 900u,  60.0 },
        { 1600u, 900u,  60.0 }, { 1600u, 1200u, 60.0 },
        { 1680u, 1050u, 60.0 },
        { 1920u, 1080u, 50.0 }, { 1920u, 1080u, 60.0 }, { 1920u, 1200u, 60.0 },
        { 2560u, 1440u, 60.0 }, { 3840u, 2160u, 60.0 }
    };
    size_t n = 0;
    size_t i;
    if (!e || !expected || cap == 0) return 0;
    for (i = 0; i < sizeof pool / sizeof pool[0] && n < cap; ++i) {
        if (edid_advertises(e, pool[i].h_active, pool[i].v_active,
                            pool[i].refresh_hz))
            expected[n++] = pool[i];
    }
    return n;
}

/* ---------------------------------------------------------------- *
 * synthetic builders (tests + simulator only)
 * ---------------------------------------------------------------- */

void dfw_edid_build_dtd(uint8_t *b, const dfw_edid_dtd *s)
{
    uint32_t pclk10k;
    if (!b || !s) return;
    memset(b, 0, 18);
    pclk10k = (uint32_t)(s->pixel_clock_mhz * 100.0 + 0.5);
    b[0] = (uint8_t)(pclk10k & 0xFFu);
    b[1] = (uint8_t)((pclk10k >> 8) & 0xFFu);
    b[2] = (uint8_t)(s->h_active & 0xFFu);
    b[3] = (uint8_t)(s->h_blank & 0xFFu);
    b[4] = (uint8_t)((((s->h_active >> 8) & 0x0Fu) << 4) |
                     ((s->h_blank >> 8) & 0x0Fu));
    b[5] = (uint8_t)(s->v_active & 0xFFu);
    b[6] = (uint8_t)(s->v_blank & 0xFFu);
    b[7] = (uint8_t)((((s->v_active >> 8) & 0x0Fu) << 4) |
                     ((s->v_blank >> 8) & 0x0Fu));
    b[8] = (uint8_t)(s->h_sync_off & 0xFFu);
    b[9] = (uint8_t)(s->h_sync_width & 0xFFu);
    b[10] = (uint8_t)(((s->v_sync_off & 0x0Fu) << 4) | (s->v_sync_width & 0x0Fu));
    b[11] = (uint8_t)((((s->h_sync_off >> 8) & 0x03u) << 6) |
                      (((s->h_sync_width >> 8) & 0x03u) << 4) |
                      (((s->v_sync_off >> 4) & 0x03u) << 2) |
                      ((s->v_sync_width >> 4) & 0x03u));
    b[12] = (uint8_t)(s->h_size_mm & 0xFFu);
    b[13] = (uint8_t)(s->v_size_mm & 0xFFu);
    b[14] = (uint8_t)((((s->h_size_mm >> 8) & 0x0Fu) << 4) |
                      ((s->v_size_mm >> 8) & 0x0Fu));
    b[15] = (uint8_t)(s->h_border & 0xFFu);
    b[16] = (uint8_t)(s->v_border & 0xFFu);
    /* byte 17: interlaced | stereo(0) | sync type | serration |
     *          (V polarity or sync-on-green) | (H polarity or composite)  */
    b[17] = (uint8_t)((s->interlaced ? 0x80u : 0u) |
                      ((uint32_t)(s->sync_type & 0x03) << 3));
    if (s->sync_type == 2) {
        if (s->v_sync_positive) b[17] |= 0x04u;
        if (s->h_sync_positive) b[17] |= 0x02u;
    } else {
        if (s->sync_serration) b[17] |= 0x04u;
        if (s->sync_on_green)  b[17] |= 0x02u;
    }
}

static void build_desc_header(uint8_t *b, uint8_t tag)
{
    memset(b, 0, 18);
    b[3] = tag;          /* VESA: tag at byte 3, payload starts at byte 5 */
}

static void copy_padded(uint8_t *dst, const char *src)
{
    size_t i, n = 0;
    /* Measure first, then copy exactly n bytes.  The previous version read
     * src[i] for all 13 slots and only used it when non-zero -- which reads
     * past the terminating NUL of any shorter string literal.  ASan caught it
     * as a global-buffer-overflow (see README "pitfalls"); it is harmless on
     * most targets and silently wrong on all of them. */
    if (src != NULL) {
        n = strlen(src);
        if (n > 13) n = 13;
    }
    for (i = 0; i < 13; ++i) {
        dst[i] = (i < n) ? (uint8_t)src[i] : (uint8_t)' ';
    }
    if (n < 13) dst[n] = '\n';   /* EDID strings are newline terminated */
}

void dfw_edid_build_desc_name(uint8_t *b, const char *name)
{
    build_desc_header(b, 0xFC);
    copy_padded(b + 5, name);
}

void dfw_edid_build_desc_serial(uint8_t *b, const char *serial)
{
    build_desc_header(b, 0xFF);
    copy_padded(b + 5, serial);
}

void dfw_edid_build_desc_range(uint8_t *b, double v_min, double v_max,
                               double h_min_khz, double h_max_khz,
                               double max_pclk_mhz)
{
    build_desc_header(b, 0xFD);
    b[5] = (uint8_t)(v_min + 0.5);
    b[6] = (uint8_t)(v_max + 0.5);
    b[7] = (uint8_t)(h_min_khz + 0.5);
    b[8] = (uint8_t)(h_max_khz + 0.5);
    /* units of 10 MHz -- writing MHz*10 here (as an earlier revision did)
     * silently lost any monitor above 25.5 MHz. */
    b[9] = (uint8_t)((max_pclk_mhz / 10.0) + 0.5);
    b[10] = 0x00;
}
