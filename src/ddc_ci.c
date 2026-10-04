/*
 * ddc_ci.c -- DDC/CI message layer, both sides of the wire.
 *
 * ---------------------------------------------------------------------
 * WIRE FORMAT (VESA DDC/CI 1.1 / MCCS)
 * ---------------------------------------------------------------------
 * Every packet, host -> monitor and monitor -> host, has the same shape:
 *
 *   byte 0        destination address (7-bit address << 1)
 *                   host    = 0x51
 *                   display = 0x6E
 *                 (0x37/0x50 is the DDC 2B pair, not used here)
 *   byte 1        source address
 *   byte 2        length = 2 + payload length.  It counts the two address
 *                 bytes and itself, so a Get VCP request has length 4 and
 *                 the frame is 6 bytes on the wire.
 *   byte 3..n-2   payload
 *   byte n-1      XOR checksum over bytes 0..n-2
 *
 * Payloads implemented here:
 *   Get VCP Request        01  <vcp_hi> <vcp_lo>              (len 4)
 *   Get VCP Feature Reply  02  <result> <vcp_hi> <vcp_lo> <type>
 *                              <value_hi> <value_lo> <max_lo> (len 10)
 *   Set VCP Feature        03  <vcp_hi> <vcp_lo> <value>      (len 7)
 *   Save Current Settings  0C  <unused = 0>                   (len 4)
 *   Capabilities Request   F3  <unused = 0>                   (len 4)
 *   Capabilities Reply     E3  <offset_hi> <offset_lo> <ascii...>
 *                                                        (len 2+n, n <= 32)
 *
 * Result codes inside a Get VCP reply:
 *   00 = supported, 01 = unsupported VCP code, 81 = checksum error
 *
 * The "type" byte after the VCP code describes the *shape* of the value,
 * not a C data type:
 *   bit7     0 = continuous value, 1 = table ("enumerated") value
 *   bits2..0 000 set parameter, 001 momentary, 010 read-only
 * Brightness and contrast are continuous set parameters (0x00), the input
 * source and power mode are table values (0x80), and the firmware version
 * register is read-only (0x02).  Getting this wrong makes a host display
 * "unsupported" for a control the monitor actually has.
 *
 * Set VCP carries only the low byte of the value in the payload: for
 * continuous controls above 0xFF the high byte travels in the message-type
 * slot, which is why a table value such as the input source fits in one
 * byte while a 16-bit control would need the extra slot.
 *
 * ---------------------------------------------------------------------
 * WHAT IS *NOT* HERE
 * ---------------------------------------------------------------------
 * There is no I2C master, no bus arbitration, no clock stretching and no
 * 40 ms timeout measured in real time.  dfw_ddc_host_transact() is a
 * synchronous function call that models one transaction including the
 * retry policy; the timeout budget is a counter, and the slave can be told
 * to stall (abort_next) or corrupt its reply (drop_next_checksum) so the
 * host's error handling is exercised by the tests instead of being
 * asserted by a comment.
 */
#include "dfw.h"

#include <string.h>

#define DDC_HEADER_BYTES  3u
#define DDC_TRAILER_BYTES 1u
#define DDC_MIN_PACKET    (DDC_HEADER_BYTES + DDC_TRAILER_BYTES)
#define DDC_GET_REPLY_LEN 12u   /* 3 header + 8 payload + 1 checksum */
#define DDC_MAX_VCP       16u

const char *dfw_ddc_status_str(int status)
{
    switch (status) {
    case DFW_DDC_OK:            return "ok";
    case DFW_DDC_E_NULL:        return "null argument";
    case DFW_DDC_E_BAD_DST:     return "destination address is not this display";
    case DFW_DDC_E_BAD_LENGTH:  return "length field inconsistent with frame";
    case DFW_DDC_E_CHECKSUM:    return "XOR checksum mismatch";
    case DFW_DDC_E_TIMEOUT:     return "no reply within the timeout budget";
    case DFW_DDC_E_TRUNCATED:   return "reply shorter than its own length field";
    case DFW_DDC_E_NO_RESPONSE: return "monitor returned no bytes";
    case DFW_DDC_E_IO:          return "no slave attached to the bus";
    case DFW_DDC_E_BAD_OPCODE:  return "unknown DDC/CI opcode";
    case DFW_DDC_E_NO_SPACE:    return "output buffer too small";
    default:                    return "unknown";
    }
}

/* ---------------------------------------------------------------- *
 * checksum / framing
 * ---------------------------------------------------------------- */

uint8_t dfw_ddc_xor_checksum(const uint8_t *buf, size_t len)
{
    uint8_t x = 0;
    size_t i;
    if (!buf) return 0;
    for (i = 0; i < len; ++i) x ^= buf[i];
    return x;
}

int dfw_ddc_verify_checksum(const uint8_t *buf, size_t len)
{
    if (!buf || len < DDC_MIN_PACKET) return 0;
    return dfw_ddc_xor_checksum(buf, len) == 0u;
}

int dfw_ddc_build(uint8_t *out, size_t cap, size_t *out_len,
                  uint8_t dst, uint8_t src,
                  const uint8_t *payload, size_t payload_len)
{
    size_t total;
    if (!out || !out_len) return DFW_DDC_E_NULL;
    if (payload_len > 0u && !payload) return DFW_DDC_E_NULL;
    if (payload_len + 2u > 0xFFu) return DFW_DDC_E_NO_SPACE;
    total = DDC_HEADER_BYTES + payload_len + DDC_TRAILER_BYTES;
    if (total > cap) return DFW_DDC_E_NO_SPACE;
    out[0] = dst;
    out[1] = src;
    out[2] = (uint8_t)(payload_len + 2u);
    if (payload_len > 0u)
        memcpy(out + DDC_HEADER_BYTES, payload, payload_len);
    out[total - 1u] = dfw_ddc_xor_checksum(out, total - 1u);
    *out_len = total;
    return DFW_DDC_OK;
}

/* ---------------------------------------------------------------- *
 * MCCS value types
 * ---------------------------------------------------------------- */

static uint8_t vcp_default_type(uint16_t code)
{
    switch (code) {
    case DFW_VCP_BRIGHTNESS:   return 0x00; /* continuous, set parameter */
    case DFW_VCP_CONTRAST:     return 0x00;
    case DFW_VCP_MUTE:         return 0x80; /* table                     */
    case DFW_VCP_COLOR_PRESET: return 0x80;
    case DFW_VCP_INPUT_SOURCE: return 0x80;
    case DFW_VCP_POWER_MODE:   return 0x80;
    case DFW_VCP_VERSION:      return 0x02; /* read-only                 */
    default:                   return 0x00;
    }
}

/* ---------------------------------------------------------------- *
 * slave
 * ---------------------------------------------------------------- */

int dfw_ddc_slave_add_vcp(dfw_ddc_slave *s, uint16_t code, uint16_t value,
                          uint16_t max_value, uint8_t type)
{
    if (!s) return DFW_DDC_E_NULL;
    if (s->vcp_count >= DDC_MAX_VCP) return DFW_DDC_E_NO_SPACE;
    if (type == 0xFFu) type = vcp_default_type(code);
    s->vcp[s->vcp_count].code      = code;
    s->vcp[s->vcp_count].value     = (value > max_value) ? max_value : value;
    s->vcp[s->vcp_count].max_value = max_value;
    s->vcp[s->vcp_count].type      = type;
    s->vcp[s->vcp_count].supported = 1;
    s->vcp_count++;
    return DFW_DDC_OK;
}

dfw_vcp_entry *dfw_ddc_slave_find(dfw_ddc_slave *s, uint16_t code)
{
    size_t i;
    if (!s) return NULL;
    for (i = 0; i < s->vcp_count; ++i)
        if (s->vcp[i].code == code) return &s->vcp[i];
    return NULL;
}

/* Values are typical settings for a 1080p-class panel, not measurements of
 * any real product.  input source: 0x0F DP-1, 0x11 HDMI-1, 0x12 HDMI-2,
 * 0x1B USB Type-C.  power mode: 0x01 on, 0x05 off. */
static void slave_seed_vcp_table(dfw_ddc_slave *s)
{
    (void)dfw_ddc_slave_add_vcp(s, DFW_VCP_BRIGHTNESS,   50u,    100u,  0x00);
    (void)dfw_ddc_slave_add_vcp(s, DFW_VCP_CONTRAST,     50u,    100u,  0x00);
    (void)dfw_ddc_slave_add_vcp(s, DFW_VCP_COLOR_PRESET, 0x01u,  0x05u, 0x80);
    (void)dfw_ddc_slave_add_vcp(s, DFW_VCP_INPUT_SOURCE, 0x11u,  0x1Bu, 0x80);
    (void)dfw_ddc_slave_add_vcp(s, DFW_VCP_POWER_MODE,   0x01u,  0x05u, 0x80);
    /* version 0x0102 = MCCS 2.2 firmware interface revision */
    (void)dfw_ddc_slave_add_vcp(s, DFW_VCP_VERSION,      0x0102u, 0x00FFu, 0x02);
    (void)dfw_ddc_slave_add_vcp(s, DFW_VCP_MUTE,         0x02u,  0x02u, 0x80);
}

void dfw_ddc_slave_init(dfw_ddc_slave *s)
{
    /* Capabilities string in the DDC/CI checksum-protected form:
     *   (prototype) (0x00,0x00) type (0x00,0x00) (length) <ascii> (checksum)
     * The live length byte counts the ASCII payload only. */
    static const char caps[] =
        "prot(monitor)type(lcd)model(DFW-2701)cmds(01 02 03 0C F3)"
        "vcp(10 12 14 60 D6 DF 8D)mccs_ver(2.2)";
    if (!s) return;
    memset(s, 0, sizeof *s);
    s->address           = DFW_DDC_DST_DISPLAY;
    s->host_address      = DFW_DDC_SRC_HOST;
    s->powered_on        = 1;
    s->reply_delay_ticks = 1;
    strncpy(s->capabilities, caps, sizeof s->capabilities - 1u);
    s->capabilities[sizeof s->capabilities - 1u] = '\0';
    slave_seed_vcp_table(s);
}

static void slave_reply_get_vcp(dfw_ddc_slave *s, uint16_t code,
                                uint8_t *reply, size_t reply_cap,
                                size_t *reply_len)
{
    dfw_vcp_entry *e = dfw_ddc_slave_find(s, code);
    uint8_t payload[8];
    uint8_t result;

    if (!e) {
        s->unsupported_requests++;
        result = DFW_DDC_OP_UNSUPPORTED;
    } else {
        result = DFW_DDC_OP_OK;
    }

    payload[0] = DFW_DDC_GET_VCP_REPLY;
    payload[1] = result;
    payload[2] = (uint8_t)(code >> 8);
    payload[3] = (uint8_t)(code & 0xFFu);
    payload[4] = e ? e->type : 0x00;
    payload[5] = e ? (uint8_t)((e->value >> 8) & 0xFFu) : 0x00;
    payload[6] = e ? (uint8_t)(e->value & 0xFFu) : 0x00;
    payload[7] = e ? (uint8_t)(e->max_value & 0xFFu) : 0x00;

    /* Frame = 3 header + 8 payload + 1 checksum = 12 bytes, so the length
     * byte holds 10.  Current value and maximum travel in the same reply:
     * a Get VCP is one round trip, not two. */
    if (dfw_ddc_build(reply, reply_cap, reply_len,
                      s->host_address, s->address, payload, sizeof payload)
        != DFW_DDC_OK)
        *reply_len = 0;
}

static void slave_reply_checksum_error(dfw_ddc_slave *s, const uint8_t *req,
                                       uint8_t *reply, size_t reply_cap,
                                       size_t *reply_len)
{
    uint8_t pl[8];
    pl[0] = DFW_DDC_GET_VCP_REPLY;
    pl[1] = DFW_DDC_OP_CHECKSUM_ERROR;
    pl[2] = req[4];
    pl[3] = req[5];
    pl[4] = 0x00;
    pl[5] = 0x00;
    pl[6] = 0x00;
    pl[7] = 0x00;
    if (dfw_ddc_build(reply, reply_cap, reply_len,
                      s->host_address, s->address, pl, sizeof pl) != DFW_DDC_OK)
        *reply_len = 0;
}

int dfw_ddc_slave_handle(dfw_ddc_slave *s,
                         const uint8_t *req, size_t req_len,
                         uint8_t *reply, size_t reply_cap, size_t *reply_len,
                         int *delay_ticks_out)
{
    uint8_t payload[64];
    size_t payload_len = 0;
    size_t declared;
    uint8_t opcode;

    if (!s || !req || !reply || !reply_len) return DFW_DDC_E_NULL;
    *reply_len = 0;
    if (delay_ticks_out) *delay_ticks_out = 0;

    if (req_len < DDC_MIN_PACKET) {
        s->malformed_requests++;
        return DFW_DDC_E_TRUNCATED;
    }
    if (req[0] != s->address) {
        s->malformed_requests++;
        return DFW_DDC_E_BAD_DST;
    }
    /* The length byte must describe exactly the frame we received. */
    declared = (size_t)req[2];
    if (declared < 2u || declared + 2u != req_len) {
        s->malformed_requests++;
        return DFW_DDC_E_BAD_LENGTH;
    }
    if (dfw_ddc_xor_checksum(req, req_len) != 0u) {
        s->checksum_errors++;
        /* Real monitors answer a bad-checksum Get VCP with result 0x81
         * rather than staying silent, which is what host tooling keys off. */
        if (req_len >= 6u && req[3] == DFW_DDC_GET_VCP)
            slave_reply_checksum_error(s, req, reply, reply_cap, reply_len);
        return DFW_DDC_E_CHECKSUM;
    }

    if (s->abort_next) {
        /* Simulate a monitor that stops acknowledging: no bytes come back. */
        s->abort_next = 0;
        return DFW_DDC_E_NO_RESPONSE;
    }

    s->transactions++;
    if (delay_ticks_out) *delay_ticks_out = s->reply_delay_ticks;

    opcode = req[3];
    switch (opcode) {
    case DFW_DDC_GET_VCP:
        /* 01 <vcp_hi> <vcp_lo>: 3 payload bytes -> length byte 5 -> 7 total.
         * The VCP code is two bytes; a 6-byte frame cannot carry it, so
         * anything shorter is a malformed request rather than a short code. */
        if (req_len != 7u) {
            s->malformed_requests++;
            return DFW_DDC_E_BAD_LENGTH;
        }
        slave_reply_get_vcp(s, (uint16_t)(((uint16_t)req[4] << 8) | req[5]),
                            reply, reply_cap, reply_len);
        break;

    case DFW_DDC_SET_VCP: {
        /* 03 <vcp_hi> <vcp_lo> <value>: 4 payload bytes -> 8 total */
        uint16_t code, value;
        dfw_vcp_entry *e;
        if (req_len != 8u) {
            s->malformed_requests++;
            return DFW_DDC_E_BAD_LENGTH;
        }
        code  = (uint16_t)(((uint16_t)req[4] << 8) | req[5]);
        value = (uint16_t)req[6];
        e = dfw_ddc_slave_find(s, code);
        if (!e || e->type == DFW_DDC_MODE_READ_ONLY) {
            s->unsupported_requests++;
        } else {
            /* The monitor clamps to its own advertised maximum; it never
             * trusts the host to stay in range. */
            e->value = (value > e->max_value) ? e->max_value : value;
            if (code == DFW_VCP_POWER_MODE)
                s->powered_on = (e->value != 0x05u) ? 1 : 0;
        }
        /* Set VCP has no payload reply: an empty response means accepted. */
        break;
    }

    case DFW_DDC_SAVE_SETTINGS:
        s->saves++;
        break;

    case DFW_DDC_GET_CAPABILITIES: {
        size_t n = strlen(s->capabilities);
        if (n > 32u) n = 32u;   /* one capabilities segment holds <= 32 */
        payload[0] = DFW_DDC_CAP_REPLY;
        payload[1] = 0x00;      /* offset high */
        payload[2] = 0x00;      /* offset low  */
        memcpy(payload + 3, s->capabilities, n);
        payload_len = 3u + n;
        if (dfw_ddc_build(reply, reply_cap, reply_len,
                          s->host_address, s->address, payload, payload_len)
            != DFW_DDC_OK)
            *reply_len = 0;
        break;
    }

    default:
        s->malformed_requests++;
        return DFW_DDC_E_BAD_OPCODE;
    }

    /* fault injection: corrupt the reply checksum so the host must reject it */
    if (s->drop_next_checksum && *reply_len > 0u) {
        reply[*reply_len - 1u] ^= 0x5Au;
        s->drop_next_checksum = 0;
    }
    return DFW_DDC_OK;
}

/* ---------------------------------------------------------------- *
 * host
 * ---------------------------------------------------------------- */

void dfw_ddc_bus_init(dfw_ddc_bus *bus, dfw_ddc_slave *slave)
{
    if (!bus) return;
    memset(bus, 0, sizeof *bus);
    bus->slave = slave;
    bus->max_retries = 2;
    bus->timeout_ticks = 40;   /* DDC/CI allows 40 ms per transaction */
}

/* One attempt, no retry. */
static int ddc_attempt(dfw_ddc_bus *bus, const uint8_t *frame, size_t frame_len,
                       uint8_t *reply, size_t reply_cap, size_t *reply_len)
{
    int delay = 0;
    int rc;

    *reply_len = 0;
    rc = dfw_ddc_slave_handle(bus->slave, frame, frame_len,
                              reply, reply_cap, reply_len, &delay);
    if (rc == DFW_DDC_E_NO_RESPONSE || rc == DFW_DDC_E_BAD_OPCODE) {
        bus->stats.timeouts++;
        return DFW_DDC_E_TIMEOUT;
    }
    if (rc == DFW_DDC_E_CHECKSUM) {
        bus->stats.checksum_errors++;
        /* The monitor did answer, with result 0x81; keep the reply so the
         * caller can see the result code, but do not call it success. */
        return (*reply_len > 0u) ? DFW_DDC_OK : DFW_DDC_E_CHECKSUM;
    }
    if (rc != DFW_DDC_OK) return rc;
    if (bus->timeout_ticks > 0 && delay > bus->timeout_ticks) {
        bus->stats.timeouts++;
        *reply_len = 0;
        return DFW_DDC_E_TIMEOUT;
    }
    return DFW_DDC_OK;
}

int dfw_ddc_host_transact(dfw_ddc_bus *bus,
                          const uint8_t *payload, size_t payload_len,
                          uint8_t *reply, size_t reply_cap, size_t *reply_len)
{
    uint8_t frame[80];
    size_t frame_len = 0;
    int attempt;
    int rc;

    if (!bus || !bus->slave || !payload || !reply || !reply_len)
        return DFW_DDC_E_NULL;

    *reply_len = 0;
    rc = dfw_ddc_build(frame, sizeof frame, &frame_len,
                       bus->slave->address, bus->slave->host_address,
                       payload, payload_len);
    if (rc != DFW_DDC_OK) return rc;

    rc = DFW_DDC_E_TIMEOUT;
    for (attempt = 0; attempt <= bus->max_retries; ++attempt) {
        if (attempt > 0) bus->stats.retries++;
        rc = ddc_attempt(bus, frame, frame_len, reply, reply_cap, reply_len);
        if (rc == DFW_DDC_OK) {
            if (*reply_len == 0u) {
                /* Only commands with no reply payload may answer empty. */
                if (payload[0] == DFW_DDC_SET_VCP ||
                    payload[0] == DFW_DDC_SAVE_SETTINGS) {
                    bus->stats.transactions++;
                    return DFW_DDC_OK;
                }
                rc = DFW_DDC_E_NO_RESPONSE;
            } else {
                if ((size_t)reply[2] + 2u != *reply_len)
                    return DFW_DDC_E_TRUNCATED;
                if (!dfw_ddc_verify_checksum(reply, *reply_len)) {
                    bus->stats.checksum_errors++;
                    *reply_len = 0;
                    rc = DFW_DDC_E_CHECKSUM;
                } else {
                    bus->stats.transactions++;
                    return DFW_DDC_OK;
                }
            }
        }
    }
    return rc;
}

/* Parse a Get VCP reply frame (12 bytes) into `out`. */
static int ddc_parse_get_reply(const uint8_t *reply, size_t reply_len,
                               uint16_t want_code, dfw_ddc_reply *out)
{
    if (reply_len < DDC_GET_REPLY_LEN) return DFW_DDC_E_TRUNCATED;
    if (reply[3] != DFW_DDC_GET_VCP_REPLY) return DFW_DDC_E_BAD_OPCODE;
    out->result_code  = reply[4];
    out->vcp_code     = (uint16_t)(((uint16_t)reply[5] << 8) | reply[6]);
    out->message_type = reply[7];
    out->value        = (uint16_t)(((uint16_t)reply[8] << 8) | reply[9]);
    out->max_value    = (uint16_t)reply[10];
    if (out->vcp_code != want_code) return DFW_DDC_E_BAD_OPCODE;
    return DFW_DDC_OK;
}

int dfw_ddc_get_vcp(dfw_ddc_bus *bus, uint16_t code, dfw_ddc_reply *out)
{
    uint8_t payload[3];
    uint8_t reply[64];
    size_t reply_len = 0;
    int rc;

    if (!bus || !out) return DFW_DDC_E_NULL;
    memset(out, 0, sizeof *out);

    /* Get VCP Feature Request is 01 <vcp_hi> <vcp_lo>: the code is TWO
     * bytes.  Sending only the low byte (as an earlier revision did) makes
     * the frame 6 bytes long, and the slave then reads the checksum as the
     * low byte -- so the reply came back for VCP 0x10xx and the host
     * rejected it with BAD_OPCODE.  Every VCP whose code has a non-zero
     * high byte (all the MCCS 0xD0.. range) was unreachable. */
    payload[0] = DFW_DDC_GET_VCP;
    payload[1] = (uint8_t)(code >> 8);
    payload[2] = (uint8_t)(code & 0xFFu);
    rc = dfw_ddc_host_transact(bus, payload, sizeof payload,
                               reply, sizeof reply, &reply_len);
    if (rc != DFW_DDC_OK) return rc;

    rc = ddc_parse_get_reply(reply, reply_len, code, out);
    if (rc != DFW_DDC_OK) return rc;
    if (out->result_code == DFW_DDC_OP_UNSUPPORTED)
        bus->stats.unsupported++;
    return DFW_DDC_OK;
}

int dfw_ddc_set_vcp(dfw_ddc_bus *bus, uint16_t code, uint16_t value)
{
    uint8_t payload[4];
    uint8_t reply[64];
    size_t reply_len = 0;

    if (!bus) return DFW_DDC_E_NULL;
    /* Set VCP Feature: 03 <vcp_hi> <vcp_lo> <value>, length byte 7, so the
     * frame is 9 bytes.  Continuous controls above 0xFF carry the value's
     * high byte in the message-type slot; this host only drives controls
     * whose maximum fits in one byte (brightness, contrast, input, power),
     * so a 16-bit set is deliberately not supported here. */
    payload[0] = DFW_DDC_SET_VCP;
    payload[1] = (uint8_t)(code >> 8);
    payload[2] = (uint8_t)(code & 0xFFu);
    payload[3] = (uint8_t)(value & 0xFFu);
    return dfw_ddc_host_transact(bus, payload, sizeof payload,
                                 reply, sizeof reply, &reply_len);
}

int dfw_ddc_save_settings(dfw_ddc_bus *bus)
{
    uint8_t payload[2];
    uint8_t reply[64];
    size_t reply_len = 0;
    int rc;

    if (!bus) return DFW_DDC_E_NULL;
    payload[0] = DFW_DDC_SAVE_SETTINGS;
    payload[1] = 0x00;
    rc = dfw_ddc_host_transact(bus, payload, sizeof payload,
                               reply, sizeof reply, &reply_len);
    if (rc == DFW_DDC_OK) bus->stats.saves++;
    return rc;
}
