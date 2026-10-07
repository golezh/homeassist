//***************************************************************************
// Serial protocol ATmega <-> ESP8266. See include/proto.h, docs/protocol.md
//
// Safety rules:
//  - The ESP can only switch "user" relays (index >= PROTO_FIRST_USER_RELAY).
//    The valve motor relays are controlled by valve_ctrl only.
//  - Every user relay may get a lease: if the ESP does not repeat the
//    command in time, the relay is switched off.
//  - If no valid frame arrives for ESP_LINK_TIMEOUT_MS, the link is lost
//    and all user relays are switched off.
//  - Leak protection does not depend on the ESP in any way.
//***************************************************************************
#include "proto.h"
#include "relay.h"
#include "UART_routines.h"

#include <string.h>

#ifdef __AVR__
#include <avr/pgmspace.h>
#else
#define pgm_read_byte(p) (*(const uint8_t *)(p))
#define strcmp_P(a, b)   strcmp((a), (b))
#endif

#define MAX_RELAYS 8U
#define MAX_FIELDS 5U

/* ---- RX line assembly ---- */
static char    line[PROTO_LINE_MAX + 1];
static uint8_t line_len;
static bool    line_active;
static bool    line_overflow;

/* ---- link ---- */
static uint32_t last_rx_ms;
static bool     link_ok;

/* ---- events for valve_ctrl ---- */
static bool ev_open;
static bool ev_close;

/* ---- user relays ---- */
#define USER_RELAYS (MAX_RELAYS - PROTO_FIRST_USER_RELAY)
static uint16_t lease_left_s[USER_RELAYS];   // 0 = no lease
static uint32_t lease_sec_ms;                // 1 s time base for leases

/* ---- status reporting ---- */
static proto_status_t last_st;
static uint8_t  last_relays;
static bool     last_link;
static bool     status_valid;
static uint32_t last_status_ms;

/* ======================================================================= */
/*  Logging                                                                */
/* ======================================================================= */

void log_P(const char *msg_P, const char *arg_P)
{
    transmitString_F(PSTR("# "));
    transmitString_F(msg_P);
    if (arg_P)
    {
        transmitString_F(arg_P);
    }
    transmitString_F(PSTR("\r\n"));
}

void log_c(const char *msg_P, char c, const char *tail_P)
{
    transmitString_F(PSTR("# "));
    transmitString_F(msg_P);
    transmitByte((uint8_t)c);
    if (tail_P)
    {
        transmitString_F(tail_P);
    }
    transmitString_F(PSTR("\r\n"));
}

/* ======================================================================= */
/*  TX frame helpers                                                       */
/* ======================================================================= */

static uint8_t tx_cs;

static char hex_digit(uint8_t v)
{
    v &= 0x0F;
    return (char)(v < 10 ? ('0' + v) : ('A' + v - 10));
}

static void tx_c(char c)
{
    transmitByte((uint8_t)c);
    tx_cs ^= (uint8_t)c;
}

static void tx_P(const char *p)
{
    char c;
    while ((c = (char)pgm_read_byte(p++)) != '\0')
    {
        tx_c(c);
    }
}

static void tx_s(const char *s)
{
    while (*s)
    {
        tx_c(*s++);
    }
}

static void tx_hex2(uint8_t v)
{
    tx_c(hex_digit((uint8_t)(v >> 4)));
    tx_c(hex_digit(v));
}

static void frame_begin_P(const char *type_P)
{
    transmitByte('$');
    tx_cs = 0;
    tx_P(type_P);
}

static void frame_end(void)
{
    uint8_t cs = tx_cs;
    transmitByte('*');
    transmitByte((uint8_t)hex_digit((uint8_t)(cs >> 4)));
    transmitByte((uint8_t)hex_digit(cs));
    transmitByte('\r');
    transmitByte('\n');
}

static void reply_ok(const char *cmd)
{
    frame_begin_P(PSTR("OK,"));
    tx_s(cmd);
    frame_end();
}

static void reply_err_P(const char *cmd, const char *reason_P)
{
    frame_begin_P(PSTR("ER,"));
    tx_s(cmd);
    tx_c(',');
    tx_P(reason_P);
    frame_end();
}

/* ======================================================================= */
/*  Status                                                                 */
/* ======================================================================= */

static const char *pos_name_P(valve_pos_t p)
{
    switch (p)
    {
        case POS_OPEN:   return PSTR("OPEN");
        case POS_CLOSED: return PSTR("CLOSED");
        case POS_FAULT:  return PSTR("FAULT");
        default:         return PSTR("MID");
    }
}

static const char *fault_name_P(fault_t f)
{
    switch (f)
    {
        case FAULT_OPEN_TIMEOUT:  return PSTR("OTMO");
        case FAULT_CLOSE_TIMEOUT: return PSTR("CTMO");
        case FAULT_SWITCHES:      return PSTR("SW");
        default:                  return PSTR("NONE");
    }
}

static void send_status(uint32_t now, const proto_status_t *st)
{
    uint8_t relays = relay_mask();

    frame_begin_P(PSTR("ST,"));
    tx_P(ctrl_state_name_P(st->state));
    tx_c(',');
    tx_P(pos_name_P(st->pos));
    tx_c(',');
    tx_hex2(st->leak_mask);
    tx_c(',');
    tx_P(fault_name_P(st->fault));
    tx_c(',');
    tx_c(st->alarm ? '1' : '0');
    tx_c(',');
    tx_hex2(relays);
    tx_c(',');
    tx_c(link_ok ? '1' : '0');
    frame_end();

    last_st        = *st;
    last_relays    = relays;
    last_link      = link_ok;
    status_valid   = true;
    last_status_ms = now;
}

static bool status_changed(const proto_status_t *st)
{
    return !status_valid
        || st->state     != last_st.state
        || st->pos       != last_st.pos
        || st->leak_mask != last_st.leak_mask
        || st->fault     != last_st.fault
        || st->alarm     != last_st.alarm
        || relay_mask()  != last_relays
        || link_ok       != last_link;
}

void proto_send_hello(void)
{
    frame_begin_P(PSTR("HELLO,LEAKDET," FW_VERSION));
    frame_end();
}

/* ======================================================================= */
/*  User relays                                                            */
/* ======================================================================= */

static void user_relay_set(uint8_t n, bool on, uint16_t lease_s, uint32_t now)
{
    (void)now;

    if (on)
    {
        relay_on(n);
        lease_left_s[n - PROTO_FIRST_USER_RELAY] = lease_s;
    }
    else
    {
        relay_off(n);
        lease_left_s[n - PROTO_FIRST_USER_RELAY] = 0;
    }
}

static void user_relays_all_off(void)
{
    uint8_t n;

    for (n = PROTO_FIRST_USER_RELAY; n < relay_count() && n < MAX_RELAYS; n++)
    {
        relay_off(n);
    }
    memset(lease_left_s, 0, sizeof(lease_left_s));
}

/* ======================================================================= */
/*  RX parsing                                                             */
/* ======================================================================= */

static int8_t hex_val(char c)
{
    if (c >= '0' && c <= '9') return (int8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (int8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (int8_t)(c - 'a' + 10);
    return -1;
}

/* Parses a decimal number 0..max. Returns false on any error. */
static bool parse_u16(const char *s, uint16_t max, uint16_t *out)
{
    uint32_t v = 0;

    if (s == NULL || *s == '\0')
    {
        return false;
    }
    while (*s)
    {
        if (*s < '0' || *s > '9')
        {
            return false;
        }
        v = v * 10U + (uint32_t)(*s - '0');
        if (v > max)
        {
            return false;
        }
        s++;
    }
    *out = (uint16_t)v;
    return true;
}

static void cmd_relay(char **f, uint8_t nf, uint32_t now)
{
    uint16_t n, v, lease = 0;

    if (nf < 3 || nf > 4
        || !parse_u16(f[1], 255, &n)
        || !parse_u16(f[2], 1, &v)
        || (nf == 4 && !parse_u16(f[3], PROTO_MAX_LEASE_S, &lease)))
    {
        reply_err_P(f[0], PSTR("ARG"));
        return;
    }

    if (n >= relay_count() || n >= MAX_RELAYS)
    {
        reply_err_P(f[0], PSTR("ARG"));
        return;
    }

    if (n < PROTO_FIRST_USER_RELAY)
    {
        reply_err_P(f[0], PSTR("DENIED"));   // valve motor relay
        return;
    }

    user_relay_set((uint8_t)n, v != 0, lease, now);
    reply_ok(f[0]);
}

static void print_help(void)
{
    log_P(PSTR("Commands: $GS $HB $VER $OPEN $CLOSE $?"), NULL);
    log_P(PSTR("$RL,<n>,<0|1>[,<lease_s>]  n = 2..6"), NULL);
}

static void handle_line(uint32_t now, const proto_status_t *st)
{
    char   *f[MAX_FIELDS];
    uint8_t nf = 0;
    char   *p;
    char   *star = strchr(line, '*');
    char   *cmd;

    /* ---- checksum ---- */
    if (star)
    {
        uint8_t cs = 0;
        int8_t  h, l;

        for (p = line; p < star; p++)
        {
            cs ^= (uint8_t)*p;
        }
        h = hex_val(star[1]);
        l = (h >= 0) ? hex_val(star[2]) : -1;
        if (h < 0 || l < 0 || star[3] != '\0' || (uint8_t)((h << 4) | l) != cs)
        {
            reply_err_P("?", PSTR("CS"));
            return;
        }
        *star = '\0';
    }
    else if (!PROTO_ALLOW_NO_CHECKSUM)
    {
        reply_err_P("?", PSTR("CS"));
        return;
    }

    /* ---- upper case, so commands can be typed in any case ---- */
    for (p = line; *p; p++)
    {
        if (*p >= 'a' && *p <= 'z')
        {
            *p = (char)(*p - 'a' + 'A');
        }
    }

    /* ---- split fields ---- */
    p = line;
    f[nf++] = p;
    while (*p)
    {
        if (*p == ',')
        {
            *p = '\0';
            if (nf >= MAX_FIELDS)
            {
                reply_err_P(f[0], PSTR("ARG"));
                return;
            }
            f[nf++] = p + 1;
        }
        p++;
    }
    cmd = f[0];

    if (*cmd == '\0')
    {
        return;
    }

    /* ---- any valid frame is a heartbeat ---- */
    last_rx_ms = now;
    if (!link_ok)
    {
        link_ok = true;
        log_P(PSTR("ESP link up"), NULL);
    }

    /* ---- commands ---- */
    if (strcmp_P(cmd, PSTR("HB")) == 0)
    {
        /* heartbeat, no reply */
    }
    else if (strcmp_P(cmd, PSTR("GS")) == 0)
    {
        send_status(now, st);
    }
    else if (strcmp_P(cmd, PSTR("VER")) == 0)
    {
        proto_send_hello();
    }
    else if (strcmp_P(cmd, PSTR("?")) == 0)
    {
        print_help();
    }
    else if (strcmp_P(cmd, PSTR("CLOSE")) == 0)
    {
        ev_close = true;
        reply_ok(cmd);
    }
    else if (strcmp_P(cmd, PSTR("OPEN")) == 0)
    {
        if (!PROTO_ALLOW_REMOTE_OPEN && st->alarm)
        {
            reply_err_P(cmd, PSTR("DENIED"));
        }
        else if (st->leak_mask != 0)
        {
            reply_err_P(cmd, PSTR("WET"));
        }
        else
        {
            ev_open = true;
            reply_ok(cmd);
        }
    }
    else if (strcmp_P(cmd, PSTR("RL")) == 0)
    {
        cmd_relay(f, nf, now);
    }
    else
    {
        reply_err_P(cmd, PSTR("UNK"));
    }
}

static void feed(char c, uint32_t now, const proto_status_t *st)
{
    if (c == '$')
    {
        /* start of frame, also resynchronizes after garbage */
        line_active = true;
        line_overflow = false;
        line_len = 0;
        return;
    }

    if (!line_active)
    {
        return;   // noise, echo, '#' log lines, etc.
    }

    if (c == '\r' || c == '\n')
    {
        line_active = false;
        if (line_overflow)
        {
            reply_err_P("?", PSTR("LEN"));
            return;
        }
        line[line_len] = '\0';
        handle_line(now, st);
        return;
    }

    if ((uint8_t)c < 0x20 || (uint8_t)c > 0x7E)
    {
        line_active = false;      // binary garbage: drop the frame
        reply_err_P("?", PSTR("FMT"));
        return;
    }

    if (line_len < PROTO_LINE_MAX)
    {
        line[line_len++] = c;
    }
    else
    {
        line_overflow = true;
    }
}

/* ======================================================================= */
/*  Public API                                                             */
/* ======================================================================= */

void proto_init(void)
{
    line_len = 0;
    line_active = false;
    line_overflow = false;
    link_ok = false;
    last_rx_ms = 0;
    ev_open = false;
    ev_close = false;
    memset(lease_left_s, 0, sizeof(lease_left_s));
    lease_sec_ms = 0;
    status_valid = false;
    last_status_ms = 0;
}

void proto_poll(uint32_t now, const proto_status_t *st)
{
    int16_t c;

    while ((c = uart_getc()) >= 0)
    {
        feed((char)c, now, st);
    }
}

void proto_tick(uint32_t now, const proto_status_t *st)
{
    uint8_t n;
    uint8_t lost;

    /* ---- link watchdog ---- */
    if (link_ok && (uint32_t)(now - last_rx_ms) >= ESP_LINK_TIMEOUT_MS)
    {
        link_ok = false;
        log_P(PSTR("ESP link LOST, user relays off"), NULL);
        user_relays_all_off();
    }

    /* ---- relay leases, 1 s resolution ---- */
    if ((uint32_t)(now - lease_sec_ms) >= 1000UL)
    {
        lease_sec_ms = now;
        for (n = 0; n < USER_RELAYS; n++)
        {
            if (lease_left_s[n] != 0 && --lease_left_s[n] == 0)
            {
                uint8_t r = (uint8_t)(n + PROTO_FIRST_USER_RELAY);
                log_c(PSTR("Lease expired, relay "), (char)('0' + r), PSTR(" off"));
                user_relay_set(r, false, 0, now);
            }
        }
    }

    /* ---- RX overflow ---- */
    lost = uart_rx_overflow_take();
    if (lost)
    {
        log_P(PSTR("UART RX overflow"), NULL);
    }

    /* ---- status report ---- */
    if (status_changed(st) || (uint32_t)(now - last_status_ms) >= PROTO_STATUS_PERIOD_MS)
    {
        send_status(now, st);
    }
}

bool proto_take_open(void)
{
    bool e = ev_open;
    ev_open = false;
    return e;
}

bool proto_take_close(void)
{
    bool e = ev_close;
    ev_close = false;
    return e;
}

bool proto_link_ok(void)
{
    return link_ok;
}
