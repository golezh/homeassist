//***************************************************************************
// Serial protocol between the leak controller (ATmega) and the ESP8266.
// Full description: docs/protocol.md
//
// Frame:  $<CMD>[,<field>...][*HH]\r\n
//         HH = XOR of all chars between '$' and '*', two hex digits.
// Lines starting with '#' are debug logs, the ESP ignores them.
//***************************************************************************
#ifndef PROTO_H
#define PROTO_H

#include <stdint.h>
#include <stdbool.h>
#include "valve_ctrl.h"

/* 1 = accept frames without "*HH", so commands can be typed by hand in a
   terminal. Set to 0 once the ESP is connected. */
#define PROTO_ALLOW_NO_CHECKSUM  1

/* 1 = $OPEN may reset a latched leak alarm remotely (only if all sensors
   are dry). 0 = alarm can be reset by the physical button only. */
#define PROTO_ALLOW_REMOTE_OPEN  1

#define ESP_LINK_TIMEOUT_MS      10000UL  // no valid frame -> link lost
#define PROTO_STATUS_PERIOD_MS   2000UL   // $ST at least this often
#define PROTO_FIRST_USER_RELAY   2U       // relays 0,1 drive the valve motor
#define PROTO_MAX_LEASE_S        3600U
#define PROTO_LINE_MAX           40U

#define FW_VERSION "0.3.0"

typedef struct
{
    sys_state_t state;
    valve_pos_t pos;
    uint8_t     leak_mask;
    fault_t     fault;
    bool        alarm;
} proto_status_t;

void proto_init(void);
void proto_send_hello(void);

/* Call on every main loop pass: reads and executes received commands */
void proto_poll(uint32_t now, const proto_status_t *st);

/* Call on every logic tick: link timeout, relay leases, status reports */
void proto_tick(uint32_t now, const proto_status_t *st);

bool proto_take_open(void);    // remote OPEN / RESET event
bool proto_take_close(void);   // remote CLOSE event
bool proto_link_ok(void);

/* Debug log line: "# <msg><arg>\r\n". Both strings in flash, arg may be NULL */
void log_P(const char *msg_P, const char *arg_P);
/* "# <msg><c><tail>\r\n" */
void log_c(const char *msg_P, char c, const char *tail_P);

#endif // PROTO_H
