//***************************************************************************
// Valve control state machine (hardware independent)
//
// The module knows nothing about pins, relays, UART or LCD. It receives
// already debounced inputs once per logic tick and drives the hardware
// through three hooks implemented in main.c:
//     hal_motor()       - start/stop the valve motor (safe switching)
//     hal_alarm_store() - persist the alarm latch (EEPROM)
//     hal_log_P()       - debug log, string in flash
// Thanks to this the logic can be unit-tested on a PC.
//***************************************************************************
#ifndef VALVE_CTRL_H
#define VALVE_CTRL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __AVR__
#include <avr/pgmspace.h>
#else
#define PSTR(s) (s)
#endif

/* Max motor run time from one end position to the other */
#define VALVE_TIMEOUT_MS   10000UL

/* Logic ticks to wait in INIT so that debouncing settles */
#define VALVE_INIT_TICKS   5U

/* Valve position as reported by the limit switches */
typedef enum
{
    POS_UNKNOWN = 0,   // no switch active: moving, stopped midway or wire broken
    POS_OPEN,
    POS_CLOSED,
    POS_FAULT          // both switches active at the same time
} valve_pos_t;

typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_OPEN,
    MOTOR_CLOSE
} motor_dir_t;

typedef enum
{
    ST_INIT = 0,
    ST_OPENING,
    ST_OPEN,        // normal operation, water on
    ST_CLOSING,
    ST_CLOSED,      // closed manually by the CLOSE button, no leak
    ST_ALARM,       // leak was detected, valve closed, waiting for RESET
    ST_FAULT,       // valve did not reach end position / switch fault
    ST_COUNT
} sys_state_t;

typedef enum
{
    FAULT_NONE = 0,
    FAULT_OPEN_TIMEOUT,
    FAULT_CLOSE_TIMEOUT,
    FAULT_SWITCHES
} fault_t;

typedef struct
{
    uint8_t     leak_mask;   // bit N = debounced leak on sensor N
    valve_pos_t pos;         // debounced valve position
    bool        btn_reset;   // RESET / OPEN button click (event)
    bool        btn_close;   // CLOSE button click (event)
    uint32_t    now_ms;
} ctrl_inputs_t;

void        ctrl_init(bool alarm_latched);
void        ctrl_step(const ctrl_inputs_t *in);

sys_state_t ctrl_state(void);
fault_t     ctrl_fault(void);
bool        ctrl_alarm(void);
const char *ctrl_state_name_P(sys_state_t st);   // name in flash
motor_dir_t ctrl_motor(void);

/* Hooks, implemented by the application */
void hal_motor(motor_dir_t dir);
void hal_alarm_store(bool latched);
void hal_log_P(const char *msg_P, const char *arg_P);   // one log line, arg may be NULL

#endif // VALVE_CTRL_H
