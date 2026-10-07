//***************************************************************************
// Valve control state machine
//
// Safety rules implemented here:
//  1. A confirmed leak latches the alarm. The latch is persisted (EEPROM)
//     and survives power loss. Only the RESET button clears it, and only
//     when all sensors are dry. The valve is never reopened automatically.
//  2. When the alarm is latched the valve is driven to CLOSED from any
//     state, including reversing a valve that is currently opening.
//  3. Position is taken from the limit switches only. "Unknown" (between
//     switches) is never treated as "closed".
//  4. Every motor run is limited by VALVE_TIMEOUT_MS. On timeout the motor
//     is stopped and the system goes to FAULT. In FAULT one more closing
//     attempt is made if a leak is present.
//***************************************************************************
#include "valve_ctrl.h"

static sys_state_t s_state;
static fault_t     s_fault;
static bool        s_alarm;
static motor_dir_t s_motor;
static uint32_t    s_motion_start;
static uint8_t     s_init_ticks;
static bool        s_fault_close_tried;

static const char *state_name_P(sys_state_t st)
{
    switch (st)
    {
        case ST_INIT:    return PSTR("INIT");
        case ST_OPENING: return PSTR("OPENING");
        case ST_OPEN:    return PSTR("OPEN");
        case ST_CLOSING: return PSTR("CLOSING");
        case ST_CLOSED:  return PSTR("CLOSED");
        case ST_ALARM:   return PSTR("ALARM");
        case ST_FAULT:   return PSTR("FAULT");
        default:         return PSTR("?");
    }
}

static void set_state(sys_state_t st)
{
    if (st == s_state)
    {
        return;
    }
    s_state = st;
    hal_log_P(PSTR("\n\rState -> "));
    hal_log_P(state_name_P(st));
}

static void motor(motor_dir_t dir, uint32_t now)
{
    if (dir == s_motor)
    {
        return;   // keep running, do not restart the timeout
    }
    s_motor = dir;
    s_motion_start = now;
    hal_motor(dir);
}

static void start_open(uint32_t now)
{
    motor(MOTOR_OPEN, now);
    set_state(ST_OPENING);
}

static void start_close(uint32_t now)
{
    motor(MOTOR_CLOSE, now);
    set_state(ST_CLOSING);
}

static void enter_fault(fault_t f, uint32_t now)
{
    motor(MOTOR_STOP, now);
    s_fault = f;

    switch (f)
    {
        case FAULT_OPEN_TIMEOUT:  hal_log_P(PSTR("\n\rFAULT: valve opening timeout")); break;
        case FAULT_CLOSE_TIMEOUT: hal_log_P(PSTR("\n\rFAULT: valve closing timeout")); break;
        case FAULT_SWITCHES:      hal_log_P(PSTR("\n\rFAULT: both limit switches active")); break;
        default: break;
    }
    set_state(ST_FAULT);
}

static bool motion_timed_out(uint32_t now)
{
    return (uint32_t)(now - s_motion_start) >= VALVE_TIMEOUT_MS;
}

void ctrl_init(bool alarm_latched)
{
    s_state = ST_INIT;
    s_fault = FAULT_NONE;
    s_alarm = alarm_latched;
    s_motor = MOTOR_STOP;
    s_motion_start = 0;
    s_init_ticks = 0;
    s_fault_close_tried = false;

    if (alarm_latched)
    {
        hal_log_P(PSTR("\n\rAlarm latch restored from EEPROM"));
    }
}

void ctrl_step(const ctrl_inputs_t *in)
{
    const uint32_t now = in->now_ms;
    const bool leak = (in->leak_mask != 0);

    /* Rule 1: latch the alarm */
    if (leak && !s_alarm)
    {
        s_alarm = true;
        hal_alarm_store(true);
        hal_log_P(PSTR("\n\rLEAK DETECTED! Alarm latched"));
    }

    switch (s_state)
    {
        case ST_INIT:
            if (++s_init_ticks < VALVE_INIT_TICKS)
            {
                break;   // let debouncing settle
            }
            if (in->pos == POS_FAULT)
            {
                enter_fault(FAULT_SWITCHES, now);
            }
            else if (s_alarm)
            {
                if (in->pos == POS_CLOSED)
                {
                    set_state(ST_ALARM);
                }
                else
                {
                    start_close(now);
                }
            }
            else if (in->pos == POS_OPEN)
            {
                set_state(ST_OPEN);
            }
            else
            {
                start_open(now);
            }
            break;

        case ST_OPEN:
            if (s_alarm || in->btn_close)
            {
                start_close(now);
            }
            else if (in->pos == POS_FAULT)
            {
                enter_fault(FAULT_SWITCHES, now);
            }
            else if (in->pos == POS_CLOSED)
            {
                hal_log_P(PSTR("\n\rValve was closed externally"));
                set_state(ST_CLOSED);
            }
            break;

        case ST_OPENING:
            if (s_alarm || in->btn_close)
            {
                /* Rule 2: reverse. hal_motor() stops first and waits. */
                start_close(now);
            }
            else if (in->pos == POS_OPEN)
            {
                motor(MOTOR_STOP, now);
                set_state(ST_OPEN);
            }
            else if (in->pos == POS_FAULT)
            {
                enter_fault(FAULT_SWITCHES, now);
            }
            else if (motion_timed_out(now))
            {
                enter_fault(FAULT_OPEN_TIMEOUT, now);
            }
            break;

        case ST_CLOSING:
            if (in->pos == POS_CLOSED)
            {
                motor(MOTOR_STOP, now);
                set_state(s_alarm ? ST_ALARM : ST_CLOSED);
            }
            else if (!s_alarm && in->btn_reset)
            {
                start_open(now);   // manual close cancelled by user
            }
            else if (motion_timed_out(now))
            {
                /* POS_FAULT is ignored while closing: keep closing
                   until the timeout, closing is the safe direction */
                enter_fault(FAULT_CLOSE_TIMEOUT, now);
            }
            break;

        case ST_CLOSED:
            if (s_alarm)
            {
                set_state(ST_ALARM);
            }
            else if (in->btn_reset)
            {
                start_open(now);
            }
            break;

        case ST_ALARM:
            if (in->pos == POS_OPEN || in->pos == POS_UNKNOWN)
            {
                /* valve left the closed position (opened by hand?) */
                hal_log_P(PSTR("\n\rValve not closed in ALARM, closing again"));
                start_close(now);
            }
            else if (in->btn_reset)
            {
                if (leak)
                {
                    hal_log_P(PSTR("\n\rReset denied: sensor still wet"));
                }
                else
                {
                    s_alarm = false;
                    hal_alarm_store(false);
                    hal_log_P(PSTR("\n\rAlarm reset by user"));
                    start_open(now);
                }
            }
            break;

        case ST_FAULT:
            if (s_alarm && !s_fault_close_tried && in->pos != POS_CLOSED)
            {
                /* Rule 4: one more attempt to close on leak */
                s_fault_close_tried = true;
                hal_log_P(PSTR("\n\rFAULT + leak: retry closing"));
                start_close(now);
            }
            else if (s_alarm && in->pos == POS_CLOSED)
            {
                /* valve is closed after all */
                set_state(ST_ALARM);
            }
            else if (in->btn_reset)
            {
                hal_log_P(PSTR("\n\rFault reset by user"));
                s_fault = FAULT_NONE;
                s_fault_close_tried = false;
                s_init_ticks = 0;
                set_state(ST_INIT);
            }
            break;

        default:
            enter_fault(FAULT_SWITCHES, now);
            break;
    }

    if (s_state != ST_FAULT)
    {
        s_fault = FAULT_NONE;
    }
}

sys_state_t ctrl_state(void) { return s_state; }
fault_t     ctrl_fault(void) { return s_fault; }
bool        ctrl_alarm(void) { return s_alarm; }
motor_dir_t ctrl_motor(void) { return s_motor; }
