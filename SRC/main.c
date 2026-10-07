//***************************************************************************
//  File........: main.c
//  Project.....: Leak Detector
//  Target(s)...: ATmega8535 @ 4 MHz (F_CPU comes from the Makefile)
//  Compiler....: avr-gcc
//
//  main.c owns the hardware: it reads and debounces the inputs, feeds them
//  to the state machine (valve_ctrl.c), drives the relays and the LCD.
//
//  Recommended fuses: enable Brown-out detection (BODEN, 2.7 V) so that
//  the EEPROM alarm latch cannot be corrupted at low supply voltage.
//***************************************************************************
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "UART_routines.h"
#include "PWM.h"
#include "key.h"
#include "relay.h"
#include "timer.h"
#include "LCD_screen.h"
#include "valve_ctrl.h"
#include "proto.h"

/* 1 = 20x4 I2C LCD is connected to this controller.
   0 = no LCD (the display moved to the ESP): frees ~2 KB of flash. */
#ifndef USE_LCD
#define USE_LCD 1
#endif

/* ---------------- Timing ---------------- */
#define LOGIC_PERIOD_MS       200U     // sensors + state machine
#define BUTTON_PERIOD_MS      20U      // button sampling
#define MOTOR_DEADTIME_MS     100U     // pause between relay switching
#define LCD_RETRY_MS          30000UL  // re-init the LCD after a failure

/* ---------------- Debounce (in logic ticks of 200 ms) ---------------- */
#define LEAK_CONFIRM_TICKS    3U       // wet 0.6 s   -> leak
#define LEAK_RELEASE_TICKS    10U      // dry 2.0 s   -> sensor dry
#define SWITCH_CONFIRM_TICKS  2U       // stable 0.4 s -> position valid
#define BUTTON_CONFIRM_SAMPLES 3U      // pressed 60 ms -> click

#define LCD_BRIGHTNESS        250U     // 0..1023

/* ---------------- Pins ---------------- */
/* Leak sensors: PORTC, active LOW. tag is shown on the LCD/UART. */
typedef struct
{
    uint8_t pin;
    char    tag;
} leak_sensor_cfg_t;

static const leak_sensor_cfg_t leak_cfg[] =
{
    { PC2, '1' },   // Kitchen + Bathroom 1 (merged)
    { PC3, '2' },   // Bathroom 2
    { PC4, '3' },   // spare
};
#define LEAK_COUNT ((uint8_t)(sizeof(leak_cfg) / sizeof(leak_cfg[0])))

/* Valve limit switches: PORTC, active LOW */
#define VALVE_SW_OPEN    PC5
#define VALVE_SW_CLOSED  PC6

/* Buttons: PORTD, active LOW */
#define BTN_RESET        PD6   // reset alarm / open valve
#define BTN_CLOSE        PD7   // close valve manually

/* Relays (indexes in relay.c) */
#define RELAY_VALVE_CLOSE 0
#define RELAY_VALVE_OPEN  1

/* EEPROM: alarm latch */
#define EE_ALARM_ADDR     ((uint8_t *)0)
#define EE_ALARM_MAGIC    0xA5

/* ======================================================================= */
/*  Hooks for valve_ctrl                                                   */
/* ======================================================================= */

void hal_log_P(const char *msg_P, const char *arg_P)
{
    log_P(msg_P, arg_P);
}

void hal_alarm_store(bool latched)
{
    eeprom_update_byte(EE_ALARM_ADDR, latched ? EE_ALARM_MAGIC : 0x00);
}

/*
 * Never energize both relays: always switch both off, wait for the
 * contacts to open, then switch on the requested direction.
 */
void hal_motor(motor_dir_t dir)
{
    relay_off(RELAY_VALVE_OPEN);
    relay_off(RELAY_VALVE_CLOSE);

    if (dir == MOTOR_STOP)
    {
        log_P(PSTR("Motor: stop"), NULL);
        return;
    }

    _delay_ms(MOTOR_DEADTIME_MS);

    if (dir == MOTOR_OPEN)
    {
        relay_on(RELAY_VALVE_OPEN);
        log_P(PSTR("Motor: opening"), NULL);
    }
    else
    {
        relay_on(RELAY_VALVE_CLOSE);
        log_P(PSTR("Motor: closing"), NULL);
    }
}

static bool alarm_load(void)
{
    return eeprom_read_byte(EE_ALARM_ADDR) == EE_ALARM_MAGIC;
}

/* ======================================================================= */
/*  Inputs                                                                 */
/* ======================================================================= */

static uint8_t leak_mask;                  // debounced, bit N = sensor N wet
static uint8_t leak_wet_cnt[LEAK_COUNT];
static uint8_t leak_dry_cnt[LEAK_COUNT];

static void leak_update(void)
{
    uint8_t pins = PINC;
    uint8_t i;

    for (i = 0; i < LEAK_COUNT; i++)
    {
        uint8_t bit = (uint8_t)(1U << i);
        bool wet = (pins & (1U << leak_cfg[i].pin)) == 0;

        if (wet)
        {
            leak_dry_cnt[i] = 0;
            if (leak_wet_cnt[i] < LEAK_CONFIRM_TICKS)
            {
                leak_wet_cnt[i]++;
            }
            if (leak_wet_cnt[i] >= LEAK_CONFIRM_TICKS && !(leak_mask & bit))
            {
                leak_mask |= bit;
                log_c(PSTR("Sensor "), leak_cfg[i].tag, PSTR(": WET"));
            }
        }
        else
        {
            leak_wet_cnt[i] = 0;
            if (leak_dry_cnt[i] < LEAK_RELEASE_TICKS)
            {
                leak_dry_cnt[i]++;
            }
            if (leak_dry_cnt[i] >= LEAK_RELEASE_TICKS && (leak_mask & bit))
            {
                leak_mask &= (uint8_t)~bit;
                log_c(PSTR("Sensor "), leak_cfg[i].tag, PSTR(": dry"));
            }
        }
    }
}

static valve_pos_t valve_pos = POS_UNKNOWN;   // debounced
static valve_pos_t valve_pos_candidate = POS_UNKNOWN;
static uint8_t     valve_pos_cnt;

static valve_pos_t valve_pos_raw(void)
{
    uint8_t pins = PINC;
    bool is_open   = (pins & (1U << VALVE_SW_OPEN))   == 0;
    bool is_closed = (pins & (1U << VALVE_SW_CLOSED)) == 0;

    if (is_open && is_closed) return POS_FAULT;
    if (is_open)              return POS_OPEN;
    if (is_closed)            return POS_CLOSED;
    return POS_UNKNOWN;
}

static void valve_pos_update(void)
{
    valve_pos_t p = valve_pos_raw();

    if (p == valve_pos_candidate)
    {
        if (valve_pos_cnt < SWITCH_CONFIRM_TICKS)
        {
            valve_pos_cnt++;
        }
    }
    else
    {
        valve_pos_candidate = p;
        valve_pos_cnt = 1;
    }

    if (valve_pos_cnt >= SWITCH_CONFIRM_TICKS && valve_pos != valve_pos_candidate)
    {
        valve_pos = valve_pos_candidate;
        switch (valve_pos)
        {
            case POS_OPEN:   log_P(PSTR("Valve position: OPEN"), NULL);   break;
            case POS_CLOSED: log_P(PSTR("Valve position: CLOSED"), NULL); break;
            case POS_FAULT:  log_P(PSTR("Valve position: both limit switches active!"), NULL); break;
            default:         log_P(PSTR("Valve position: between end positions"), NULL); break;
        }
    }
}

/* Non-blocking button: returns true once per press */
typedef struct
{
    uint8_t pin;
    uint8_t cnt;
    bool    pressed;
    bool    event;
} button_t;

static button_t btn_reset = { BTN_RESET, 0, false, false };
static button_t btn_close = { BTN_CLOSE, 0, false, false };

static void button_sample(button_t *b)
{
    bool down = (PIND & (1U << b->pin)) == 0;

    if (down)
    {
        if (b->cnt < BUTTON_CONFIRM_SAMPLES)
        {
            b->cnt++;
        }
        if (b->cnt >= BUTTON_CONFIRM_SAMPLES && !b->pressed)
        {
            b->pressed = true;
            b->event = true;   // event on press
        }
    }
    else
    {
        if (b->cnt > 0)
        {
            b->cnt--;
        }
        if (b->cnt == 0)
        {
            b->pressed = false;
        }
    }
}

static bool button_take(button_t *b)
{
    bool e = b->event;
    b->event = false;
    return e;
}

#if USE_LCD
/* ======================================================================= */
/*  LCD                                                                    */
/* ======================================================================= */

/*
 * RAM is scarce (512 B): instead of keeping a copy of every row we keep a
 * 16-bit checksum per row, and one shared line buffer.
 */
static uint16_t lcd_row_sum[LCD_ROWS];
static char     lcd_buf[LCD_COLS + 1];
static uint32_t lcd_last_try;

static void lcd_cache_invalidate(void)
{
    memset(lcd_row_sum, 0xFF, sizeof(lcd_row_sum));
}

/* Pads lcd_buf to 20 chars and draws it, only if the row changed */
static void lcd_commit(uint8_t row)
{
    uint8_t  i = (uint8_t)strnlen(lcd_buf, LCD_COLS);
    uint16_t sum = 0;

    while (i < LCD_COLS)
    {
        lcd_buf[i++] = ' ';
    }
    lcd_buf[LCD_COLS] = '\0';

    for (i = 0; i < LCD_COLS; i++)
    {
        sum = (uint16_t)((sum << 1) | (sum >> 15)) ^ (uint8_t)lcd_buf[i];
    }

    if (!lcd_is_ok() || sum == lcd_row_sum[row])
    {
        return;
    }

    lcd_goto(row, 0);
    lcd_print(lcd_buf);

    if (lcd_is_ok())
    {
        lcd_row_sum[row] = sum;
    }
}

static void lcd_line_P(uint8_t row, const char *text_P)
{
    strncpy_P(lcd_buf, text_P, LCD_COLS);
    lcd_buf[LCD_COLS] = '\0';
    lcd_commit(row);
}

static void lcd_start(void)
{
    lcd_last_try = millis();
    lcd_cache_invalidate();

    if (lcd_init())
    {
        log_P(PSTR("LCD: ok"), NULL);
    }
    else
    {
        log_P(PSTR("LCD: not responding"), NULL);
    }
}

static void lcd_update(void)
{
    uint8_t i, n;

    if (!lcd_is_ok())
    {
        if ((uint32_t)(millis() - lcd_last_try) >= LCD_RETRY_MS)
        {
            lcd_start();
        }
        if (!lcd_is_ok())
        {
            return;
        }
    }

    lcd_line_P(0, proto_link_ok() ? PSTR("LeakDet v" FW_VERSION " ESP:+")
                                  : PSTR("LeakDet v" FW_VERSION " ESP:-"));

    /* Row 1: valve */
    switch (ctrl_motor())
    {
        case MOTOR_OPEN:  lcd_line_P(1, PSTR("Valve: OPENING...")); break;
        case MOTOR_CLOSE: lcd_line_P(1, PSTR("Valve: CLOSING...")); break;
        default:
            switch (valve_pos)
            {
                case POS_OPEN:   lcd_line_P(1, PSTR("Valve: OPEN"));     break;
                case POS_CLOSED: lcd_line_P(1, PSTR("Valve: CLOSED"));   break;
                case POS_FAULT:  lcd_line_P(1, PSTR("Valve: SW FAULT")); break;
                default:         lcd_line_P(1, PSTR("Valve: MIDDLE"));   break;
            }
            break;
    }

    /* Row 2: sensors */
    if (leak_mask == 0)
    {
        lcd_line_P(2, PSTR("Leak: none"));
    }
    else
    {
        strcpy_P(lcd_buf, PSTR("Leak: "));
        n = (uint8_t)strlen(lcd_buf);
        for (i = 0; i < LEAK_COUNT && n < LCD_COLS - 1; i++)
        {
            if (leak_mask & (1U << i))
            {
                lcd_buf[n++] = leak_cfg[i].tag;
                lcd_buf[n++] = ' ';
            }
        }
        lcd_buf[n] = '\0';
        lcd_commit(2);
    }

    /* Row 3: status */
    switch (ctrl_state())
    {
        case ST_INIT:    lcd_line_P(3, PSTR("Starting...")); break;
        case ST_OPEN:    lcd_line_P(3, PSTR("OK")); break;
        case ST_OPENING:
        case ST_CLOSING:
            if (ctrl_alarm()) lcd_line_P(3, PSTR("ALARM! Closing"));
            else              lcd_line_P(3, PSTR("Moving..."));
            break;
        case ST_CLOSED:  lcd_line_P(3, PSTR("Closed. RESET=open")); break;
        case ST_ALARM:   lcd_line_P(3, PSTR("ALARM! Press RESET")); break;
        case ST_FAULT:
            switch (ctrl_fault())
            {
                case FAULT_OPEN_TIMEOUT:  lcd_line_P(3, PSTR("FAULT: open timeout"));  break;
                case FAULT_CLOSE_TIMEOUT: lcd_line_P(3, PSTR("FAULT: close timeout")); break;
                default:                  lcd_line_P(3, PSTR("FAULT: switches"));      break;
            }
            break;
        default: break;
    }
}

#else
static void lcd_start(void)  { }
static void lcd_update(void) { }
#endif /* USE_LCD */

/* ======================================================================= */
/*  Init                                                                   */
/* ======================================================================= */

static void report_reset_cause(uint8_t mcucsr)
{
    if (mcucsr & (1 << WDRF))  log_P(PSTR("Reset: WATCHDOG"), NULL);
    if (mcucsr & (1 << BORF))  log_P(PSTR("Reset: brown-out"), NULL);
    if (mcucsr & (1 << EXTRF)) log_P(PSTR("Reset: external"), NULL);
    if (mcucsr & (1 << PORF))  log_P(PSTR("Reset: power-on"), NULL);
}

static void init_devices(void)
{
    cli();

    PORTA = 0x00;
    DDRA  = 0x00;

    relay_init();      // relays off as early as possible
    init_pins();
    uart0_init();
    pwm_init();

    MCUCR = 0x00;
    GICR  = 0x00;
    TIMSK = 0x00;
    timer2_init_1ms();

    sei();
}

static void fill_status(proto_status_t *st)
{
    st->state     = ctrl_state();
    st->pos       = valve_pos;
    st->leak_mask = leak_mask;
    st->fault     = ctrl_fault();
    st->alarm     = ctrl_alarm();
}

/* true once every period_ms; no catch-up bursts after a long stall */
static bool every(uint32_t *last, uint16_t period_ms)
{
    uint32_t now = millis();
    uint32_t elapsed = now - *last;

    if (elapsed < period_ms)
    {
        return false;
    }
    if (elapsed >= 2UL * period_ms)
    {
        *last = now;
    }
    else
    {
        *last += period_ms;
    }
    return true;
}

int main(void)
{
    uint8_t  mcucsr = MCUCSR;
    uint32_t t_logic = 0;
    uint32_t t_button = 0;
    ctrl_inputs_t in;
    proto_status_t st;

    MCUCSR = 0;
    wdt_enable(WDTO_2S);

    init_devices();

    transmitString_F(PSTR("\r\n"));
    log_P(PSTR("**************************************"), NULL);
    log_P(PSTR("  Leak Detector started. V." FW_VERSION), NULL);
    log_P(PSTR("  Type $? for the list of commands"), NULL);
    log_P(PSTR("**************************************"), NULL);
    report_reset_cause(mcucsr);
    proto_init();
    proto_send_hello();

    pwm_set(LCD_BRIGHTNESS);
    lcd_start();
    wdt_reset();

    ctrl_init(alarm_load());

    t_logic = t_button = millis();

    while (1)
    {
        wdt_reset();

        fill_status(&st);
        proto_poll(millis(), &st);

        if (every(&t_button, BUTTON_PERIOD_MS))
        {
            button_sample(&btn_reset);
            button_sample(&btn_close);
        }

        if (every(&t_logic, LOGIC_PERIOD_MS))
        {
            leak_update();
            valve_pos_update();

            in.leak_mask = leak_mask;
            in.pos       = valve_pos;
            /* "|" (not "||") so that both events are always consumed */
            in.btn_reset = button_take(&btn_reset) | proto_take_open();
            in.btn_close = button_take(&btn_close) | proto_take_close();
            in.now_ms    = millis();

            ctrl_step(&in);

            fill_status(&st);
            proto_tick(in.now_ms, &st);

            lcd_update();
        }
    }
}
