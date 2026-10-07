#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>
#include "LCD_screen.h"

/*
 * TWI status codes (TWSR & 0xF8), master transmitter
 */
#define TW_ST_START      0x08
#define TW_ST_REP_START  0x10
#define TW_ST_SLA_W_ACK  0x18
#define TW_ST_DATA_ACK   0x28

/*
 * One byte at 100 kHz takes ~90 us. 1000 loops with 1 us delay give
 * about 1.5-2 ms of timeout, which is plenty.
 */
#define I2C_TIMEOUT_LOOPS 1000U

static bool lcd_fault = true;   // true until lcd_init() succeeds

void i2c_init(void)
{
    /*
     * SCL = F_CPU / (16 + 2 * TWBR * prescaler)
     * F_CPU = 4 MHz, TWBR = 12, prescaler = 1  ->  SCL = 100 kHz
     */
    TWSR = 0x00;
    TWBR = (uint8_t)(((F_CPU / 100000UL) - 16UL) / 2UL);
    TWCR = 0x00;
}

static bool i2c_wait(void)
{
    uint16_t n = I2C_TIMEOUT_LOOPS;

    while (!(TWCR & (1 << TWINT)))
    {
        if (--n == 0)
        {
            return false;
        }
        _delay_us(1);
    }
    return true;
}

static void i2c_stop(void)
{
    uint16_t n = I2C_TIMEOUT_LOOPS;

    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);

    /* TWSTO is cleared by hardware when STOP has been sent */
    while (TWCR & (1 << TWSTO))
    {
        if (--n == 0)
        {
            /* bus is stuck: disable TWI to release SDA/SCL */
            TWCR = 0x00;
            return;
        }
        _delay_us(1);
    }
}

static bool i2c_start(void)
{
    uint8_t st;

    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    if (!i2c_wait())
    {
        return false;
    }

    st = TWSR & 0xF8;
    return (st == TW_ST_START) || (st == TW_ST_REP_START);
}

static bool i2c_write(uint8_t data, uint8_t expected_status)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    if (!i2c_wait())
    {
        return false;
    }

    return (TWSR & 0xF8) == expected_status;
}

static void lcd_i2c_write(uint8_t data)
{
    bool ok;

    if (lcd_fault)
    {
        return;
    }

    ok = i2c_start()
      && i2c_write((uint8_t)(LCD_I2C_ADDR << 1), TW_ST_SLA_W_ACK)
      && i2c_write((uint8_t)(data | LCD_BACKLIGHT), TW_ST_DATA_ACK);

    i2c_stop();

    if (!ok)
    {
        lcd_fault = true;
        TWCR = 0x00;
    }
}

static void lcd_pulse_enable(uint8_t data)
{
    lcd_i2c_write(data | LCD_ENABLE);
    _delay_us(1);

    lcd_i2c_write(data & (uint8_t)~LCD_ENABLE);
    _delay_us(50);
}

static void lcd_write4(uint8_t nibble, uint8_t rs)
{
    /*
     * PCF8574 backpack mapping:
     * P0 = RS, P1 = RW, P2 = EN, P3 = BACKLIGHT, P4..P7 = D4..D7
     */
    uint8_t data = (uint8_t)(nibble & 0xF0);

    if (rs)
    {
        data |= LCD_RS;
    }

    data |= LCD_BACKLIGHT;

    lcd_pulse_enable(data);
}

static void lcd_send(uint8_t value, uint8_t rs)
{
    lcd_write4(value & 0xF0, rs);
    lcd_write4((uint8_t)(value << 4) & 0xF0, rs);
}

static void lcd_command(uint8_t command)
{
    lcd_send(command, 0);
}

static void lcd_data(uint8_t data)
{
    lcd_send(data, 1);
}

bool lcd_init(void)
{
    i2c_init();
    lcd_fault = false;

    _delay_ms(50);

    /* HD44780 4-bit initialization sequence */
    lcd_write4(0x30, 0);
    _delay_ms(5);

    lcd_write4(0x30, 0);
    _delay_us(150);

    lcd_write4(0x30, 0);
    _delay_us(150);

    lcd_write4(0x20, 0);   // 4-bit mode
    _delay_us(150);

    lcd_command(0x28);     // 4-bit, 2-line mode, 5x8 font
    lcd_command(0x0C);     // Display ON, cursor OFF
    lcd_command(0x06);     // Entry mode
    lcd_command(0x01);     // Clear display
    _delay_ms(2);

    return !lcd_fault;
}

bool lcd_is_ok(void)
{
    return !lcd_fault;
}

void lcd_clear(void)
{
    if (lcd_fault)
    {
        return;
    }
    lcd_command(0x01);
    _delay_ms(2);
}

void lcd_goto(uint8_t row, uint8_t col)
{
    /* 20x4 HD44780 line addresses */
    static const uint8_t row_offsets[LCD_ROWS] = { 0x00, 0x40, 0x14, 0x54 };

    if (row >= LCD_ROWS)
    {
        row = LCD_ROWS - 1;
    }
    if (col >= LCD_COLS)
    {
        col = LCD_COLS - 1;
    }

    lcd_command((uint8_t)(0x80 | (row_offsets[row] + col)));
}

void lcd_print(const char *str)
{
    while (*str && !lcd_fault)
    {
        lcd_data((uint8_t)*str);
        str++;
    }
}

void lcd_print_P(const char *str)
{
    char c;

    while (((c = (char)pgm_read_byte(str++)) != '\0') && !lcd_fault)
    {
        lcd_data((uint8_t)c);
    }
}
