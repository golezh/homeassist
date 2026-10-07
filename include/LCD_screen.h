#ifndef LCD_SCREEN_H
#define LCD_SCREEN_H

#include <stdint.h>
#include <stdbool.h>
#include <avr/pgmspace.h>

#define LCD_I2C_ADDR 0x27

#define LCD_COLS 20
#define LCD_ROWS 4

#define LCD_BACKLIGHT 0x08
#define LCD_ENABLE    0x04
#define LCD_RW        0x02
#define LCD_RS        0x01

/*
 * All functions are non-blocking with respect to bus faults: every I2C
 * wait has a timeout. After the first failed transfer the driver marks
 * the display as faulty and all further calls return immediately until
 * lcd_init() succeeds again. The display can never hang the main loop.
 */
void i2c_init(void);
bool lcd_init(void);        // returns true if the display answered
bool lcd_is_ok(void);
void lcd_clear(void);
void lcd_goto(uint8_t row, uint8_t col);
void lcd_print(const char *str);
void lcd_print_P(const char *str);   // string in flash (PSTR)

#endif // LCD_SCREEN_H
