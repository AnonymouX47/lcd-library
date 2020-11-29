/* 
 * File:   lcd.h
 * Author: anonymoux47
 *
 * Created on 26 November 2020, 23:54
 */

#ifndef LCD_8_H
#define	LCD_8_H

#include <stdbool.h>

enum {LOW, HIGH};

// Data Bus State
#if LCD_MODE  // 8-bit
#define DB_send() TRISB = 0x00
#define DB_receive() TRISB = 0xFF
#else  // 4-bit
#define DB_send() TRISB &= 0x0F
#define DB_receive() TRISB |= 0xF0
#endif

// MCU to LCD connection
#define DB_DATA PORTB
#define DB0 RB0
#define DB1 RB1
#define DB2 RB2
#define DB3 RB3
#define DB4 RB4
#define DB5 RB5
#define DB6 RB6
#define DB7 RB7
#define RS RD7
#define R_W RD6
#define EN RD5

// HD44780 Register Selection and modes
#define IR_write() RS = R_W = 0
#define IR_read() RS = 0, R_W = 1
#define DR_write() RS = 1, R_W = 0
#define DR_read() RS = R_W = 1

// HD44780 Instructions (headers) and parameters
#define CLR_DISP 0x01
#define RETURN_HOME 0x02

#define ENTRY_MODE 0x04
#define I_D DB1
#define S DB0

#define DISPLAY_SET 0x08
#define D DB2
#define C DB1
#define B DB0

#define CUR_DISP_SHIFT 0x10
#define S_C DB3
#define R_L DB2

#define FUNCTION_SET 0x20
// Data length
#define DL DB4
#define _4bit 0
#define _8bit 1
// No of display lines
#define N DB3
#define _1line 0
#define _2line 1
// Character font
#define F DB2
#define _5x8 0
#define _5x10 1

#define SET_CGRAM_ADR 0x40
#define SET_DDRAM_ADR 0x80

// Busy Flag
#define BF DB7

// Enable signal
#define enable() \
EN = 1;\
__delay_us(1);\
EN = 0

// RAM designations
#define DDRAM 0
#define CGRAM 1
bool lcd_RAM;

// Cursor control
#define FIRST_ROW 0
#define SECOND_ROW 1
// LINE1_ENDX is also the maximum address offset per line with X lines
#define LINE1_END1 0x4f
#define LINE1_END2 0x27
#define LINE1_BEGIN 0x00
#define LINE2_BEGIN 0x40
#define LINE2_END 0x67
bool lcd_lines;

// Other macros
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

void lcd_wait(void);

// Data Bus Functions START

#if LCD_MODE  // 8-bit

void lcd_send(bool reg, unsigned char data)
{
    DB_send();
    if (reg) DR_write(); else IR_write();
    DB_DATA = data;
    enable(); lcd_wait();
}

#else  // 4-bit

#define send_nibble(nib) \
    DB_DATA = (DB_DATA & 0x0f) | (nib & 0xf0);\
    enable(); lcd_wait()

void lcd_send(bool reg, unsigned char data)
{
    DB_send();
    if (reg) DR_write(); else IR_write();
    send_nibble(data);  // upper nibble
    // DR_write();
    send_nibble(data << 4);  // lower nibble
}

#endif

// Data Bus Functions END

// Instructions START

void lcd_clr_disp(void)
{
    lcd_send(LOW, CLR_DISP);
    __delay_ms(2);
}

void lcd_return_home(void)
{
    lcd_send(LOW, RETURN_HOME);
    __delay_ms(2);
}

void lcd_entry_mode(bool i_d, bool s)
{
    lcd_send(LOW, ENTRY_MODE | i_d << 1 | s);
}

void lcd_display_set(bool d, bool c, bool b)
{
    lcd_send(LOW, DISPLAY_SET | d << 2 | c << 1 | b);
}

void lcd_cur_disp_shift(bool s_c, bool r_l)
{
    lcd_send(LOW, CUR_DISP_SHIFT | s_c << 3 | r_l << 2);
}

void lcd_function_set(bool dl, bool n, bool f)
{
    lcd_send(LOW, FUNCTION_SET | dl << 4 | n << 3 | f << 2);
    lcd_lines = n;
}

bool lcd_set_cgram_adr(unsigned char address)
{
    if (address < 0x40) {
        lcd_RAM = CGRAM;
        lcd_send(LOW, SET_CGRAM_ADR | address);
    } else
        return false;
    
    return true;
}

bool lcd_set_ddram_adr(unsigned char address)
{
    if (address <= (lcd_lines == _1line ? LINE1_END1 : LINE2_END)) {
        lcd_RAM = DDRAM;
        lcd_send(LOW, SET_DDRAM_ADR | address);
    } else
        return false;
    
    return true;
}

void lcd_busy(void)
{
    DB_receive(); IR_read();
    enable();
    while (BF);
}

unsigned char lcd_read_address(void)
{
    DB_receive(); IR_read();
    enable();
    lcd_wait();
    
    if (LCD_MODE == _4bit) {
        RD2 = 1;
        unsigned char upper = DB_DATA & 0x70;
        enable();
        lcd_wait();
        return upper | DB_DATA >> 4;
    }
    
    return DB_DATA & 0x7F;
}

#define lcd_write_char(data) lcd_send(HIGH, data)

#if LCD_MODE  // 8-bit

unsigned char lcd_read_char(void)
{
    DB_receive(); DR_read();
    enable(); lcd_wait();
    return DB_DATA;
}

#else  // 4-bit

unsigned char lcd_read_char(void)
{
    DB_receive(); DR_read();
    enable(); lcd_wait();
    unsigned char upper = DB_DATA & 0xF0;
    // DR_read();
    enable(); lcd_wait();
    
    return upper | DB_DATA >> 4;
}

#endif

// Instructions END


// Utility functions

void lcd_wait(void)
{
    __delay_us(40);
}

void lcd_init(bool n, bool f)
{
    TRISD = 0x00;

// The first 'function set' must be sent in 8-bit mode due to internal initialization
// Turned out 'function set' was required twice (in 8-bit mode) to take effect.

#if LCD_MODE
    lcd_function_set(_8bit, n, f);
#else
    DB_send(); IR_write();
    send_nibble((FUNCTION_SET | n << 3 | f << 2));
    send_nibble((FUNCTION_SET | n << 3 | f << 2));
#endif
    lcd_function_set(LCD_MODE, n, f);
    lcd_display_set(LOW, LOW, LOW);  // Display OFF
    lcd_clr_disp();  // Clear display
    lcd_entry_mode(HIGH, LOW);  // Entry mode set
    lcd_display_set(1, 1, 1);  // Display ON, Cursor ON, Blinking ON
}

void lcd_set_cursor(bool row, unsigned char col)
{
    if (lcd_lines == _2line)
        lcd_set_ddram_adr((row ? LINE2_BEGIN : LINE1_BEGIN) + min(col, LINE1_END2));
    else
        lcd_set_ddram_adr(LINE1_BEGIN + min(col, LINE1_END1));
}

void lcd_clr_row(bool row)
{
    unsigned char i = lcd_lines ? LINE1_END2 : LINE1_END1;
    lcd_set_cursor(row, 0);
    while (i--) lcd_write_char(' ');
    lcd_set_cursor(row, 0);
}

// For cursor movement and display shifts
#define CURSOR 0
#define DISPLAY 1
#define LEFT 0
#define RIGHT 1

/* move the cursor left n times */
void lcd_cursor_left(unsigned char n)
{
    while(n--) lcd_cur_disp_shift(CURSOR, LEFT);
}

void lcd_cursor_right(unsigned char n)
{
    while(n--) lcd_cur_disp_shift(CURSOR, RIGHT);
}

void lcd_shift_left(unsigned char n)
{
    while(n--) lcd_cur_disp_shift(DISPLAY, LEFT);
}

void lcd_shift_right(unsigned char n)
{
    while(n--) lcd_cur_disp_shift(DISPLAY, RIGHT);
}

#endif	/* LCD_8_H */
