/* 
 * File:   lcd.h
 * Author: anonymoux47
 *
 * Created on 26 November 2020, 23:54
 */

#ifndef LCD_8_H
#define LCD_8_H

#include <stdio.h>
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
#define DB0 PORTBbits.RB0
#define DB1 PORTBbits.RB1
#define DB2 PORTBbits.RB2
#define DB3 PORTBbits.RB3
#define DB4 PORTBbits.RB4
#define DB5 PORTBbits.RB5
#define DB6 PORTBbits.RB6
#define DB7 PORTBbits.RB7
#define RS PORTDbits.RD7
#define R_W PORTDbits.RD6
#define EN PORTDbits.RD5

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
__bit lcd_RAM;

// Cursor control
#define FIRST_ROW 0
#define SECOND_ROW 1
// LINE1_ENDX is also the maximum address offset per line with X lines
#define LINE1_END1 0x4f
#define LINE1_END2 0x27
#define LINE1_BEGIN 0x00
#define LINE2_BEGIN 0x40
#define LINE2_END 0x67
__bit lcd_lines;  // Still only for 2 lines max

// Other macros
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define to_bit(n) ((n) ? 1 : 0)

signed char lcd_cursor_col, lcd_shift_pos;
signed char left_edge, right_edge;
__bit entry_mode_i_d, entry_mode_s,
      lcd_cursor_row;  // Still only for 2 lines max

void lcd_wait(void);

// Data Bus Timing Delays
// data_delay + data_setup > 450ns Min required for Pulse width
//  Write only
#define delay_data_setup() __delay_us(1)  // Data setup time (Min: 195ns)
//  Read only
#define delay_data_delay() __delay_us(1)  // Data delay time (Max: 360ns)
//  Both
#define delay_addr_setup() __delay_us(1)  // Address setup time (Min: 60ns)
#define delay_en_cycle() __delay_us(1)  // Make-up for Enable cyle time
#define delay_hold() __delay_us(1)  // Address and Data Hold time (Min: 20ns)

// Data Bus Functions START

#define pulse(operation) \
    delay_addr_setup(); \
    EN = HIGH; \
    delay_data_delay(); \
    operation;\
    delay_data_setup(); \
    EN = LOW; \
    delay_hold()

#if LCD_MODE  // 8-bit

void lcd_send(bool reg, unsigned char data)
{
    DB_send();
    if (reg) DR_write(); else IR_write();

    pulse(DB_DATA = data);

    lcd_wait();
}

char lcd_read(bool reg)
{
    char data = 0;

    DB_receive();
    if (reg) DR_read(); else IR_read();

    pulse(data = DB_DATA);

    return data;
}

#else  // 4-bit

#define send_nibble(nib) DB_DATA = (nib) | (DB_DATA & 0x0f)

void lcd_send(bool reg, unsigned char data)
{
    DB_send();
    if (reg) DR_write(); else IR_write();
    pulse(send_nibble(data & 0xf0));  // upper nibble
    pulse(send_nibble(data << 4));  // lower nibble

    lcd_wait();
}

char lcd_read(bool reg)
{
    char data = 0;

    DB_receive();
    if (reg) DR_read(); else IR_read();
    pulse(data = DB_DATA & 0xf0);
    pulse(data |= DB_DATA >> 4);
    
    return data;
}

#endif

// Data Bus Functions END

// Instructions START

void lcd_clr_disp(void)
{
    void lcd_entry_mode(bool i_d, bool s);

    lcd_send(LOW, CLR_DISP);
    lcd_wait();

    lcd_cursor_row = lcd_cursor_col = lcd_shift_pos = 0;
    lcd_entry_mode(entry_mode_i_d, entry_mode_s);  // Restore 'entry mode'
}

void lcd_return_home(void)
{
    lcd_send(LOW, RETURN_HOME);
    lcd_wait();

    lcd_cursor_row = lcd_cursor_col = lcd_shift_pos = 0;
}

void lcd_entry_mode(bool i_d, bool s)
{
    entry_mode_i_d = i_d, entry_mode_s = s;
    lcd_send(LOW, ENTRY_MODE | to_bit(i_d) << 1 | to_bit(s));
}

void lcd_display_set(bool d, bool c, bool b)
{
    lcd_send(LOW, DISPLAY_SET | to_bit(d) << 2 | to_bit(c) << 1 | to_bit(b));
}

void lcd_cur_disp_shift(bool s_c, bool r_l)
{
    lcd_send(LOW, CUR_DISP_SHIFT | to_bit(s_c) << 3 | to_bit(r_l) << 2);
}

void lcd_function_set(bool dl, bool n, bool f)
{
    lcd_send(LOW, FUNCTION_SET | to_bit(dl) << 4 | to_bit(n) << 3 | to_bit(f) << 2);
    lcd_lines = to_bit(n);  // Still only for 2 lines max
}

__bit lcd_set_cgram_adr(unsigned char address)
{
    if (address < 0x40) {
        lcd_RAM = CGRAM;
        lcd_send(LOW, SET_CGRAM_ADR | address);
    } else
        return 0;
    
    return 1;
}

__bit lcd_set_ddram_adr(unsigned char address)
{
    if (address <= (lcd_lines == _1line ? LINE1_END1 : LINE2_END) + entry_mode_i_d) {
        lcd_RAM = DDRAM;
        lcd_send(LOW, SET_DDRAM_ADR | address);
    } else
        return 0;
    
    return 1;
}

#define lcd_busy_flag lcd_read(LOW) & 0x80

unsigned char lcd_read_address(void)
{
    return lcd_read(LOW) & 0x7f;
}

void lcd_shift_left(signed char);
void lcd_shift_right(signed char);

void lcd_write_char(char data)
{
    if (lcd_cursor_col == (entry_mode_i_d ? right_edge+1 : left_edge-1))
        return;

    lcd_send(HIGH, data);

    if (entry_mode_i_d && lcd_cursor_col == lcd_shift_pos + 15)
        lcd_shift_left(1);
    else if (!entry_mode_i_d && lcd_cursor_col == lcd_shift_pos)
        lcd_shift_right(1);

    lcd_cursor_col += entry_mode_i_d ? 1 : -1;
    if (entry_mode_s) lcd_shift_pos += entry_mode_i_d ? 1 : -1;
}

unsigned char lcd_read_char(void)
{
    lcd_cursor_col += entry_mode_i_d ? 1 : -1;

    return lcd_read(HIGH);
}

// Instructions END


// Utility functions

void lcd_wait(void)
{
    while (lcd_busy_flag) delay_en_cycle();
}

/* Initializes LCD */
void lcd_init(bool n, bool f)
{
    TRISD &= 0x1f;
    n = to_bit(n);  // Still only for 2 lines max
    f = to_bit(f);

// The first 'function set' must be sent in 8-bit mode due to internal initialization
// Turned out 'function set' was required twice (in 8-bit mode) to take effect.

#if LCD_MODE  // 8-bit
    lcd_function_set(_8bit, n, f);
#else  // 4-bit
    DB_send(); IR_write();
    pulse(send_nibble(FUNCTION_SET));
    __delay_ms(40);
    pulse(send_nibble(FUNCTION_SET));
    __delay_ms(40);
#endif
    lcd_function_set(to_bit(LCD_MODE), n, f);
    lcd_display_set(LOW, LOW, LOW);  // Display OFF
    lcd_clr_disp();  // Clear display
    lcd_entry_mode(HIGH, LOW);  // Entry mode set
    lcd_display_set(HIGH, HIGH, HIGH);  // Display ON, Cursor ON, Blinking ON

    __delay_ms(5);

    left_edge = 0;
    right_edge = lcd_lines ? LINE1_END2 : LINE1_END1;
}

#define lcd_reverse_cursor() lcd_entry_mode(!entry_mode_i_d, entry_mode_s)

// For cursor movement and display shifts
#define CURSOR 0
#define DISPLAY 1
#define LEFT 0
#define RIGHT 1

/* Places the cursor at the sepecified location
 * if (`row` > max_row): sets to last row */
void lcd_set_cursor(bool row, signed char col)
{
    lcd_set_ddram_adr((to_bit(row) ? LINE2_BEGIN : LINE1_BEGIN) \
                        + min(col, right_edge + entry_mode_i_d));

    lcd_cursor_row = lcd_lines ? to_bit(row) : 0;  // Still only for 2 lines max
    lcd_cursor_col = min( max(col, left_edge - !entry_mode_i_d), \
                        right_edge + entry_mode_i_d );
    
    if (lcd_cursor_col < lcd_shift_pos)
        lcd_shift_right(lcd_shift_pos - lcd_cursor_col);
    else if (lcd_cursor_col > lcd_shift_pos + 15)
        lcd_shift_left(lcd_cursor_col - lcd_shift_pos);
}

/* Jump to start of current line */
void lcd_goto_begin(void)
{
    lcd_set_cursor(lcd_cursor_row, left_edge);
}

/* Jump to end of current line */
void lcd_goto_end(void)
{
    lcd_set_cursor(lcd_cursor_row, right_edge);
}

/* Clears a row of display with the cursor position and display shift unaffected
 * row = 0: Clears first row, row = 1: Clears second row
 * if (`row` > max_row): clears last row */
void lcd_clr_row(unsigned char row)
{
    signed char prev_row = lcd_cursor_row, prev_col = lcd_cursor_col,
                  i = right_edge + 1;

    lcd_set_ddram_adr(to_bit(row) ? LINE2_BEGIN : LINE1_BEGIN);
    while (i--) lcd_send(HIGH, ' ');

    lcd_set_cursor(prev_row, prev_col);
}

/* Clears current row and resets cursor position and display shift
 * to the begining of the line */
void lcd_clr_curr_row(void)
{
    lcd_clr_row(lcd_cursor_row);
    lcd_set_cursor(lcd_cursor_row, 0);
}

/* Moves the cursor up once (if possible) and retains cursor column */
void lcd_cursor_up(void)
{
    if (lcd_cursor_row)
        lcd_set_cursor(lcd_cursor_row-1, lcd_cursor_col);
}

/* Moves the cursor down once (if possible) and retains cursor column */
void lcd_cursor_down(void)
{
    if (lcd_cursor_row < lcd_lines)
        lcd_set_cursor(lcd_cursor_row+1, lcd_cursor_col);
}

/* Moves the cursor `n` times to the left */
void lcd_cursor_left(signed char n)
{
    while(n-- > 0 && lcd_cursor_col-- > left_edge) {
        lcd_cur_disp_shift(CURSOR, LEFT);
        if (lcd_cursor_col == lcd_shift_pos)
            // Reached one column to the left edge
            lcd_shift_right(1);
            while(n--) {
                lcd_cur_disp_shift(CURSOR, LEFT);
                lcd_shift_right(1);
            }
    }

    if (lcd_cursor_col < left_edge) lcd_cursor_col = left_edge;
}

/* Moves the cursor `n` times to the right */
void lcd_cursor_right(signed char n)
{
    while(n-- > 0 && lcd_cursor_col++ < right_edge) {
        lcd_cur_disp_shift(CURSOR, RIGHT);
        if (lcd_cursor_col == lcd_shift_pos + 15)
            // Reached one column to the right edge
            lcd_shift_left(1);
            while(n--) {
                lcd_cur_disp_shift(CURSOR, RIGHT);
                lcd_shift_left(1);
            }
    }

    if (lcd_cursor_col > right_edge) lcd_cursor_col = right_edge;
}

/* Shifts the display `n` times to the left */
void lcd_shift_left(signed char n)
{
    signed char edge = right_edge - 15;

    while(n-- && lcd_shift_pos++ < edge) lcd_cur_disp_shift(DISPLAY, LEFT);
    
    if (lcd_shift_pos > edge) lcd_shift_pos = edge;
}

/* Shifts the display `n` times to the right */
void lcd_shift_right(signed char n)
{
    while(n-- && lcd_shift_pos-- > left_edge) lcd_cur_disp_shift(DISPLAY, RIGHT);

    if (lcd_shift_pos < left_edge) lcd_shift_pos = left_edge;
}

/* Deletes `n` characters backward on display */
void lcd_backspace(signed char n)
{
    signed char edge = entry_mode_i_d ? left_edge-1 : right_edge+1;

    // Move over to previous character
    if (entry_mode_i_d) lcd_cursor_left(1);
    else lcd_cursor_right(1);

    lcd_entry_mode(!entry_mode_i_d, LOW);  // Reverse cursor direction

    while(n-- && lcd_cursor_col != edge) lcd_write_char(' ');

    lcd_entry_mode(!entry_mode_i_d, LOW);  // Restore cursor direction

    // Move cursor to next position
    if (entry_mode_i_d) lcd_cursor_right(1);
    else lcd_cursor_left(1);
}

/* Displays null-terminated character string `s` */
unsigned char lcd_write_str(const char * const restrict s)
{
    const char * restrict p = s;

    while(*p) lcd_write_char(*p++);

    return p - s;
}

/* Displays an integer `n` of <= 10 digits (long int = 32bit)
 *
 * returns number of characters displayed */
unsigned char lcd_write_int(unsigned long n)
{
    char s[12] = {0}, *p = s + sizeof(s) - 1;

    if ((long)n < 0) {
        lcd_write_char('-');
        p--;
        n = -(long)n;
    }

    do *--p = '0' + n % 10;
    while (p > s && (n /= 10));
    lcd_write_str(p);

    return s + sizeof(s) - p - 1;
}


/* Displays a floating-point number `n` with `dp` decimal places
 * and <= 16 digits all together (not including '-' or '.')
 *
 * returns number of characters displayed */
unsigned char lcd_write_float(double n, unsigned char dp)
{
    char s[17];

    sprintf(s, "%.*f", dp, n);
    sprintf(s, "%.16s", s);

    return lcd_write_str(s);
}

/* Scroll display forth and back `n` times
 * showing only charater positions from `start` to `end`
 * at a speed of one shift per (`speed`*50)ms and 1s delay at both ends of scrolling */
void lcd_scroll_anim
(unsigned char n, signed char start, signed char end, const unsigned char speed)
{
    if (start > end) return;
    start = max(left_edge, start), end = min(end, right_edge);

    unsigned char i = max(end - start, 15) - 15, j, k;

    lcd_set_cursor(lcd_cursor_row, start);
    while (n--) {
        __delay_ms(500);
        for (j = i; j--;) {
            lcd_shift_left(1);
            for (k = speed; k--;) __delay_ms(50);
        }
        __delay_ms(500);
        for (j = i; j--;) {
            lcd_shift_right(1);
            for (k = speed; k--;) __delay_ms(50);
        }
    }
    __delay_ms(500);
}

#endif  /* LCD_8_H */
