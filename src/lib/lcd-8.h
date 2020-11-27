/* 
 * File:   lcd-8.h
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
#define DB_recieve() TRISB |= 0xF0
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
__delay_us(2);\
EN = 0

// RAM designations
#define DDRAM 0
#define CGRAM 1


bool lcd_RAM, lcd_lines;
void lcd_wait(void);


// Data Bus Functions START

#if LCD_MODE  // 8-bit

void send_ins(unsigned char data)
{
    DB_send(); IR_write();
    DB_DATA = data;
    enable();
    lcd_wait();
}

void send_data(unsigned char data)
{
    DB_send(); DR_write();
    DB_DATA = data;
    enable();
    lcd_wait();
}

unsigned char read_data(void)
{
    DB_receive(); DR_read();
    enable();
    lcd_wait();
    return DB_DATA;
}

#else  // 4-bit

#define send_nibble(nib) \
DB4 = (nib) & 1;\
DB5 = (nib) >> 1 & 1;\
DB6 = (nib) >> 2 & 1;\
DB7 = (nib) >> 3 & 1

void send_ins(unsigned char ins)
{
    DB_send(); IR_write();
    send_nibble(data >> 4);  // upper nibble
    enable(); lcd_wait();
    IR_write();
    send_nibble(data & 0x0F);  // lower nibble
    enable(); lcd_wait();
}

void send_data(unsigned char data)
{
    DB_send(); DR_write();
    send_nibble(data >> 4);  // upper nibble
    enable(); lcd_wait();
    DR_write();
    send_nibble(data & 0x0F);  // lower nibble
    enable(); lcd_wait();
}

unsigned char read_data(void)
{
    DB_receive(); DR_read();
    enable(); lcd_wait();
    unsigned char upper = DB_DATA & 0xF0;
    DR_read();
    enable(); lcd_wait();
    
    return upper | DB_DATA >> 4;
}

#endif

// Data Bus Functions END

// Instructions START

void lcd_clr_disp(void)
{
    send_ins(CLR_DISP);
    __delay_ms(2);
}

void lcd_return_home(void)
{
    send_ins(RETURN_HOME);
}

void lcd_entry_mode(bool i_d, bool s)
{
    send_ins(ENTRY_MODE | i_d << 1 | s);
}

void lcd_display_set(bool d, bool c, bool b)
{
    send_ins(DISPLAY_SET | d << 2 | c << 1 | b);
}

void lcd_cur_disp_shift(bool s_c, bool r_l)
{
    send_ins(CUR_DISP_SHIFT | s_c << 3 | r_l << 2);
}

void lcd_function_set(bool dl, bool n, bool f)
{
    send_ins(FUNCTION_SET | dl << 4 | n << 3 | f << 2);
    lcd_lines = n;
}

bool lcd_set_cgram_adr(unsigned char address)
{
    if (address < 0x40) {
        lcd_RAM = CGRAM;
        send_ins(SET_CGRAM_ADR | address);
    } else
        return false;
    
    return true;
}

bool lcd_set_ddram_adr(unsigned char address)
{
    if (address < (lcd_lines == _1line ? 0x50 : 0x68)) {
        lcd_RAM = DDRAM;
        send_ins(SET_DDRAM_ADR | address);
    } else
        return false;
    
    return true;
}

void lcd_busy(void)
{
    IR_read(); DB_receive();
    enable();
}

unsigned char lcd_read_address(void)
{
    IR_read(); DB_receive();
    enable();
    return DB_DATA & 0x7F;
}

void (*lcd_write_char)(unsigned char) = &send_data;

unsigned char (*lcd_read_char)(void) = &read_data;

// Instructions END


void lcd_wait(void)
{
    lcd_busy();
    RD3 = 1;
    while (BF);
    RD3 = 0;
}

void lcd_init(bool n, bool f)
{
    TRISD = 0x00;
    lcd_function_set(LCD_MODE, n, f);
    __delay_us(100);
    lcd_display_set(LOW, LOW, LOW);  // Display OFF
    __delay_us(100);
    lcd_clr_disp();  // Display Clear
    __delay_ms(2);
    lcd_entry_mode(HIGH, LOW);  // Entry mode set
    __delay_us(100);
    lcd_display_set(1, 1, 1);  // Display ON, Cursor ON, Blinking ON
    __delay_us(100);
}


#endif	/* LCD_8_H */
