/* 
 * File:   lcd-8.h
 * Author: anonymoux47
 *
 * Created on 26 November 2020, 23:54
 */

#ifndef LCD_8_H
#define	LCD_8_H

#include <stdbool.h>

// MCU to LCD connection

// Data Bus
#define DB_send() if (TRISB) TRISB = 0x00
#define DB_receive() if (!TRISB) TRISB = 0xFF
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
#define DR_read() RS = 1, R_W = 1

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
#define DL DB4
#define N DB3
#define F DB2

#define SET_CGRAM_ADR 0x40
#define SET_DDRAM_ADR 0x80

// Busy Flag
#define BF DB7

// Enable signal
#define enable()    EN = 1;\
                    __delay_us(1);\
                    EN = 0

// RAM designations
#define DDRAM 0
#define CGRAM 1
bool lcd_RAM;

bool lcd_lines;  // same as `N`

TRISD = 0x00;

// Instructions START
bool lcd_busy(void);

void lcd_clr_disp(void)
{
    IR_write(); DB_send();
    DB_DATA = CLR_DISP;
    enable();
    while (lcd_busy());
}

void lcd_return_home(void)
{
    IR_write(); DB_send();
    DB_DATA = RETURN_HOME;
    enable();
    while (lcd_busy());
}

void lcd_entry_mode(bool i_d, bool s)
{
    IR_write(); DB_send();
    DB_DATA = ENTRY_MODE;
    I_D = i_d, S = s;
    enable();
    while (lcd_busy());
}

void lcd_display_set(bool d, bool c, bool b)
{
    IR_write(); DB_send();
    DB_DATA = DISPLAY_SET;
    D = d, C = c, B = b;
    enable();
    while (lcd_busy());
}

void lcd_cur_disp_shift(bool s_c, bool r_l)
{
    IR_write(); DB_send();
    DB_DATA = CUR_DISP_SHIFT;
    S_C = s_c, R_L = r_l;
    enable();
    while (lcd_busy());
}

void lcd_function_set(bool dl, bool n, bool f)
{
    IR_write(); DB_send();
    DB_DATA = FUNCTION_SET;
    DL = dl, N = n, F = f;
    enable();
    while (lcd_busy());
    lcd_lines = n;
}

bool lcd_set_cgram_adr(unsigned char address)
{
    IR_write(); DB_send();
    if (address < 0x40) {
        lcd_RAM = CGRAM;
        DB_DATA = SET_CGRAM_ADR | address;
    } else
        return false;
    enable();
    while (lcd_busy());
    
    return true;
}

bool lcd_set_ddram_adr(unsigned char address)
{
    IR_write(); DB_send();
    if (address < (lcd_lines ? 0x68 : 0x50)) {
        lcd_RAM = DDRAM;
        DB_DATA = SET_DDRAM_ADR | address;
    } else
        return false;
    enable();
    while (lcd_busy());
    
    return true;
}

bool lcd_busy(void)
{
    IR_read(); DB_receive();
    enable();
    while (lcd_busy());
    return BF;
}

unsigned char lcd_read_address(void)
{
    IR_read(); DB_receive();
    enable();
    while (lcd_busy());
    return (DB_DATA << 1) >> 1;
}

void lcd_write_char(unsigned char data)
{
    DR_write(); DB_send();
    DB_DATA = data;
    enable();
    while (lcd_busy());
}

unsigned char lcd_read_char(void)
{
    DR_read(); DB_receive();
    enable();
    while (lcd_busy());
    return DB_DATA;
}

// Instructions END

#endif	/* LCD_8_H */
