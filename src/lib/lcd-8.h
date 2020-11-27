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
#define DB_STATE TRISB
#define DB_DATA PORTB

#define RS RD7
#define R_W RD6
#define EN RD5

#define DB0 RB0
#define DB1 RB1
#define DB2 RB2
#define DB3 RB3
#define DB4 RB4
#define DB5 RB5
#define DB6 RB6
#define DB7 RB7

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


TRISD = 0xE0;

#endif	/* LCD_8_H */
