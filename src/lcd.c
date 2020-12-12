
// PIC16F877A Configuration Bit Settings

// CONFIG
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF        // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF         // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3/PGM pin has PGM function; low-voltage programming enabled)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#include <xc.h>
#define _XTAL_FREQ 20000000

#define LCD_MODE 0
#include "lib/lcd.h"

void main(void)
{
    TRISA = TRISB = TRISC = TRISD = TRISE = 0x00;
    PORTA = PORTB = PORTC = PORTD = PORTE = 0x00;

    lcd_init(_1line, _5x8);
    lcd_reverse_cursor();

    char c;
    unsigned char line=0, col;
    while (1) {
        c = 'a';
        lcd_goto_end();
        for (unsigned char i = right_edge+1; i--; c = (c >= 'a' && c <= 'z') ? c : 'a') {
            lcd_write_char(c++);
            __delay_ms(300);
        }
        __delay_ms(1000);
        lcd_goto_end();
        for (unsigned char i = 10; i--;) {
            lcd_cur_disp_shift(1, 0);
            __delay_ms(500);
        }
        lcd_return_home();
        __delay_ms(1000);
        for (unsigned char i = 10; i--;) {
            lcd_cur_disp_shift(1, 1);
            __delay_ms(500);
        }

        lcd_clr_disp();
    }
}
