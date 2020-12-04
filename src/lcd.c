
// PIC16F877A Configuration Bit Settings

// CONFIG
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF        // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = ON         // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3/PGM pin has PGM function; low-voltage programming enabled)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#include <xc.h>
#define _XTAL_FREQ 20000000

#define LCD_MODE 1
#include "lib/lcd.h"

void main(void)
{
    TRISA = TRISB = TRISC = TRISD = TRISE = 0x00;
    TRISC = 0xFF;
    PORTA = PORTB = PORTC = PORTD = PORTE = 0x00;
    
    lcd_init(_2line, _5x8);

    signed long d = 123456;

    lcd_set_cursor(FIRST_ROW, 0);
    while (1) {
        lcd_clr_curr_row();
        lcd_write_int(d);

        while (!PORTC);
        if (RC0) {
            lcd_clr_curr_row();
            lcd_set_cursor(!lcd_cursor_row, 0);
        }
        else if (RC1) d += 125;
        else if (RC2) d -= 125;
        else if (RC3) d = -d;

        while(PORTC);
    }
}
