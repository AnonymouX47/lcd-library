
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

#define LCD_MODE 1
#include "lib/lcd.h"

void main(void)
{
    TRISA = TRISB = TRISC = TRISD = TRISE = 0x00;
    // TRISC = 0xFF;
    PORTA = PORTB = PORTC = PORTD = PORTE = 0x00;
    
    lcd_init(_2line, _5x8);

    char c = 'a';
    unsigned char line = 0;
    signed curr_col;

    lcd_clr_curr_row();
    lcd_write_str("Hello World!");
    __delay_ms(5000);
    lcd_clr_curr_row();
    while (1) {
        while (lcd_cursor_col < right_edge+1) {
            c = (c >= 'a' && c <= 'z') ? c : 'a';
            lcd_write_char(c++);
            lcd_cursor_down();
            __delay_ms(400);
            lcd_write_char(c++);
            lcd_cursor_up();
            __delay_ms(400);
        }

        lcd_scroll_anim(1, left_edge, right_edge, 5);
        lcd_clr_disp();
        c = 'a';
    }
}
