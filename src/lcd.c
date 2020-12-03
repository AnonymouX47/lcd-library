
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

    char c = 'a';
    unsigned char line = 0;
    signed curr_col;

    lcd_set_cursor(FIRST_ROW, 0);
    lcd_write_str("This is not cool!!!\n");
    while (1) {
        while (!PORTC);
        if (RC0) {
            c = (c >= 'a' && c <= 'z') ? c : 'a';
            lcd_write_char(c++);
        }
        else if (RC1)
            c--, lcd_backspace(1);
        else if (RC2)
            c = 'a', lcd_clr_curr_row();
        else if (RC3)
            lcd_cursor_left(1);
        else if (RC4)
            lcd_cursor_right(1);
        else if (RC5)
            lcd_shift_left(1);
        else if (RC6)
            lcd_shift_right(1);
        else if (RC7)
            lcd_scroll_anim(3, 0, 20, 4);

        while(PORTC);

        curr_col = lcd_cursor_col;
        lcd_set_cursor(!lcd_cursor_row, lcd_shift_pos);
        lcd_clr_row(SECOND_ROW);
        lcd_write_int(curr_col), lcd_cursor_right(1), lcd_write_int(lcd_shift_pos);
        lcd_set_cursor(!lcd_cursor_row, curr_col);

        // lcd_backspace(3);
        // lcd_write_int(l);

        // __delay_ms(500);
        // __delay_ms(500);
        // lcd_clr_disp();
        
    }
}
