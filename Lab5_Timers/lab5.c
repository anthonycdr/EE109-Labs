/********************************************
 *
 *  Name: Anthony Chandra
 *  Email: ac68801@usc.edu
 *  Section: Wednesday
 *  Assignment: Lab 5 - Timers
 *
 ********************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "lcd.h"

void debounce(uint8_t);
volatile unsigned ones, tens, tenths;
volatile unsigned l_ones, l_tens, l_tenths;

void timer1_init(void);

enum states { PAUSE, STARTRUN, RUN, LAPPED };

volatile uint8_t state = PAUSE;  

int main(void) {
    
    // Initialize the LCD and TIMER1
	lcd_init();
	timer1_init();
    
    // Enable pull-ups for buttons
	PORTC |= (1 << PC2) | (1 << PC4) | (1 << PC5);
    
    // Show the splash screen
 	lcd_moveto(0, 4); 
    lcd_stringout("Anthony");
    lcd_moveto(1, 2);
    lcd_stringout("EE109 Lab 5");
    _delay_ms(1000); 
    lcd_writecommand(1); 
    
	DDRC |= (1 << PC5);
    
    // Enable interrupts
    sei();
    
    while (1) { 
    
                    // Loop forever
	// Read the buttons
	 if ((PINC & (1 << PC2)) == 0){
    	if (state == PAUSE) {               // PAUSE state
        	state = STARTRUN;
        	TCCR1B |= (1 << CS12);  // Start timer immediately
        	_delay_ms(5);
    	}
		else if (state == RUN) {            // RUN state
			debounce(PC2);
			state = PAUSE;
     		TCCR1B &= ~(1 << CS12);  // Stop timer
    	} 
		else if (state == LAPPED) {         // LAPPED state
			debounce(PC2);
			state = RUN;
		}
	}
    
    // Transition from STARTRUN to RUN when button is released
    if (state == STARTRUN && (PINC & (1 << PC2)) != 0) {  
        _delay_ms(5);
        state = RUN;
    }
    
	if ((PINC & (1 << PC4)) == 0){
    	debounce(PC4); 
		if (state == PAUSE) {               // PAUSE state
        	ones = 0;  
			tens = 0;
			tenths = 0;
    	}
		else if (state == RUN) {            // RUN state
			state = LAPPED;
			l_tenths = tenths;
            l_ones = ones;
            l_tens = tens;
    	} 
		else if (state == LAPPED) {         // LAPPED state
			state = RUN;
		}
	}
	// If necessary write time to LCD
	lcd_moveto(0, 0);
    if (state == LAPPED) {
            if (l_tens > 0) {
                lcd_writedata(l_tens + '0');
            } else {
                lcd_writedata(' ');  
            }
            lcd_writedata(l_ones + '0');
            lcd_writedata('.');
            lcd_writedata(l_tenths + '0');
        } else {
            if (tens > 0) {
                lcd_writedata(tens + '0');
            } else {
                lcd_writedata(' ');  
            }
            lcd_writedata(ones + '0');
            lcd_writedata('.');
            lcd_writedata(tenths + '0');
        }
}
    
    return 0;   /* never reached */
}
/* ----------------------------------------------------------------------- */
void debounce(uint8_t bit)
{
    // Add code to debounce input "bit" of PINC
    // assuming we have sensed the start of a press.
    _delay_ms(10);  
    while ((PINC & (1 << bit)) == 0);  
    _delay_ms(10);
}
/* ----------------------------------------------------------------------- */
void timer1_init(void)
{
    // Add code to configure TIMER1 by setting or clearing bits in
    // the TIMER1 registers.
	TCCR1B |= (1 << WGM12); 
	TIMSK1 |= (1 << OCIE1A); 
    OCR1A = 6250;  
}
/* ----------------------------------------------------------------------- */
ISR(TIMER1_COMPA_vect)
{
	if (state == PAUSE){
		return;
	}
	
    tenths++;
    if (tenths == 10) {
        tenths = 0;
        ones++;          
        if (ones == 10) {
            ones = 0;
            tens++;
            if (tens == 6) {
                tens = 0;  
            }
        }
    } 
    // Increment the time
	PORTC ^= (1 << PC5); 
}
