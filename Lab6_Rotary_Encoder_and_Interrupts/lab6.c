/********************************************
 *
 *  Name: Anthony Chandra
 *  Email: ac68801@usc.edu
 *  Section: Wednesday 3:30
 *  Assignment: Lab 6 - Rotary Encoder
 *
 ********************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>

#include "lcd.h"

void play_note(uint16_t);
void variable_delay_us(int16_t);

// Frequencies for natural notes from middle C (C4)
// up one octave to C5.
uint16_t frequency[8] =
    { 262, 294, 330, 349, 392, 440, 494, 523 };

// Make variables global and volatile since they're accessed from ISR
volatile uint8_t new_state, old_state;
volatile uint8_t changed = 0;  // Flag for state change
volatile int16_t count = 0;    // Count to display
volatile uint16_t tcount = 0;  
volatile uint16_t max = 0;    

int main(void) {
    uint8_t a, b;

    // Initialize DDR and PORT registers and LCD
    lcd_init();
    
    PORTC |= (1 << PC1) | (1 << PC5);

    DDRC &= ~((1 << PC1) | (1 << PC5));

    DDRB |= (1 << PB4);

    // Write a splash screen to the LCD    
    lcd_moveto(0, 4); 
    lcd_stringout("Anthony");
    lcd_moveto(1, 2);
    lcd_stringout("EE109 Lab 5");
    _delay_ms(1000); 
    lcd_writecommand(1); 
    
    
    /* Call lcd_stringout to print out your name */
    lcd_moveto(0, 0);  // Move to first row
    lcd_stringout("Anthony");  // Display your name

   /* Use snprintf to create a string with your birthdate */

    char bday[17];
    snprintf(bday, 17, "Born: %2d/%02d/%4d", 11, 22, 2004);  

   /* Use lcd_moveto to start at an appropriate column
      in the bottom row to appear centered */
    lcd_moveto(1,0);

   /* Use lcd_stringout to print the birthdate string */
   lcd_stringout(bday);

   /* Delay 1 second */
   _delay_ms(1000);

   /* Use lcd_writecommand to clear the screen */
    lcd_writecommand(1);
    
    char buf[17];
    snprintf(buf, 17, "Count: %d", count);
    lcd_moveto(0,0);
    lcd_stringout(buf);  // Fixed: display count instead of bday
    
    while(0)
    {
      uint8_t x = PINC;
   
      uint8_t a = (x & (1 << PC1)) >> PC1;
      uint8_t b = (x & (1 << PC5)) >> PC5;
   
      char buf[10];
      snprintf(buf, 10, "A=%d B=%d", a, b);
   
      lcd_moveto(0,0);
      lcd_stringout(buf);
   
      _delay_ms(100); 
    }
    
    // Read the A and B inputs to determine the initial state.
    uint8_t x = PINC;
    a = (x & (1 << PC1)) >> PC1;
    b = (x & (1 << PC5)) >> PC5;

    if (!b && !a)
        old_state = 0;
    else if (!b && a)
        old_state = 1;
    else if (b && !a)
        old_state = 2;
    else
        old_state = 3;

    new_state = old_state;
    
    // Set up pin change interrupts
    PCICR |= (1 << PCIE1);  
    PCMSK1 |= (1 << PCINT9) | (1 << PCINT13);  
    
    sei();  // Enable global interrupts

    while (1) {                 // Loop forever
        if (changed) { // Did state change?
            changed = 0;        // Reset changed flag

            // Output count to LCD
            snprintf(buf, 17, "Count: %3d", count);
            lcd_moveto(0,0);
            lcd_stringout(buf);

            // Do we play a note?
            if ((count % 8) == 0) {
                // Determine which note (0-7) to play
                int16_t x = count;
                if (x < 0)
                    x = -x;
                
                uint8_t note = (x % 64)/8;
            
                // Find the frequency of the note
                uint16_t freq = frequency[note];

                // Call play_note and pass it the frequency
                play_note(freq);
            }
        }
    }
}

/*
  Play a tone at the frequency specified for one second
*/
void play_note(uint16_t freq)
{
   /* uint32_t period;

    period = 1000000 / freq;    // Period of note in microseconds

    while (freq--) {
        PORTB |= (1 << PB4);    // Buzzer output high
        variable_delay_us(period / 2);  // Delay for half the period
        PORTB &= ~(1 << PB4);   // Buzzer output low
        variable_delay_us(period / 2);  // Delay for half the period
    }*/
    
	max = freq * 2;
    tcount = 0;
    
    OCR1A = 16000000 / (2 * freq);
    
    timer1_init();
    
    TCCR1B |= (1 << CS10);
}

/*
    variable_delay_us - Delay a variable number of microseconds
*/
void variable_delay_us(int delay)
{
    int i = (delay + 5) / 10;

    while (i--)
        _delay_us(10);
}

ISR(PCINT1_vect)
{
    // Read the encoder inputs and determine the new count value
    uint8_t x = PINC;
    uint8_t a = (x & (1 << PC1)) >> PC1;
    uint8_t b = (x & (1 << PC5)) >> PC5;

    // For each state, examine the two input bits to see if state
    // has changed, and if so adjust the count value.
    if (old_state == 0) {
        // Handle A and B inputs for state 0
        if(!b && a) 
            new_state = 1, count++;
        else if(b && !a)
            new_state = 2, count--; 
    }
    else if (old_state == 1) {
        // Handle A and B inputs for state 1
        if(b && a)
            new_state = 3, count++;
        else if(!b && !a)
            new_state = 0, count--;
    }
    else if (old_state == 2) {
        // Handle A and B inputs for state 2
        if(!b && !a)
            new_state = 0, count++;
        else if(b && a)
            new_state = 3, count--;
    }
    else {   // old_state = 3
        // Handle A and B inputs for state 3
        if(b && !a)
            new_state = 2, count++;
        else if(!b && a)  // Fixed: Changed "if" to "else if" to avoid double increments
            new_state = 1, count--;
    }

    // If state changed, update the value of old_state,
    // and set a flag that the state has changed.
    if (new_state != old_state) {
        changed = 1;
        old_state = new_state;
    }
}

void timer1_init()
{
    // In Task 8, add code to initialize TIMER1, but don't start it counting
    TCCR1B |= (1 << WGM12);
    TCCR1B &= ~(1 << WGM13);
    TCCR1A &= ~((1 << WGM11) | (1 << WGM10));

    TIMSK1 |= (1 << OCIE1A);
    
    TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
    
    
}

ISR(TIMER1_COMPA_vect)
{
    // In Task 8, add code to change the output bit to the buzzer, and to turn
    // off the timer after enough periods of the signal have gone by.
    PORTB ^= (1 << PB4);
    
    tcount++;

    if (tcount >= max) {

        TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
        
        PORTB &= ~(1 << PB4);
        }
}