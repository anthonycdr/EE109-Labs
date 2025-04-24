/********************************************
 *
 *  Name: Anthony Chandra
 *  Email: ac68801@usc.edu
 *  Section: Wednesday 3 - 4:50 p.m.
 *  Assignment: Lab 7 - ADC and PWM
 *
 ********************************************/
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

#include "lcd.h"
#include "adc.h"

void timer2_init(void); // Function prototype

#define VARIABLE 0
#define RIGHT 1
#define LEFT 2

int main(void)
{
    char buf[5]; 
    
    // Initialize the LCD
    lcd_init();
    
    // Initialize the ADC
    adc_init();
    
    // Initialize TIMER2
    timer2_init();
    
    // Write splash screen and delay for 1 second
    lcd_moveto(0, 0);
    lcd_stringout("Anthony");
    lcd_moveto(1, 0);
    lcd_stringout("EE109 Lab 7");
    _delay_ms(1000);
    
    lcd_writecommand(1);
    
    // Use this "while (1)" loop ONLY for doing Tasks 2 and 3
    while (0) {
        // Use adc_sample to read ADC value for buttons
        
        // Use snprintf and lcd_stringout to display number on LCD
        snprintf(buf, 5, "%4d", adc_sample(0));
        
        // Move cursor to first position and display the ADC value
        lcd_moveto(0, 0);
        lcd_stringout("ADC Value: ");
        lcd_stringout(buf);
        
        // Add a small delay to make the display readable
        _delay_ms(200);
    }
    
    // state def
    uint8_t state = VARIABLE;
    uint8_t old = 24; 
    
    while (1) {                 // Loop forever
        // Check buttons and determine state
        uint8_t button_val = adc_sample(0);
        
        if (button_val <= 2) {
            state = RIGHT;
        } else if (button_val >= 155 && button_val <= 156) {
            state = LEFT;
        } else if (button_val == 206) {
            state = VARIABLE;
        }
        
        // Change output based on state
        // If RIGHT or LEFT button pressed, move servo accordingly
        // If SELECT button pressed read potentiometer ADC channel
		//    Convert ADC value to OCR2A number for PWM signal
        uint8_t new;
        
        if (state == VARIABLE) {
            uint8_t pot_val = adc_sample(1);
            uint16_t temp = (pot_val * 23) / 255;
            new = 35 - temp;
        } else if (state == RIGHT) {
            new = 12;
        } else { 
            new = 35;
        }
        
        OCR2A = new;
    
		// Display the PWM value on the LCD
        if (new!= old) {
            lcd_moveto(1, 3);
            lcd_stringout("PWM = ");
            snprintf(buf, 5, "%3d", new);
            lcd_stringout(buf);
            
            old= new;
        }
        _delay_ms(200); // debounce
    }
    return 0;   /* never reached */
}

/*
  timer2_init - Initialize Timer/Counter2 for Fast PWM
*/
void timer2_init(void)
{
    // Add code to initialize TIMER2
    // This will be implemented for the servo control part
    DDRB |= (1 << PB3);
    
    TCCR2A |= (0b11 << WGM20);  // Fast PWM mode, modulus = 256
    TCCR2A |= (0b10 << COM2A0); // Turn D11 on at 0x00 and off at OCR2A
    OCR2A = 24;        // Initial pulse width (calculate this)
    TCCR2B |= (0b111 << CS20);  // Prescaler = 1024 for 16ms period
}