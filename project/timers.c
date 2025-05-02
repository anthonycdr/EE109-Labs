#include <avr/io.h>
#include <avr/interrupt.h>
#include "project.h"

#define red  PC4
#define green PC3
#define blue  PC2
#define servo PB3

void timer1_init(void) {
    TCCR1A = (1 << WGM10) | (1 << WGM11);
    TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS11);
    OCR1A = 1999;
    OCR1B = 0;
    TIMSK1 = (1 << OCIE1B) | (1 << TOIE1);
}

void timer2_init(void) {
    TCCR2A = (1 << WGM21) | (1 << WGM20) | (1 << COM2A1);
    TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
    DDRB |= (1 << servo);
    OCR2A = 12;
    TIMSK2 = (1 << TOIE2);
}

ISR(TIMER1_OVF_vect) {
    PORTC |= (1 << green);
}

ISR(TIMER1_COMPB_vect) {
    PORTC &= ~(1 << green);
}

ISR(TIMER2_OVF_vect) {
    const uint8_t min_pos = 12; 
    const uint8_t max_pos = 35; 
    const uint16_t servo_time = 610;

    if (servo_active) {
        servo_counter++;
        if (servo_counter <= servo_time) {
            uint8_t pos = min_pos + ((uint32_t)servo_counter * (max_pos - min_pos)) / servo_time;
            OCR2A = pos;
        } else {
            servo_counter = servo_time;
        }
    } else {
        OCR2A = min_pos;
        servo_counter = 0;
    }
}
