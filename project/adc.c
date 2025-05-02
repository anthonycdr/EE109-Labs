#include <avr/io.h>

#include "adc.h"


void adc_init(void)
{
    // Initialize the ADC
    
    // Set/clear the REFS[1:0] bits in ADMUX to select the high voltage reference. Using the AVCC reference is appropriate for this lab.
    ADMUX |= (1 << REFS0);
    ADMUX &= ~(1 << REFS1);
    
    // Set or clear the ADLAR bit in ADMUX such that we will use 8-bit conversion results (not 10-bit).
    ADMUX |= (1 << ADLAR);
    
    // Set/clear the ADPS[2:0] bits in ADCSRA to select an appropriate prescalar value.
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

	// Set the ADEN bit in ADCSRA to enable the ADC module.
	ADCSRA |= (1 << ADEN);

}

uint8_t adc_sample(uint8_t channel)
{
    // Set ADC input mux bits to 'channel' value
    // Set/clear the MUX[3:0] bits in ADMUX to select the input channel as specified by the argument to the function. The correct bits to go in the MUX[3:0] bits are simply the lower four bits of the argument to the function so just use bit copying to fill these in.
	ADMUX &= ~(0x0F); 
	ADMUX |= (channel & 0x0F);

	// Set the ADSC bit in the ADCSRA register to a 1. This starts the conversion process.
	ADCSRA |= (1 << ADSC);

	// Enter a loop that tests the ADSC bit each time through the loop and exits the loop when ADSC is 0. This indicates that the conversion is now complete.
	while (ADCSRA & (1 << ADSC));
	
	// Copy the 8-bit conversion result from the ADCH register and return it to the calling program.
	return ADCH;
	
	// Convert an analog input and return the 8-bit result

}


