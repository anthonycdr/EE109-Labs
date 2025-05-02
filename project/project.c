#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "lcd.h"
#include "ds18b20.h"

#define ENCODER_A PD2
#define ENCODER_B PD3
#define BUTTON    PB5
#define RED_LED   PC4
#define GREEN_LED PC3
#define BLUE_LED  PC2
#define SERVO_PIN PB3
#define SEND_BUTTON PC0      
#define LOCAL_REMOTE_BUTTON PC5  

// function prototypes
void setup(void);
void read_thresholds_from_eeprom(void);
void save_thresholds_to_eeprom(void);
void show_display(void);
void get_temp(void);
void timer1_init(void);
void timer2_init(void);
void update_leds(void);
void encoder_init(void);
void button_init(void);
void send_button_init(void);
void local_remote_button_init(void);
void usart_init(void);
void send_thresholds(void);
void buffer_to_thresholds(void);
void update_active_thresholds(void);

volatile uint8_t mode_hi = 0;
volatile uint8_t b_pressed = 0;
volatile uint8_t enc_state = 0;
int8_t t_min = 65;
int8_t t_max = 75;            
int16_t temp_now = 0;
volatile uint8_t servo_active = 0;
volatile uint16_t servo_counter = 0;
volatile uint8_t send_pressed = 0;
volatile uint8_t mode_toggle_pressed = 0;

// remote threshold variables
int8_t remote_min = 65;
int8_t remote_max = 75;
volatile uint8_t has_remote_data = 0;
volatile uint8_t use_remote = 0;

// serial comm variables
char rx_buffer[5];
volatile uint8_t rx_count = 0;
volatile uint8_t rx_started = 0;
volatile uint8_t rx_complete = 0;

#define LOW_THRESHOLD_ADDR  100
#define HIGH_THRESHOLD_ADDR 101

void setup(void) {
    lcd_init();
    encoder_init();
    button_init();
    send_button_init();
    local_remote_button_init();
    usart_init();
    
    // enable tristate buffer for RX line
    DDRB |= (1 << PB4);
    PORTB &= ~(1 << PB4);
    
    DDRC |= (1 << RED_LED) | (1 << GREEN_LED) | (1 << BLUE_LED);
    PORTC |= (1 << RED_LED) | (1 << GREEN_LED) | (1 << BLUE_LED);
    timer1_init();
    timer2_init();

    if (ds_init() == 0) {
        lcd_moveto(0, 0);
        lcd_stringout("Can't Detect Sensor");
        while (1);
    }

    lcd_writecommand(1);
    lcd_moveto(0, 0);
    lcd_stringout("Anthony");
    lcd_moveto(1, 0);
    lcd_stringout("EE109 Project");
    _delay_ms(1000);
    lcd_writecommand(1);

    read_thresholds_from_eeprom();

    show_display();
    ds_convert();
    sei();
}


void read_thresholds_from_eeprom(void) {
    int8_t stored_t_min = eeprom_read_byte((void *)LOW_THRESHOLD_ADDR);
    int8_t stored_t_max = eeprom_read_byte((void *)HIGH_THRESHOLD_ADDR);

    if (stored_t_min >= 50 && stored_t_min <= 90) {
        t_min = stored_t_min;
    }
    if (stored_t_max >= 50 && stored_t_max <= 90) {
        t_max = stored_t_max;
    }
    if (t_min > t_max) {
        t_min = t_max;
    }
}

void save_thresholds_to_eeprom(void) {
    eeprom_update_byte((void *)LOW_THRESHOLD_ADDR, t_min);
    eeprom_update_byte((void *)HIGH_THRESHOLD_ADDR, t_max);
}

void show_display(void) {
    char line[16];
    
    lcd_writecommand(1);
    lcd_moveto(0, 0);
    
    if (!use_remote) {
        // show selection indicator for local mode
        if (mode_hi) {
            snprintf(line, 17, " L=%2d>H=%2d< %d.%d", t_min, t_max, temp_now / 10, temp_now % 10);
        } else {
            snprintf(line, 17, ">L=%2d H=%2d< %d.%d", t_min, t_max, temp_now / 10, temp_now % 10);
        }
    } else {
        snprintf(line, 17, " L=%2d H=%2d  %d.%d", t_min, t_max, temp_now / 10, temp_now % 10);
    }
    lcd_stringout(line);
    
    lcd_moveto(1, 0);
    
    if (has_remote_data) {
        if (use_remote) {
            if (mode_hi) {
                snprintf(line, 17, " L=%2d>H=%2d<", remote_min, remote_max);
            } else {
                snprintf(line, 17, ">L=%2d H=%2d<", remote_min, remote_max);
            }
        } else {
            snprintf(line, 17, " L=%2d H=%2d ", remote_min, remote_max);
        }
        lcd_stringout(line);
    } else {
        lcd_stringout("                ");
    }
}

void get_temp(void) {
    unsigned char t[2];
    if (ds_temp(t)) {
        int16_t raw = ((int8_t)t[1] << 8) | t[0];
        int32_t c = ((int32_t)raw * 100) / 16;  
        int16_t f = (c * 9) / 50 + 320;      
        temp_now = f;
        show_display();
        update_leds();
        ds_convert();
    }
}

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
    DDRB |= (1 << SERVO_PIN);
    OCR2A = 12;
    TIMSK2 = (1 << TOIE2);
}

void update_leds(void) {
    // get current active thresholds
    int8_t active_min, active_max;
    static int8_t prev_condition = 0; // 0 = initial, -1 = too cold, 1 = too hot
    int8_t curr_condition = 0;
    
    if (use_remote && has_remote_data) {
        active_min = remote_min;
        active_max = remote_max;
    } else {
        active_min = t_min;
        active_max = t_max;
    }
    
    int8_t int_temp = temp_now / 10;
    
    if (int_temp < active_min) {
        // blue LED for too cold
        PORTC |= (1 << RED_LED) | (1 << GREEN_LED);
        PORTC &= ~(1 << BLUE_LED);

        OCR1B = OCR1A;
        
        curr_condition = -1; // too cold
        
        // Check if transitioning from too hot to too cold
        if (prev_condition == 1) {
            // Reset and restart servo
            servo_active = 1;
            servo_counter = 0;
        } else if (!servo_active) {
            // Normal activation if servo not already active
            servo_active = 1;
            servo_counter = 0;
        }
    } 
    else if (int_temp > active_max) {
        // red LED for too hot
        PORTC |= (1 << GREEN_LED) | (1 << BLUE_LED);
        PORTC &= ~(1 << RED_LED);
        
        OCR1B = OCR1A;
        
        curr_condition = 1; // too hot
        
        if (prev_condition == -1) {
            servo_active = 1;
            servo_counter = 0;
        } else if (!servo_active) {
            servo_active = 1;
            servo_counter = 0;
        }
    } 
    else {
        // green LED with PWM for just right
        PORTC |= (1 << RED_LED) | (1 << BLUE_LED);
        PORTC &= ~(1 << GREEN_LED);
        uint16_t pwm_val;
        
        if (active_min == active_max) {
            pwm_val = OCR1A / 2;  
        } 
        else {
            pwm_val = OCR1A - ((int_temp - active_min) * (OCR1A / 2)) / (active_max - active_min);
        }
        
        OCR1B = pwm_val;  
        
        curr_condition = 0; // normal
        servo_active = 0;
    }
    
    // Store current condition for next comparison
    prev_condition = curr_condition;
}
void encoder_init(void) {
    DDRD &= ~((1 << ENCODER_A) | (1 << ENCODER_B));
    PORTD |= ((1 << ENCODER_A) | (1 << ENCODER_B));
    uint8_t pin_a = 0;
    uint8_t pin_b = 0;
    if (PIND & (1 << ENCODER_A)) {
        pin_a = 1;
    }
    if (PIND & (1 << ENCODER_B)) {
        pin_b = 1;
    }
    enc_state = (pin_b << 1) | pin_a;
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
}

void button_init(void) {
    DDRB &= ~(1 << BUTTON);
    PORTB |= (1 << BUTTON);
    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT5);
}

void send_button_init(void) {
    DDRC &= ~(1 << SEND_BUTTON);
    
    PCICR |= (1 << PCIE1);
    PCMSK1 |= (1 << PCINT8);
}

void local_remote_button_init(void) {
    DDRC &= ~(1 << LOCAL_REMOTE_BUTTON);
    PORTC |= (1 << LOCAL_REMOTE_BUTTON);
    PCMSK1 |= (1 << PCINT13);
}

void usart_init(void) {
    // 9600 baud @ 16MHz
    UBRR0H = 0;
    UBRR0L = 103;
    
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void send_thresholds(void) {
    // format: <LLHH>
    while ((UCSR0A & (1 << UDRE0)) == 0) { }
    UDR0 = '<';
    
    while ((UCSR0A & (1 << UDRE0)) == 0) { }
    UDR0 = '0' + (t_min / 10);
    
    while ((UCSR0A & (1 << UDRE0)) == 0) { }
    UDR0 = '0' + (t_min % 10);
    
    while ((UCSR0A & (1 << UDRE0)) == 0) { }
    UDR0 = '0' + (t_max / 10);
    
    while ((UCSR0A & (1 << UDRE0)) == 0) { }
    UDR0 = '0' + (t_max % 10);
    
    while ((UCSR0A & (1 << UDRE0)) == 0) { }
    UDR0 = '>';
}

void buffer_to_thresholds(void) {
    int low, high;
    
    sscanf(rx_buffer, "%2d%2d", &low, &high);
    
    if (low >= 50 && low <= 90 && high >= 50 && high <= 90) {
        remote_min = low;
        remote_max = high;
        
        if (remote_min > remote_max) {
            remote_min = remote_max;
        }
        
        has_remote_data = 1;
        
        show_display();
        
        if (use_remote) {
            update_leds();
        }
    }
}

int main(void) {
    setup();
    
    show_display();
    
    while (1) {
        if (send_pressed) {
            send_thresholds();
            send_pressed = 0;
        }
        
        if (mode_toggle_pressed) {
            use_remote = !use_remote;
            
            show_display();
            update_leds();
            
            mode_toggle_pressed = 0;
        }
        
        if (rx_complete) {
            buffer_to_thresholds();
            rx_complete = 0;
        }
        
        if (b_pressed) {
            mode_hi = !mode_hi;
            b_pressed = 0;
            show_display();  
        }
        
        get_temp(); 
        _delay_ms(100);
    }
    return 0;  
}

ISR(PCINT2_vect) {
    uint8_t pin_a = 0;
    uint8_t pin_b = 0;
    uint8_t changed = 0;
    
    if (PIND & (1 << ENCODER_A)) {
        pin_a = 1;
    }
    if (PIND & (1 << ENCODER_B)) {
        pin_b = 1;
    }
    
    uint8_t new_state = (pin_b << 1) | pin_a;
    
    if (enc_state != new_state) {
        if ((enc_state == 0 && new_state == 1) ||
            (enc_state == 1 && new_state == 3) ||
            (enc_state == 3 && new_state == 2) ||
            (enc_state == 2 && new_state == 0)) {
            // clockwise - increase
            if (!use_remote) {
                if (mode_hi) {
                    if (t_max < 90) {
                        t_max++;
                        changed = 1;
                    }
                } else {
                    if (t_min < 90 && t_min < t_max) {
                        t_min++;
                        changed = 1;
                    }
                }
            } else if (has_remote_data) {
                if (mode_hi) {
                    if (remote_max < 90) {
                        remote_max++;
                        changed = 1;
                    }
                } else {
                    if (remote_min < 90 && remote_min < remote_max) {
                        remote_min++;
                        changed = 1;
                    }
                }
            }
        } else if ((enc_state == 0 && new_state == 2) ||
                  (enc_state == 2 && new_state == 3) ||
                  (enc_state == 3 && new_state == 1) ||
                  (enc_state == 1 && new_state == 0)) {
            // counter-clockwise - decrease
            if (!use_remote) {
                if (mode_hi) {
                    if (t_max > 50 && t_max > t_min) {
                        t_max--;
                        changed = 1;
                    }
                } else {
                    if (t_min > 50) {
                        t_min--;
                        changed = 1;
                    }
                }
            } else if (has_remote_data) {
                if (mode_hi) {
                    if (remote_max > 50 && remote_max > remote_min) {
                        remote_max--;
                        changed = 1;
                    }
                } else {
                    if (remote_min > 50) {
                        remote_min--;
                        changed = 1;
                    }
                }
            }
        }
        
        enc_state = new_state;
        
        if (changed) {
            if (!use_remote) {
                save_thresholds_to_eeprom();
            }
            
            show_display();
            update_leds();
        }
    }
}

ISR(PCINT0_vect) {
    static uint8_t prev_btn = 1;
    uint8_t curr_btn = 0;
    if (PINB & (1 << BUTTON)) {
        curr_btn = 1;
    }
    if (prev_btn == 1 && curr_btn == 0) {
        _delay_ms(10);
        if ((PINB & (1 << BUTTON)) == 0) {
            b_pressed = 1;
        }
    }
    prev_btn = curr_btn;
}

ISR(PCINT1_vect) {
    // send button (PC0)
    static uint8_t prev_send = 1;
    uint8_t curr_send = 0;
    
    if (PINC & (1 << SEND_BUTTON)) {
        curr_send = 1;
    }
    
    if (prev_send == 1 && curr_send == 0) {
        _delay_ms(5);
        if ((PINC & (1 << SEND_BUTTON)) == 0) {
            send_pressed = 1;
        }
    }
    prev_send = curr_send;
    
    // mode toggle button (PC5)
    static uint8_t prev_mode = 1;
    uint8_t curr_mode = 0;
    
    if (PINC & (1 << LOCAL_REMOTE_BUTTON)) {
        curr_mode = 1;
    }
    
    if (prev_mode == 1 && curr_mode == 0) {
        _delay_ms(5);
        if ((PINC & (1 << LOCAL_REMOTE_BUTTON)) == 0) {
            if (has_remote_data) {
                mode_toggle_pressed = 1;
            }
        }
    }
    prev_mode = curr_mode;
}
ISR(USART_RX_vect) {
    char c = UDR0;
    
    if (c == '<') {
        rx_started = 1;
        rx_count = 0;
        rx_complete = 0;
        return;
    }
    
    if (!rx_started) {
        return;
    }
    
    if (c == '>') {
        if (rx_count == 4) {
            rx_buffer[rx_count] = '\0';
            rx_complete = 1;
        }
        rx_started = 0;
        return;
    }
    
    if (c >= '0' && c <= '9') {
        if (rx_count < 4) {
            rx_buffer[rx_count] = c;
            rx_count++;
        } else {
            rx_started = 0;
        }
    } else {
        rx_started = 0;
    }
}

ISR(TIMER1_OVF_vect) {
    PORTC |= (1 << GREEN_LED);
}

ISR(TIMER1_COMPB_vect) {
    PORTC &= ~(1 << GREEN_LED);
}

ISR(TIMER2_OVF_vect) {
    const uint8_t MIN_POS = 12;
    const uint8_t MAX_POS = 35;
    const uint16_t SWEEP_TIME = 610;

    if (servo_active) {
        servo_counter++;
        if (servo_counter <= SWEEP_TIME) {
            uint8_t pos = MIN_POS + ((uint32_t)servo_counter * (MAX_POS - MIN_POS)) / SWEEP_TIME;
            OCR2A = pos;
        } else {
            servo_counter = SWEEP_TIME;
        }
    } else {
        OCR2A = MIN_POS;
        servo_counter = 0;
    }
}
