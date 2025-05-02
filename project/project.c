#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "lcd.h"
#include "ds18b20.h"
#include "project.h"

#define encA PD2 
#define encB PD3 
#define h_l_btn   PB5 
#define red  PC4 
#define green PC3 
#define blue  PC2 
#define servo PB3 
#define sendbtn PC0       
#define l_r_btn PC5

// Function prototypes
void setup(void);
void read_thresholds_from_eeprom(void);
void save_thresholds_to_eeprom(void);
void show_display(void);
void get_temp(void);
void button_init(void);
void send_button_init(void);
void local_remote_button_init(void);
void usart_init(void);
void send_thresholds(void);
void buffer_to_thresholds(void);

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
    
    DDRC |= (1 << red) | (1 << green) | (1 << blue);
    PORTC |= (1 << red) | (1 << green) | (1 << blue);
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

void button_init(void) {
    DDRB &= ~(1 << h_l_btn);
    PORTB |= (1 << h_l_btn);
    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT5);
}

void send_button_init(void) {
    DDRC &= ~(1 << sendbtn);
    PCICR |= (1 << PCIE1);
    PCMSK1 |= (1 << PCINT8);
}

void local_remote_button_init(void) {
    DDRC &= ~(1 << l_r_btn);
    PORTC |= (1 << l_r_btn);
    PCMSK1 |= (1 << PCINT13);
}

void usart_init(void) {
    UBRR0H = 0;
    UBRR0L = 103;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void send_thresholds(void) {
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

ISR(PCINT0_vect) {
    static uint8_t prev_btn = 1;
    uint8_t curr_btn = 0;
    if (PINB & (1 << h_l_btn)) {
        curr_btn = 1;
    }
    if (prev_btn == 1 && curr_btn == 0) {
        _delay_ms(10);
        if ((PINB & (1 << h_l_btn)) == 0) {
            b_pressed = 1;
        }
    }
    prev_btn = curr_btn;
}

ISR(PCINT1_vect) {
    static uint8_t prev_send = 1;
    uint8_t curr_send = 0;
    if (PINC & (1 << sendbtn)) {
        curr_send = 1;
    }
    if (prev_send == 1 && curr_send == 0) {
        _delay_ms(5);
        if ((PINC & (1 << sendbtn)) == 0) {
            send_pressed = 1;
        }
    }
    prev_send = curr_send;
    
    static uint8_t prev_mode = 1;
    uint8_t curr_mode = 0;
    if (PINC & (1 << l_r_btn)) {
        curr_mode = 1;
    }
    if (prev_mode == 1 && curr_mode == 0) {
        _delay_ms(5);
        if ((PINC & (1 << l_r_btn)) == 0) {
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
