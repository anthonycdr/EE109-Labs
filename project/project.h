#ifndef PROJECT_H
#define PROJECT_H

#include <stdint.h>

// Global variables
extern volatile uint8_t mode_hi;
extern volatile uint8_t b_pressed;
extern volatile uint8_t enc_state;
extern int8_t t_min;
extern int8_t t_max;
extern int16_t temp_now;
extern volatile uint8_t servo_active;
extern volatile uint16_t servo_counter;
extern volatile uint8_t send_pressed;
extern volatile uint8_t mode_toggle_pressed;
extern int8_t remote_min;
extern int8_t remote_max;
extern volatile uint8_t has_remote_data;
extern volatile uint8_t use_remote;

// Serial comm variables
extern char rx_buffer[5];
extern volatile uint8_t rx_count;
extern volatile uint8_t rx_started;
extern volatile uint8_t rx_complete;

// Function prototypes
void save_thresholds_to_eeprom(void);
void show_display(void);
void update_leds(void);

#endif
