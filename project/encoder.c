#include <avr/io.h>
#include <avr/interrupt.h>
#include "lcd.h"
#include "project.h"

#define encA PD2
#define encB PD3

void encoder_init(void) {
    DDRD &= ~((1 << encA) | (1 << encB));
    PORTD |= ((1 << encA) | (1 << encB));
    uint8_t pin_a = 0;
    uint8_t pin_b = 0;
    if (PIND & (1 << encA)) {
        pin_a = 1;
    }
    if (PIND & (1 << encB)) {
        pin_b = 1;
    }
    enc_state = (pin_b << 1) | pin_a;
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
}

ISR(PCINT2_vect) {
    uint8_t pin_a = 0;
    uint8_t pin_b = 0;
    uint8_t changed = 0;

    if (PIND & (1 << encA)) {
        pin_a = 1;
    }
    if (PIND & (1 << encB)) {
        pin_b = 1;
    }

    uint8_t new_state = (pin_b << 1) | pin_a;

    if (enc_state != new_state) {
        if ((enc_state == 0 && new_state == 1) ||
            (enc_state == 1 && new_state == 3) ||
            (enc_state == 3 && new_state == 2) ||
            (enc_state == 2 && new_state == 0)) {
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
