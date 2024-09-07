#include <avr/io.h>    // Required for AVR input/output
#include <util/delay.h>  // Provides functions for delays
#define TRIG_PIN PB1  // PB1 refers to pin 15
#define ECHO_PIN PB0  // PB0 refers to pin 14
#define DISTANCE_FACTOR 1.098  // Conversion factor from clock ticks to distance
#include <string.h>  // Required for string manipulation functions
#include <stdlib.h>  // Required for general utility functions

#define LED_DISPLAY_TIME 1  // Time in milliseconds to display each LED segment

void configure_timer0(void);  // Declare the timer0 configuration function

int main(void) {
    unsigned char start_time, end_time, pulse_duration;
    float calculated_distance;

    // Configure TRIG as an output pin
    DDRB = 1 << TRIG_PIN;
    // Initialize TRIG to low
    PORTB &= ~(1 << TRIG_PIN);

    configure_timer0();  // Setup timer0 for echo timing

    while(1){
        TCNT0 = 0;  // Reset the timer count to 0
        // Send a 10us pulse to trigger the sensor
        PORTB |= 1 << TRIG_PIN;
        _delay_us(10);
        PORTB &= ~(1 << TRIG_PIN);

        // Wait for the start of the echo
        while (!(PINB & (1 << ECHO_PIN)));
        start_time = TCNT0;  // Record the timer count at echo start

        // Wait for the end of the echo
        while (PINB & (1 << ECHO_PIN));
        end_time = TCNT0;  // Record the timer count at echo end

        // Calculate the pulse duration and subsequently the distance
        if (end_time > start_time){
            pulse_duration = end_time - start_time;
            calculated_distance = pulse_duration * DISTANCE_FACTOR;
            // Only process valid distance measurements
            if(calculated_distance < 200 && calculated_distance > 5){
                display_on_led(calculated_distance);
            }
        }
    }
}

// Function to display a float value on a 4-digit LED display
void display_on_led(float distance) {
    unsigned char segment_values[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x67};
    unsigned char digit1, digit2, digit3, digit4;

    // Set all PORTD pins to output for the display
    DDRD = 0xFF;
    unsigned char port_b_config = DDRB;
    port_b_config &= 0b00000011;
    port_b_config |= 0b11111100;
    DDRB = port_b_config;

    // Display each digit with persistence
    digit4 = (int)distance % 10;
    PORTD = segment_values[digit4];
    PORTB = ~(1 << 2);
    _delay_ms(LED_DISPLAY_TIME);

    digit3 = (int)(distance / 10) % 10;
    PORTD = segment_values[digit3];
    PORTB = ~(1 << 3);
    _delay_ms(LED_DISPLAY_TIME);

    digit2 = (int)(distance / 100) % 10;
    PORTD = segment_values[digit2];
    PORTB = ~(1 << 4);
    _delay_ms(LED_DISPLAY_TIME);

    digit1 = (int)(distance / 1000) % 10;
    PORTD = segment_values[digit1];
    PORTB = ~(1 << 5);
    _delay_ms(LED_DISPLAY_TIME);
}

// Configure timer0 in normal mode with a prescaler of 1024
void configure_timer0() {
    TCCR0A = 0;  // Set timer/counter mode to normal
    TCCR0B = 5;  // Set prescaler to 1024
    TCNT0 = 0;   // Initialize timer count to 0
}
