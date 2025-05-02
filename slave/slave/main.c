/*
 * slave.c
 * Author : roope
 
###################################
##  THIS IS CODE FOR THE SLAVE  ##
###################################
 */ 

#define F_CPU 16000000UL
#define MOVEMENT_LED PB1 // LED for elevator movement (pin 9 on UNO)
#define DOOR_LED PB2 // LED for door opening (pin 10 on UNO)
#define BUZZER_PIN PB3 // Buzzer for melody (pin 11 on UNO)

#define SLAVE_ADDRESS 0b1010111

// Define melody notes and durations (adjust as needed)
#define NOTE_C4 262
#define NOTE_D4 294
#define NOTE_E4 330
#define NOTE_F4 349

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>

volatile uint8_t melody_active = 1;

// Function to play a single tone on the buzzer
void buzzer_play(uint16_t frequency, uint8_t timer_config) {
    TCCR2A = (1 << WGM21) | (1 << COM2A0); // CTC mode
    TCCR2B = timer_config; // Set prescaler
    OCR2A = F_CPU / (2 * frequency * (timer_config & 0x07)) - 1; // Calculate OCR2A
    DDRD |= (1 << BUZZER_PIN);
}

// Function to stop the buzzer
void buzzer_stop() {
    TCCR2A &= ~((1 << COM2A1) | (1 << COM2A0) | (1 << WGM21)); // Disable compare output and CTC
    TCCR2B = 0;
    DDRD &= ~(1 << BUZZER_PIN);
}

void TWI_init(){
	// Init the TWI Slave
	TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWIE); // Enable TWI, ACK and interrupt
	TWAR = (SLAVE_ADDRESS << 1); // Set slave address
	sei();
}

void blink(uint8_t led_pin, int times){
	for (int i = 0; i < times; i++){
		PORTB |= (1 << led_pin); // Turn ON LED
		_delay_ms(200);
		PORTB &= ~(1 << led_pin); // Turn OFF LED
		_delay_ms(200);		
	}
}

void play_emergency_melody() {
    melody_active = 1;
    while (melody_active) {
        buzzer_play(NOTE_C4, (1 << CS21)); // Example: C4 with prescaler 8
        buzzer_play(NOTE_D4, (1 << CS21));
        buzzer_play(NOTE_E4, (1 << CS21));
        buzzer_play(NOTE_F4, (1 << CS21));
        // Loop continues until melody_active is set to 0
    }
    buzzer_stop();
}

ISR(TWI_vect){
	// Get status code
	uint8_t twi_status = (TWSR & 0xF8);
	
	if(twi_status == 0x60 || twi_status == 0x68) {
		// SLA+W received, ACK returned
		TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
	}
	else if((twi_status == 0x80) || (twi_status == 0x90)) {
		// Data received, ACK returned
		char command = TWDR;
		
		if(command == 'M'){
			PORTB |= (1 << MOVEMENT_LED); // Turn ON movement LED
		} else if(command == 'S'){
			PORTB &= ~(1 << MOVEMENT_LED); // Turn OFF movement LED
		} else if (command == 'O') {
			PORTB |= (1 << DOOR_LED);  // Turn ON door LED
		} else if (command == 'C'){
			PORTB &= ~(1 << DOOR_LED); // Turn OFF door LED
		} else if (command == 'F'){ // Fault state, blink movement LED 3 times
			blink(MOVEMENT_LED, 3);
        } else if (command == 'E') { // Emergency state
            blink(MOVEMENT_LED, 3);
        } else if (command == 'O') {
            PORTB |= (1 << DOOR_LED);  // Turn ON door LED (open door)
            play_emergency_melody();   // Start the infinite melody loop
        } else if (command == 'X') {
            melody_active = 0;          // Stop the melody loop
            PORTB &= ~(1 << DOOR_LED); // Automatically close the door
        }
  		
		TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
	}
	else {
		// Default - send ACK and continue
		TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
	}
}

int main(void){
	
	// Set LED pins as outputs
	DDRB |= (1 << MOVEMENT_LED) | (1 << DOOR_LED);
    
    // Set Buzzer pin as output
    DDRD |= (1 << BUZZER_PIN);
	
	// Init TWI, will use ISR
	TWI_init();
	
	while (1){
		
	}
	
	return 0;
}

