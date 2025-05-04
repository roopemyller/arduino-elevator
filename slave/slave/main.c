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

#define SLAVE_ADDRESS 0b1010111

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>

volatile uint8_t melody_active = 1;

volatile uint8_t play_emergency = 0;

void buzzer_play(uint16_t frequency) {
	// Set PB3 (Digital pin 11) as output
	DDRB |= (1 << PB3);

	// Clear previous TIMER settings just in case
	// We selected the timer 2 because it is 8-bit, like the example in Exc5
	TCCR2A = 0; // This controls the comparison stuff
	TCCR2B = 0; // This controls the waveform generation

	// Set CTC (clear timer on compare) mode (WGM21, waveform generation mode) and toggle OC2A (output compare 2A) on compare match (COM2A0)
	// Basically counts up to OC2A register (this is initialized later in the code)
	TCCR2A |= (1 << WGM21) | (1 << COM2A0);

	// This sets the 8-bit timer (Timer 2) prescaler as 128, so CS22 and CS20 (clock select) uses the system clock divided by the prescaler 128 (1,0,1)
	TCCR2B |= (1 << CS22) | (1 << CS20);

	// Calculate and set compare value
	// OCR2A = (F_CPU / (2 * Prescaler * Frequency)) - 1
	OCR2A = (F_CPU/(2 * 128 * frequency)) - 1;
}

void buzzer_stop() {
	// Disable Timer2
	// Reset the timers again.
	TCCR2A = 0;
	TCCR2B = 0;

	// Pull the pin low just in case
	PORTB &= ~(1 << PB3);
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
	blink(MOVEMENT_LED, 5);

	uint16_t tones[] = {2500, 2000, 3250, 2750};
	for (int i = 0; i < 4; i++) {
		buzzer_play(tones[i]);
		_delay_ms(250);
	}

	buzzer_stop();
}

void TWI_poll_receive() {
	
	if (TWCR & (1 << TWINT)) {
		
		uint8_t status = TWSR & 0xF8;
		
		if(status == 0x60 || status == 0x68) {
			// SLA+W received, ACK returned
			TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWINT); // Prepare to receive		
		}
		else if((status == 0x80) || (status == 0x90)) {
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
			} else if (command == 'P') {
				PORTB |= (1 << DOOR_LED);  // Turn ON door LED (open door)
				play_emergency = 1;
			} else if (command == 'X') {
				play_emergency = 0;
				PORTB &= ~(1 << DOOR_LED); // Automatically close the door
			}
			
			TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWINT); // Prepare for next byte
		}
		else {
			// Default - send ACK and continue
			TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWINT); // Safe default
		}
	}
}

int main(void){
	
	// Set LED pins as outputs
	DDRB |= (1 << MOVEMENT_LED) | (1 << DOOR_LED);
	
	// TWI Init
	TWAR = (SLAVE_ADDRESS << 1);
	TWCR = (1 << TWEN) | (1 << TWEA);
	
	while (1){
		TWI_poll_receive();
		
		if (play_emergency) {
			play_emergency_melody();
			// play_emergency = 0;
		}
		
	}
	
	return 0;
}