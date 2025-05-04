/*
 
 main.c
 
###################################
##  THIS IS CODE FOR THE SLAVE  ##
###################################
 */ 

#define F_CPU 16000000UL
#define MOVEMENT_LED PB1 // LED for elevator movement (pin 9 on UNO)
#define DOOR_LED PB2 // LED for door opening (pin 10 on UNO)
#define BUZZER PB3 // Buzzer (pin 11 on UNO)

#define SLAVE_ADDRESS 0b1010111

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>

volatile uint8_t melody_active = 1; // flag for melody
volatile uint8_t play_emergency = 0; // flag for emergency

// Function for buzzer to play a frequency
void buzzer_play(uint16_t frequency) {
	// Set BUZZER/PB3 (Digital pin 11) as output
	DDRB |= (1 << BUZZER);

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
	OCR2A = (F_CPU/(2 * 128 * frequency)) - 1;
}

// Function to set buzzer to stop
void buzzer_stop() {
	// Disable Timer2
	// Reset the timers again.
	TCCR2A = 0;
	TCCR2B = 0;

	// Pull the pin low just in case
	PORTB &= ~(1 << BUZZER);
}

// Function to blink LED times, times
void blink(uint8_t led_pin, int times){
	
	// For loop, each time LED is first turned ON then OFF
	for (int i = 0; i < times; i++){
		PORTB |= (1 << led_pin); // Turn ON LED
		_delay_ms(200);
		PORTB &= ~(1 << led_pin); // Turn OFF LED
		_delay_ms(200);
	}
}

// Function to play emergency melody
void play_emergency_melody() {
	
	// Blink movement led for fun effect :)
	blink(MOVEMENT_LED, 5);

	// frequenzies for buzzer
	uint16_t freqs[] = {2500, 2000, 3250, 2750};
	
	// for loop to go through the frequenzies
	for (int i = 0; i < 4; i++) {
		buzzer_play(freqs[i]);
		_delay_ms(250);
	}
	buzzer_stop();
}

// Function to receive command from master
void TWI_receive_command() {
	
	if (TWCR & (1 << TWINT)) {
		
		// Get the status code	
		uint8_t status = TWSR & 0xF8;
		
		if(status == 0x60 || status == 0x68) {
			// SLA+W received, ACK returned
			TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWINT); // Prepare to receive		
		}
		else if((status == 0x80) || (status == 0x90)) {
			// Data received, ACK returned
			
			// Get the sent command
			char command = TWDR;
			
			// Check what command sent
			if(command == 'M'){ // Movement, Turn ON movement LED
				PORTB |= (1 << MOVEMENT_LED); 
			} else if(command == 'S'){ // Stop, Turn OFF movement LED
				PORTB &= ~(1 << MOVEMENT_LED); 
			} else if (command == 'O') { // Open door, Turn ON door LED
				PORTB |= (1 << DOOR_LED);  
			} else if (command == 'C'){ // Close door, Turn OFF door LED
				PORTB &= ~(1 << DOOR_LED); 
			} else if (command == 'F'){ // Fault state, blink movement LED 3 times
				blink(MOVEMENT_LED, 3);
			} else if (command == 'E') { // Emergency state, blink movement LED 3 times
				blink(MOVEMENT_LED, 3);
			} else if (command == 'P') { // Play Emergency Melody, Turn ON door LED (open door)
				PORTB |= (1 << DOOR_LED);  
				play_emergency = 1;
			} else if (command == 'X') { // Stop Emergency Melody, close the door
				play_emergency = 0;
				PORTB &= ~(1 << DOOR_LED); // Automatically close the door
			}
			
			// Prepare for next byte
			TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWINT); 
		}
		else {
			// Default - send ACK and continue
			TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWINT);
		}
	}
}

int main(void){
	
	// Set LED pins as outputs
	DDRB |= (1 << MOVEMENT_LED) | (1 << DOOR_LED);
	
	// TWI Init
	TWAR = (SLAVE_ADDRESS << 1);
	TWCR = (1 << TWEN) | (1 << TWEA);
	
	// Inside main while loop
	// Receive a command from TWI
	// If play_emergency activated, play emergency
	while (1){
		TWI_receive_command();	
		if (play_emergency) {
			play_emergency_melody();
		}
	}
	return 0;
}