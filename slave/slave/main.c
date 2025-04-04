/*
 * slave.c
 *
 * Created: 25.3.2025 14.19.19
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

// Global variable to handle door timing outside the ISR
volatile uint32_t door_timer = 0;
volatile uint8_t door_active = 0;

void TWI_init(){
	// Init the TWI Slave
	TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWIE); // Enable TWI, ACK and interrupt
	TWAR = (SLAVE_ADDRESS << 1); // Set slave address
	sei();
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
			PORTB |= (1 << DOOR_LED);  // Turn on door LED
			door_active = 1;
			door_timer = 0;  // Reset timer
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
	
	// Init TWI, will use ISR
	TWI_init();
	
	while (1){
		// Handle door timing in the main loop
		if (door_active) {
			_delay_ms(10);  // Small delay for counting
			door_timer += 10;
			
			if (door_timer >= 5000) {  // 5 seconds elapsed
				PORTB &= ~(1 << DOOR_LED);  // Turn off door LED
				door_active = 0;
			}
		}
	}
	
	return 0;
}

