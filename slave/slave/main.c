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
#define BUZZER_PIN PB3

#define SLAVE_ADDRESS 0b1010111

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>

void TWI_init(){
	// Init the TWI Slave
	TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWIE); // Enable TWI, ACK and interrupt
	TWAR = (SLAVE_ADDRESS << 1); // Set slave address
	sei();
}

void buzzer_init(){
	DDRB |= (1 << BUZZER_PIN); // Set buzzer pin as output

	TCNT1 = 0;        // Reset counter
	TCCR1B = 0;       // Stop timer

	TCCR1A |= (1 << COM1A0); // Toggle OC1A on Compare Match
	TCCR1A |= (1 << WGM10);  // Mode 9, Phase and frequency correct PWM
	TCCR1B |= (1 << WGM13);  // Mode 9 continued

	TIMSK1 |= (1 << OCIE1A); // Enable Timer1 Compare A interrupt
}

void blink(uint8_t led_pin, int times){
	for (int i = 0; i < times; i++){
		PORTB |= (1 << led_pin); // Turn ON LED
		_delay_ms(200);
		PORTB &= ~(1 << led_pin); // Turn OFF LED
		_delay_ms(200);		
	}
}

void buzzer_play(uint16_t top_value, uint8_t prescaler_bits){
	OCR1A = top_value;
	TCCR1B |= prescaler_bits;
}

void buzzer_stop(){
	TCCR1B &= ~((1 << CS10) | (1 << CS11) | (1 << CS12)); // Stop timer
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
		} else if (command == 'E') { // Emergency state, blink
			blink(MOVEMENT_LED, 3);
		} else if (command == 'M') { // Melody start
			buzzer_play(30534, (1 << CS10)); // C4
			_delay_ms(300);
			buzzer_play(38461, (1 << CS11)); // E3
			_delay_ms(300);
			buzzer_play(61156, (1 << CS11)); // F2
			_delay_ms(300);
			buzzer_play(13881, (1 << CS11) | (1 << CS10)); // D0
			_delay_ms(300);
		} else if (command == 'X') {
			buzzer_stop();
		}// Melody end
  		
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

	// Init buzzer
	buzzer_init();
	
	while (1){
		
	}
	
	return 0;
}

