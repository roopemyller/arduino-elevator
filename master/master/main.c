/*
 * master.c
 *
 * Created: 4.4.2025
 * Author : roope
 
###################################
##  THIS IS CODE FOR THE MASTER  ##
###################################
 */ 
#define F_CPU 16000000UL
#define SLAVE_ADDRESS 0b1010111  // The address of the slave (UNO)
#define BAUD 9600 // Baudrate for UART

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "keypad.h"
#include "lcd.h" // lcd header file made by Peter Fleury
#include <stdio.h>
#include <stdlib.h>
//#include <util/setbaud.h>

// Variables to track button states for debouncing
volatile uint8_t prev_movement_state = 1;
volatile uint8_t prev_stop_state = 1;
volatile uint8_t prev_door_state = 1;
volatile uint8_t current_floor = 0;
volatile uint8_t current_state = 0;
volatile uint8_t floors_to_travel = 0;

// Function prototypes
void setup(void);
void TWI_init(void);
uint8_t TWI_start(void);
uint8_t TWI_write(uint8_t data);
void TWI_stop(void);
void UART_init(void);
void UART_send_char(unsigned char data);
void UART_send_string(char* str);
void send_command(char cmd);
void door_open_close();

char key_str[4];
char* ptr;


int main(void) {
	
	setup();
	
	char str[20];  // Varataan 16 muistipaikkaa stringille
	// char *str_ptr;        // Luodaan apumuuttuja pointerille stringiin.
	// str_ptr = str;
	
    while (1) {
  	
		uint8_t key_input = KEYPAD_GetKey();
		
		// Tallennetaan	kaapattu keypad painallus int ja str pointer muodossa	
		if(key_input != 0){ // Jos painiketta on painettu (ei ole 0)

			key_input = key_input - '0';
			itoa(key_input, str, 20);
			UART_send_string("Pressed key: ");
			UART_send_string(str);
			UART_send_string ("\r\n");
			
		} else continue;
		
		// Define next state
		if(current_floor < key_input)	{ current_state = 1; }
		if(current_floor == key_input)	{ current_state = 2; }
		if(current_floor > key_input)	{ current_state = 3; }
		
		floors_to_travel = abs(current_floor - key_input);
		
		// Debug:
		
		UART_send_string("Current floor: ");
		itoa(current_floor, str, 20);
		UART_send_string(str);
		UART_send_string(", Floors to travel: ");
		itoa(floors_to_travel, str, 20);
		UART_send_string(str);
		UART_send_string("\r\n");
		
		switch(current_state){
			case 1: // GO UP
			
				_delay_ms(20);  // Debounce
    
				UART_send_string("Going up!\r\n");
				send_command('M');
				
				for(int i = 0; i < floors_to_travel; i++){
					current_floor++;
					// Update LCD to display current floor
					lcd_clrscr();
					//lcd_puts("Floor: %d", current_floor);
					char lcd_str[20];
					sprintf(lcd_str, "Floor: %d", current_floor);
					lcd_puts(lcd_str);
					
					
					UART_send_string("Moving to floor: ");
					itoa(current_floor, str, 10);
					UART_send_string(str);
					UART_send_string("\r\n");
					_delay_ms(500); // ELEVATOR MOVEMENT SPEED
				}
				send_command('S');
				
				// Door open-close sequence
				door_open_close();
				
				_delay_ms(200);  // Prevent multiple triggers				
				break;
			
			case 2: // SAME FLOOR -> Fault state
			
				_delay_ms(20);  // Debounce
			
				UART_send_string("Fault state.\r\n");
				lcd_clrscr();
				lcd_puts("Same Floor");
				send_command('F'); // Fault state, blink movement led 3 times
				_delay_ms(1200);
				lcd_clrscr();
				lcd_puts("Choose the floor");
				
				_delay_ms(200);  // Prevent multiple triggers			
				break;
				
			case 3: // GO DOWN
			
	            _delay_ms(20);  // Debounce
	            UART_send_string("Going down!\r\n");
				send_command('M');
				for(int i = 0; i < floors_to_travel; i++){
					current_floor--;
					// Update LCD to display current floor
					lcd_clrscr();
					//lcd_puts("Floor: %d", current_floor);
					char lcd_str[20];
					sprintf(lcd_str, "Floor: %d", current_floor);
					lcd_puts(lcd_str);
					
					UART_send_string("Moving to floor: ");
					itoa(current_floor, str, 10);
					UART_send_string(str);
					UART_send_string("\r\n");
					_delay_ms(500); // ELEVATOR MOVEMENT SPEED
				}
				
				send_command('S');
				
				// Door open-close sequence
				door_open_close();
				
	            _delay_ms(200);  // Prevent multiple triggers	
				break;
		}
		
		// Small delay
		_delay_ms(10);
	}
	return 0;
}

void door_open_close(){
	UART_send_string("Door open!\r\n");
	send_command('O');
	// Update LCD to show door opening message for 5 seconds
	lcd_clrscr();
	lcd_puts("Door is open");
	_delay_ms(5000);
	UART_send_string("Door close!\r\n");
	send_command('C');
	// Update LCD to door closed for 1 second and return to idle
	lcd_clrscr();
	lcd_puts("Door is closed");
	_delay_ms(1000);
	lcd_clrscr();
	lcd_puts("Choose the floor");
}

void setup(){
	// Initialize LCD
	lcd_init(LCD_DISP_ON);
	lcd_clrscr();
	lcd_puts("Choose the floor");
	_delay_ms(20);
	// Initialize UART for debugging
	UART_init();
	_delay_ms(20);
	UART_send_string("Elevator Master Program - NASA SERTIFIED PRODUCT\r\n");
	
	// Initialize TWI as master
	TWI_init();
	_delay_ms(20);    
	// Init keypad
	KEYPAD_Init();
	_delay_ms(20);
}

char* get_key_pressed(){
	// Read raw signal from keypad
	uint8_t key_signal = KEYPAD_GetKey();

	_delay_ms(300);

	// Check for valid key press
	if (key_signal != 0xFF) { // Assuming 0xFF means no key pressed
		char key_str[4];

		// Check if it's a numeric key (1 to 9) or special key (A, B, C, D, *, #)
		if (key_signal >= '1' && key_signal <= '9') {
			// Convert key to numeric value
			uint8_t key_value = key_signal - '0'; // Convert ASCII value to numeric value
			itoa(key_value, key_str, 10); // Convert numeric value to string

			} else {
				// For special keys, just display the character
				key_str[0] = key_signal;
				key_str[1] = '\0'; // Null terminate the string
		}
		
		ptr = key_str;
	}
	return ptr;
}

// Send a command to the slave device
void send_command(char cmd) {
	UART_send_string("Sending command: ");
	UART_send_char(cmd);
	UART_send_string("\r\n");
	
	if (TWI_start()) {
		UART_send_string("TWI started\r\n");
		
		if (TWI_write((SLAVE_ADDRESS << 1) | 0)) {  // SLA+W
			if (TWI_write(cmd)) {
				UART_send_string("Command sent successfully\r\n");
				// Flash an indicator LED on the master
				PORTB |= (1 << PB7);  // Turn on LED on pin 13
				_delay_ms(100);
				PORTB &= ~(1 << PB7); // Turn off LED
			} else {
				UART_send_string("Failed to send command\r\n");
			}
		} else {
			UART_send_string("Slave not responding\r\n");
		}		
		TWI_stop();
		UART_send_string("TWI stopped\r\n");
	} else {
		UART_send_string("TWI start failed\r\n");
	}
}

// Initialize TWI as master
void TWI_init(void) {
	// Set SCL frequency: TWBR = ((F_CPU / SCL_freq) - 16) / (2 * prescaler)
	// For 100kHz with 16MHz CPU and prescaler=1: TWBR = ((16000000/100000)-16)/2 = 72
	TWBR = 72;  // Set bit rate register
	TWSR = 0;   // Prescaler = 1
}

// Start TWI transmission
uint8_t TWI_start(void) {
	// Send START condition
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	
	// Wait for TWINT flag to be set
	while (!(TWCR & (1 << TWINT)));
	
	// Check if START was sent successfully (status code 0x08)
	if ((TWSR & 0xF8) != 0x08) {
		UART_send_string("TWI start error: 0x");
		UART_send_char('0' + ((TWSR & 0xF8) >> 4));
		UART_send_char('0' + (TWSR & 0x0F));
		UART_send_string("\r\n");
		return 0;  // Error: START not acknowledged
	}
	
	return 1;  // Success
}

// Write a byte to the TWI bus
uint8_t TWI_write(uint8_t data) {
	// Load data into TWDR register
	TWDR = data;
	
	// Start transmission
	TWCR = (1 << TWINT) | (1 << TWEN);
	
	// Wait for TWINT flag to be set
	while (!(TWCR & (1 << TWINT)));
	
	// Check status
	uint8_t status = TWSR & 0xF8;
	
	// For SLA+W, we expect 0x18 (ACK) or 0x20 (NACK)
	// For data byte, we expect 0x28 (ACK) or 0x30 (NACK)
	if (status != 0x18 && status != 0x28) {
		UART_send_string("TWI write error: 0x");
		UART_send_char('0' + (status >> 4));
		UART_send_char('0' + (status & 0x0F));
		UART_send_string("\r\n");
		return 0;  // Error
	}
	
	return 1;  // Success
}

// Stop TWI transmission
void TWI_stop(void) {
	// Send STOP condition
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
	
	// No need to wait for TWINT here - TWSTO is cleared automatically
	_delay_ms(1);  // Small delay to ensure the bus is released
}

// Initialize UART for debugging
void UART_init(void) {
	// Set baud rate to 9600 for 16MHz: UBRR = (F_CPU/(16*BAUD))-1 = 103
	UBRR0H = 0;
	UBRR0L = 103;
	
	// Enable transmitter
	UCSR0B = (1 << TXEN0);
	
	// Set frame format: 8 data bits, 1 stop bit, no parity
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Send a character through UART
void UART_send_char(unsigned char data) {
	// Wait for empty transmit buffer
	while (!(UCSR0A & (1 << UDRE0)));
	
	// Put data into buffer
	UDR0 = data;
}

// Send a string through UART
void UART_send_string(char* str) {
	// Send each character until null terminator
	while (*str) {
		UART_send_char(*str++);
	}
}