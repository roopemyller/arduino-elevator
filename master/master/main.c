#define F_CPU 16000000UL
#define SLAVE_ADDRESS 0b1010111
#define BAUD 9600

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "keypad.h"
#include "lcd.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

// Pin definitions
#define EMERGENCY_BUTTON PD2

// Variables to track elevator state
volatile uint8_t current_floor = 0;
volatile uint8_t current_state = 0;
volatile uint8_t floors_to_travel = 0;
volatile bool elevator_moving = false; // Add this flag

// Variable for checking emergency state
volatile bool emergency_stop_requested = false;
volatile bool emergency_sequence_active = false;

// Function prototypes
void setup(void);
uint8_t get_key_pressed();
uint8_t get_any_key_press();
void TWI_init(void);
uint8_t TWI_start(void);
uint8_t TWI_write(uint8_t data);
void TWI_stop(void);
void UART_init(void);
void UART_send_char(unsigned char data);
void UART_send_string(char* str);
void send_command(char cmd);
void door_open_close();
void LCD_top(const char* msg);
void LCD_bottom(int floor);
void clear_LCD_line(uint8_t row);
void emergency_mode();
void stop_emergency_melody();
void start_movement(); // Added
void stop_movement();  // Added

char key_str[4];
char* ptr;

uint8_t key_input = 0;

// Trigger emergency mode function when button pressed (Falling edge interrupt)
ISR(INT2_vect) {
	if (elevator_moving) { // Check if the elevator is moving
		emergency_stop_requested = true;
	}
}

int main(void) {
	setup();

	char str[30];

	while (1) {
		if (emergency_stop_requested) {
			cli();
			emergency_stop_requested = false;
			emergency_mode();
			sei();
			continue;
		}

		if (!emergency_sequence_active) {
			key_input = get_key_pressed();

			itoa(key_input, str, 10);
			UART_send_string("Pressed key: ");
			UART_send_string(str);
			UART_send_string("\r\n");

			// Define next state
			if (current_floor < key_input) {
				current_state = 1;
			}
			if (current_floor == key_input) {
				current_state = 2;
			}
			if (current_floor > key_input) {
				current_state = 3;
			}

			floors_to_travel = abs(current_floor - key_input);

			// Debug:
			UART_send_string("Current floor: ");
			itoa(current_floor, str, 10);
			UART_send_string(str);
			UART_send_string(", Floors to travel: ");
			itoa(floors_to_travel, str, 10);
			UART_send_string("\r\n");

			switch (current_state) {
				case 1: // GO UP
				_delay_ms(20);
				UART_send_string("Going up!\r\n");
				send_command('M');
				LCD_top("Going up");
				elevator_moving = true; // Set moving flag
				for (int i = 0; i < floors_to_travel; i++) {
					if (emergency_stop_requested) {
						cli();
						emergency_stop_requested = false;
						emergency_mode();
						sei();
						goto idle_state;
					}
					current_floor++;
					UART_send_string("Moving to floor: ");
					itoa(current_floor, str, 10);
					UART_send_string(str);
					UART_send_string("\r\n");
					LCD_bottom(current_floor);
					_delay_ms(100);
				}
				send_command('S');
				elevator_moving = false; // Clear moving flag
				door_open_close();
				_delay_ms(200);
				break;

				case 2: // SAME FLOOR -> Fault state
				_delay_ms(20);
				UART_send_string("Fault state.\r\n");
				LCD_top("Same Floor");
				send_command('F');
				_delay_ms(1200);
				LCD_top("Choose the floor");
				_delay_ms(200);
				break;

				case 3: // GO DOWN
				_delay_ms(20);
				UART_send_string("Going down!\r\n");
				send_command('M');
				LCD_top("Going down");
				elevator_moving = true; // Set moving flag
				for (int i = 0; i < floors_to_travel; i++) {
					if (emergency_stop_requested) {
						cli();
						emergency_stop_requested = false;
						emergency_mode();
						sei();
						goto idle_state;
					}
					current_floor--;
					UART_send_string("Moving to floor: ");
					itoa(current_floor, str, 10);
					UART_send_string(str);
					UART_send_string("\r\n");
					LCD_bottom(current_floor);
					_delay_ms(100);
				}
				send_command('S');
				elevator_moving = false; // Clear moving flag
				door_open_close();
				_delay_ms(200);
				break;
			}
			} else {
			uint8_t stop_key = get_any_key_press();
			UART_send_string("Key pressed during emergency: ");
			itoa(stop_key, str, 10);
			UART_send_string(str);
			UART_send_string("\r\n");
			stop_emergency_melody();
			emergency_sequence_active = false;
		}

		idle_state:
		_delay_ms(10);
	}
	return 0;
}

void door_open_close() {
	UART_send_string("Door open!\r\n");
	send_command('O');
	LCD_top("Door is open");
	_delay_ms(5000);

	UART_send_string("Door close!\r\n");
	send_command('C');
	LCD_top("Door is closed");
	_delay_ms(1000);
	LCD_top("Choose the floor");
}

void setup(){
	// Initialize LCD
	lcd_init(LCD_DISP_ON);
	lcd_clrscr();
	LCD_top("Choose the floor");
	LCD_bottom(0);
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

	// Initialize interrupts for emergency button
	EIMSK |= (1 << INT2);      // Enable external interrupt INT2
	EICRA |= (1 << ISC21);      // Falling edge generates interrupt request
	EICRA &= ~(1 << ISC20);
	DDRD &= ~(1 << EMERGENCY_BUTTON); // PD2 (INT2) as input
	PORTD |= (1 << EMERGENCY_BUTTON);  // Enable pull-up resistor on PD2
	sei();                      // Enable global interrupts
}

// LCD display top row updates
void LCD_top(const char* msg) {
	lcd_gotoxy(0,0);
	clear_LCD_line(0);
	lcd_puts(msg);
}

// LCD display bottom row updates
void LCD_bottom(int floor){
	char str[20];
	sprintf(str, "Floor now: %d", floor);
	lcd_gotoxy(0,1);
	clear_LCD_line(1);
	lcd_puts(str);
}

// Clear one line (row) from LCD
void clear_LCD_line(uint8_t row){
	lcd_gotoxy(0, row);
	for(uint8_t i=0; i < 16; i++)
	lcd_putc(' ');
	lcd_gotoxy(0,row);
}

uint8_t get_key_pressed(){


	uint8_t keypad_ready = 0;
	uint8_t key_index = 0;
	uint8_t floor_value = 0;
	
	char key[16];
	
	UART_send_string("Select floor between 0 - 99.");
	UART_send_string ("\r\n");
	UART_send_string("'*' confirms floor.");
	UART_send_string ("\r\n");
	
	while(keypad_ready == 0){
		
		// Read raw signal from keypad
		uint8_t key_signal = KEYPAD_GetKey();
		
		// Debounce:
		_delay_ms(100);
		
		// Check for valid key press
		//if (key_signal != 0xFF) { // Assuming 0xFF means no key pressed

		// For numeric keys (0 to 9)
		if (key_signal >= '0' && key_signal <= '9') {
			// Convert key to numeric value
			uint8_t key_value = key_signal - '0'; // Convert ASCII value to numeric value

			itoa(key_value, key, 10);
			UART_send_string("Pressed key: ");
			UART_send_string(key);
			UART_send_string(" -- ");
			UART_send_string("Key index: ");
			itoa(key_index, key, 10);
			UART_send_string(key);
			UART_send_string ("\r\n");

			// If two values are already given, reset the current input and start over
			if(key_index == 2){
				floor_value = 0;
				key_index = 0;
			}
			
			// Add the First value "X + Y"
			if(key_index == 0){
				floor_value = key_value;
				key_index++;
				
				// Add the Second value "X + Y"
				} else if (key_index == 1){
				floor_value = floor_value*10 + key_value;
				key_index++;
			}
			
			itoa(floor_value, key, 10);
			UART_send_string("Current floor: ");
			UART_send_string(key);
			char lcd_msg[16];
			sprintf(lcd_msg, "Input: %s", key);
			LCD_top(lcd_msg);
			UART_send_string ("\r\n");
		}
		
		// If key input is '*' we are ready with inputs.
		//}
		
		else if(key_signal == 42){
			keypad_ready = 1;
			itoa(floor_value, key, 10);
			UART_send_string("Set floor: ");
			UART_send_string(key);
			UART_send_string ("\r\n");
			} else {
			itoa(floor_value, key, 10);
			UART_send_string("Key pressed: ");
			UART_send_string(key);
			UART_send_string ("\r\n");
			UART_send_string("Give proper value between 0 - 9");
			UART_send_string ("\r\n");
		}
	}
	return floor_value;
}

uint8_t get_any_key_press(){
	uint8_t key_signal = 0xFF;
	UART_send_string("Press any key to stop emergency melody.\r\n");
	while(key_signal == 0xFF){
		key_signal = KEYPAD_GetKey();
		_delay_ms(100); // Debounce
	}
	return key_signal;
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
				// Flash an indicator LED on the master (optional for debugging)
				PORTB |= (1 << PB7);  // Assuming LED on PB7 (adjust if needed)
				_delay_ms(100);
				PORTB &= ~(1 << PB7);
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

// Initialize TWI as master (remains the same)
void TWI_init(void) {
	TWBR = 72;
	TWSR = 0;
}

// Start TWI transmission (remains the same)
uint8_t TWI_start(void) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
	if ((TWSR & 0xF8) != 0x08) {
		UART_send_string("TWI start error\r\n");
		return 0;
	}
	return 1;
}

// Write a byte to the TWI bus (remains the same)
uint8_t TWI_write(uint8_t data) {
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
	if ((TWSR & 0xF8) != 0x18 && (TWSR & 0xF8) != 0x28) {
		UART_send_string("TWI write error\r\n");
		return 0;
	}
	return 1;
}

// Stop TWI transmission (remains the same)
void TWI_stop(void) {
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
	_delay_ms(1);
}

// Initialize UART for debugging (remains the same)
void UART_init(void) {
	UBRR0H = 0;
	UBRR0L = 103;
	UCSR0B = (1 << TXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Send a character through UART (remains the same)
void UART_send_char(unsigned char data) {
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}

// Send a string through UART (remains the same)
void UART_send_string(char* str) {
	while (*str) {
		UART_send_char(*str++);
	}
}

void emergency_mode() {
	emergency_sequence_active = true;
	UART_send_string("Emergency!!!\r\n");
	LCD_top("EMERGENCY!");
	clear_LCD_line(1);
	send_command('E');  // Send emergency command to slave (blinking LED)
	UART_send_string("Press any key on keypad to open door and start emergency melody.\r\n");
	get_any_key_press(); // Wait for any key press
	send_command('P');  // Send command to open door and start melody on slave
	LCD_top("Door is open!");
}

void stop_emergency_melody() {
	send_command('X');  // Send command to stop melody on slave
	UART_send_string("Emergency melody stopped. Door will close.\r\n");
	LCD_top("EMERGENCY - Melody OFF");
	// Slave will handle door closing automatically after melody stop
	LCD_bottom(current_floor); // Update the displayed floor
	LCD_top("Choose the floor"); // Go back to idle message
}