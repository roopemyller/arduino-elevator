/*

main.c
 
###################################
##  THIS IS CODE FOR THE MASTER  ##
###################################
*/ 

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


// For UART
#define FOSC  16000000
#define MYUBRR (FOSC/16/BAUD)-1

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

// Function prototypes / declaration
void setup(void);
uint8_t get_key_pressed();
uint8_t get_any_key_press();
void TWI_init(void);
uint8_t TWI_start(void);
uint8_t TWI_write(uint8_t data);
void TWI_stop(void);
static void UART_init(uint16_t ubrr);
void UART_send_char(unsigned char data);
void UART_send_string(char* str);
void send_command(char cmd);
void door_open_close();
void LCD_top(const char* msg);
void LCD_bottom(int floor);
void clear_LCD_line(uint8_t row);
void emergency_mode();
void stop_emergency_melody();
void start_movement();
void stop_movement();


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
	
	// setup contains critical initializations
	setup();
	
	// string for converting key input for UART
	char str[30];
	
	// This is the main loop
	while (1) {
		
		// First we check if emergency button is activated or not
		// Emergency is set with ISR and the flag "emergency_stop_requested" that is raised in the ISR function
		// Flag is used to minimize ISR loop size
		// This is the FIRST sequence of emergency! (no button press in between)
		if (emergency_stop_requested) {
			// Reset the flag
			emergency_stop_requested = false;
			// Disable interrupts for security reasons inside the emergency mode!
			cli();
			// Jump to emergency mode!
			emergency_mode();
			// Activate interrupts once again when emergency is over.
			sei();
			// Jump to beginning of the loop
			continue;
		}
		
		// This is the SECOND part of the emergency sequence!
		// If emergency is activated and the button is already pressed once (to start the melody)
		if(emergency_sequence_active){
			
			emergency_sequence_active = false;
			uint8_t stop_key = get_any_key_press();
			UART_send_string("Key pressed during emergency: ");
			itoa(stop_key, str, 10);
			UART_send_string(str);
			UART_send_string("\r\n");
			stop_emergency_melody();
			
			// If emergency is NOT active!
			// This branch does the basic elevator functions
			} else {

			key_input = get_key_pressed();

			itoa(key_input, str, 10);
			UART_send_string("Pressed key: ");
			UART_send_string(str);
			UART_send_string("\r\n");

			// Define next state
			// Check whether to move up, down or idle (fault state).
			if (current_floor < key_input) {
				current_state = 1;
			}
			if (current_floor == key_input) {
				current_state = 2;
			}
			if (current_floor > key_input) {
				current_state = 3;
			}
			
			
			// Calculate how many floors we need to move
			floors_to_travel = abs(current_floor - key_input);

			// UART Debug:
			UART_send_string("Current floor: ");
			itoa(current_floor, str, 10);
			UART_send_string(str);
			UART_send_string(", Floors to travel: ");
			itoa(floors_to_travel, str, 10);
			UART_send_string("\r\n");
			
			/*** GOING UP state ***/
			switch (current_state) {
				case 1:
				// Debounce:
				_delay_ms(20);
				// Debug:
				UART_send_string("Going up!\r\n");
				
				// Send a command "move" to the slave
				send_command('M');
				// Update LCD:
				LCD_top("Going up");
				// Set moving flag
				elevator_moving = true;
				// Move the elevator with a for loop (one floor at a time)
				for (int i = 0; i < floors_to_travel; i++) {
					// Check if emergency is activated DURING the movement!
					if (emergency_stop_requested) {
						// Same as before!
						cli();
						emergency_stop_requested = false;
						emergency_mode();
						sei();
						// Jump to an idle state (skips the next logic)
						goto idle_state;
					}
					// Otherwise move floor up one by one:
					current_floor++;
					// Debug:
					UART_send_string("Moving to floor: ");
					itoa(current_floor, str, 10);
					UART_send_string(str);
					UART_send_string("\r\n");
					// Update LCD:
					LCD_bottom(current_floor);
					// Delay between floor changes:
					_delay_ms(100);
				}
				
				/*** When floor is reached: ***/
				
				// Send "stop" command to slave!
				send_command('S');
				elevator_moving = false; // Clear moving flag
				// Activate open / close sequence:
				door_open_close();
				// Slight delay to stablize functions
				_delay_ms(200);
				break;
				

				/*** FAULT STATE (same floor) ***/
				case 2:
				// Debounce:
				_delay_ms(20);
				// Degub commands to terminal with UART:
				UART_send_string("Fault state.\r\n");
				// Updaet LCD:
				LCD_top("Same Floor");
				// Send "fault" to slave (control the led)
				send_command('F');
				
				// Small delay between normal operation:
				_delay_ms(1200);
				
				// Update LCD once again:
				LCD_top("Choose the floor");
				
				// Small delay
				_delay_ms(200);
				break;
				
				
				/*** GO DOWN STATE ***/
				case 3:
				// Debounce:
				_delay_ms(20);
				// Debug:
				UART_send_string("Going down!\r\n");
				
				// Send a "move" command to the LED (controls leds)
				send_command('M');
				
				// Update LCD:
				LCD_top("Going down");
				
				// Set moving flag
				elevator_moving = true;
				
				// Move the elevator with a for loop (one floor at a time)
				for (int i = 0; i < floors_to_travel; i++) {
					
					// Same logic as with the going up state:
					if (emergency_stop_requested) {
						cli();
						emergency_stop_requested = false;
						emergency_mode();
						sei();
						// Jump to the idle state to skip the next up fucntionality:
						goto idle_state;
					}
					// Otherwise normal operation:
					current_floor--;
					UART_send_string("Moving to floor: ");
					itoa(current_floor, str, 10);
					UART_send_string(str);
					UART_send_string("\r\n");
					LCD_bottom(current_floor);
					_delay_ms(100);
				}
				// When floor reached:
				
				// Send "stop" to slave
				send_command('S');
				elevator_moving = false; // Clear moving flag
				
				// Door open close sequence
				door_open_close();
				// Small delay
				_delay_ms(200);
				break;
			}
		}
		
		// Idle state for skipping logic from emergency
		idle_state:
		// Small debounce:
		_delay_ms(10);
	}
	return 0;
}

// Setup function
void setup(){
	// Initialize LCD
	lcd_init(LCD_DISP_ON);
	lcd_clrscr();
	LCD_top("Choose the floor");
	LCD_bottom(0);
	_delay_ms(20);
	
	// Initialize UART for debugging
	UART_init(MYUBRR);
	// Just in case a delay to stabilize init
	_delay_ms(20);
	
	// Important command, starts UART debug terminal
	UART_send_string("Elevator Master Program - NASA SERTIFIED PRODUCT\r\n");

	// Initialize TWI
	// Due to conection integrity issues we reduced the I2C speed to 100 khz
	TWBR = 72; // TWI bit rate register is set to 100 kHz, this is less likely to fail compared to the 0x03 in the course material (400 kHz)
	TWSR = 0x00; // TWI status register prescaler value set to 1
	
	// Just in case a delay to stabilize init
	_delay_ms(20);
	// Init keypad
	KEYPAD_Init();
	// Just in case a delay to stabilize init
	_delay_ms(20);

	// Initialize interrupts for emergency button
	EIMSK |= (1 << INT2);      // Enable external interrupt INT2
	EICRA |= (1 << ISC21);      // Falling edge generates interrupt request, datasheet p. 110.
	EICRA &= ~(1 << ISC20);		// Bitmask: flip all but the ISC20 (1,0)
	DDRD &= ~(1 << EMERGENCY_BUTTON); // PD2 (INT2) as input
	PORTD |= (1 << EMERGENCY_BUTTON);  // Enable pull-up resistor on PD2. No external resistor needed and the button drives the pin to ground.
	sei();                      // Enable global interrupts
}

// Door open close sequence
void door_open_close() {
	
	// UART debug:
	UART_send_string("Door open!\r\n");
	// Send "open" command to slave"
	send_command('O');
	LCD_top("Door is open");
	_delay_ms(5000);

	UART_send_string("Door close!\r\n");
	// Send "close" command to slave
	send_command('C');
	LCD_top("Door is closed");
	_delay_ms(1000);
	LCD_top("Choose the floor");
}

// LCD display top row updates
void LCD_top(const char* msg) {
	
	// Move the cursor to top left
	lcd_gotoxy(0,0);
	// Clear the line
	clear_LCD_line(0);
	// Write the message to the line
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
	// Go to the target row
	lcd_gotoxy(0, row);
	
	// overwrite the row with "' '" (clear).
	for(uint8_t i=0; i < 16; i++){
		lcd_putc(' ');
	}
	// Put the cursor back to beginning.
	lcd_gotoxy(0,row);
}

// Function to retreive the floor input
uint8_t get_key_pressed(){
	
	// Helping variables
	uint8_t keypad_ready = 0;
	// Indexing the two input values
	uint8_t key_index = 0;
	// Selected floor value
	uint8_t floor_value = 0;
	
	// Key input string
	char key[16];
	
	// UART debug messages:
	UART_send_string("Select floor between 0 - 99.");
	UART_send_string ("\r\n");
	UART_send_string("'*' confirms floor.");
	UART_send_string ("\r\n");
	
	// While the keypad input is not ready
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
			
			
			// UART debug messages:
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
				// with only 2 digit floors we can use simple maths: e.g. first number = 7 and second is 2
				// Logic: 7 -> 7*10 = 70 + 2 --> floor is 72!
				floor_value = floor_value*10 + key_value;
				key_index++;
			}
			
			// UART debug:
			itoa(floor_value, key, 10);
			UART_send_string("Current floor: ");
			UART_send_string(key);
			UART_send_string ("\r\n");
			
			// Update the lcd:
			char lcd_msg[16];
			sprintf(lcd_msg, "Input: %s", key);
			LCD_top(lcd_msg);
		}
		
		// If key input is '*' we are ready with inputs (keypad_ready = true).
		// 42 = '*'
		else if(key_signal == 42){
			keypad_ready = 1;
			// UART debug:
			itoa(floor_value, key, 10);
			UART_send_string("Set floor: ");
			UART_send_string(key);
			UART_send_string ("\r\n");
			
			// if not '*' or 0-9, inproper input:
			} else {
			// UART debug:
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

// This is for the SECOND emergency phase
uint8_t get_any_key_press(){
	// Get any keypad input and continue
	uint8_t key_signal = 0;
	UART_send_string("Press any key to stop emergency melody.\r\n");
	while(key_signal == 0){
		key_signal = KEYPAD_GetKey();
		_delay_ms(100); // Debounce
	}
	// Continue by returning any value
	return key_signal;
}

// Start TWI transmission
uint8_t TWI_start(void) {
	// These states and comments are from datasheet page 246
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // 'Send START condition'
	while (!(TWCR & (1 << TWINT))); // 'Wait for TWINT Flag set. This indicates that the START condition has been transmitted'
	if ((TWSR & 0xF8) != 0x08) { // 'Check value of TWI Status Register. Mask prescaler bits. If status different from START go to ERROR'
		// UART debug
		UART_send_string("TWI start error\r\n");
		return 0;
	}
	return 1;
}

// Stop TWI transmission (remains the same)
void TWI_stop(void) {
	// 'Transmit STOP condition' from datasheet page 246
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
	// debounce
	_delay_ms(1);
}

// Write a byte to the TWI bus (remains the same)
uint8_t TWI_write(uint8_t data) {
	// TWDR contains the data to be sent. This loads data to the data register
	TWDR = data;
	// 'Clear TWINT bit in TWCR to start transmission of data' from datasheet page 246
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT))); // While ACK has been received and data sent
	// 'Check value of TWI StatusRegister. Mask prescaler bits. If status different from MT_DATA_ACK go to ERROR' from datasheet
	if ((TWSR & 0xF8) != 0x18 && (TWSR & 0xF8) != 0x28) {
		// UART debug
		UART_send_string("TWI write error\r\n");
		return 0;
	}
	return 1;
}

// Send a command to the slave device
void send_command(char cmd) {
	
	// UART debug:
	UART_send_string("Sending command: ");
	UART_send_char(cmd);
	UART_send_string("\r\n");

	// if TWI is ready:
	// TWI_start() inits / starts the TWI transmission, if OK, we can continue!
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
		// Stop TWI transmission
		TWI_stop();
		// UART debug
		UART_send_string("TWI stopped\r\n");
		} else {
		UART_send_string("TWI start failed\r\n");
	}
}

// This is the FIRST SEQUENCE of emergency mode!
void emergency_mode() {
	// Activate the second sequence already (needed when exiting this sequence)
	emergency_sequence_active = true;
	// UART debug and LCD update
	UART_send_string("Emergency!!!\r\n");
	LCD_top("EMERGENCY!");
	clear_LCD_line(1);
	
	// Send the EMERGENCY command activat the first sequence!
	send_command('E');  // Send emergency command to slave (blinking LED)
	UART_send_string("Press any key on keypad to open door and start emergency melody.\r\n");
	
	// Wait for any keypress to contiue to second sequence
	get_any_key_press(); // Wait for any key press
	// Send command "play" to slave to activate buzzer and led! This activates SECOND phase for slave!
	send_command('P');  // Send command to open door and start melody on slave
	
	// Update LCD
	LCD_top("Door is open!");
}

// This function stops the second emergency sequence
void stop_emergency_melody() {
	// Send command to stop melody on slave
	send_command('X');
	
	// UART debug message
	UART_send_string("Emergency melody stopped. Door will close.\r\n");
	LCD_top("EMERGENCY Off.");
	_delay_ms(300); // Debounce
	LCD_top("EMERGENCY Off.");
	_delay_ms(300); // Debounce
	LCD_top("EMERGENCY Off..");
	_delay_ms(300); // Debounce
	LCD_top("EMERGENCY Off..");
	_delay_ms(300); // Debounce
	LCD_top("EMERGENCY Off...");
	_delay_ms(1300); // Debounce
	// Slave will handle door closing automatically after melody stop
	LCD_bottom(current_floor); // Update the displayed floor
	LCD_top("Choose the floor"); // Go back to idle message

}


// Initialize UART for debugging (remains the same)
static void UART_init(uint16_t myubrr) {
	// Set baudrate:
	UBRR0H = (unsigned char) (MYUBRR >> 8); // from course material, datasheet p.206
	UBRR0L = (unsigned char) MYUBRR;		// from course material, datasheet p.206
	UCSR0B = (1 << TXEN0);					// Enable TX, no receiving needed.
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // stop bits = 2, this is the standard way, otherwise datasheet p.221 and p.222
}

// Send a character through UART (remains the same)
void UART_send_char(unsigned char data) {
	// From course material: Wait until the transmit buffer is empty
	while (!(UCSR0A & (1 << UDRE0))){
		;
	}
	// From course material: Put the data into a buffer (UDR0), then transmit the data
	UDR0 = data;
}

// Send a string through UART
void UART_send_string(char* str) {
	// This function puts the string to the sending function
	while (*str) {
		UART_send_char(*str++);
	}
}
