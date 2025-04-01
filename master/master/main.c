/*
 * master.c
 *
 * Created: 25.3.2025 14.17.55
 * Author : roope
 
 ###################################
 ##  THIS IS CODE FOR THE MASTER  ##
 ###################################
 */ 

#define F_CPU 16000000UL
#define SLAVE_ADDRESS 8  // The address of the slave (UNO)

// Button pins - Using actual Arduino Mega pin mappings
// PE4 = Digital Pin 2
// PE5 = Digital Pin 3
// PG5 = Digital Pin 4
#define MOVEMENT_BUTTON_PORT PORTE
#define MOVEMENT_BUTTON_PIN PINE
#define MOVEMENT_BUTTON_DDR DDRE
#define MOVEMENT_BUTTON PE4

#define STOP_BUTTON_PORT PORTE
#define STOP_BUTTON_PIN PINE
#define STOP_BUTTON_DDR DDRE
#define STOP_BUTTON PE5

#define DOOR_BUTTON_PORT PORTG
#define DOOR_BUTTON_PIN PING
#define DOOR_BUTTON_DDR DDRG
#define DOOR_BUTTON PG5

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Variables to track button states for debouncing
volatile uint8_t prev_movement_state = 1;
volatile uint8_t prev_stop_state = 1;
volatile uint8_t prev_door_state = 1;

// Function prototypes
void TWI_init(void);
uint8_t TWI_start(void);
uint8_t TWI_write(uint8_t data);
void TWI_stop(void);
void UART_init(void);
void UART_send_char(unsigned char data);
void UART_send_string(char* str);
void send_command(char cmd);

int main(void) {
    // Initialize UART for debugging
    UART_init();
    UART_send_string("I2C Master Test - FIXED VERSION\r\n");
    
    // Initialize TWI as master
    TWI_init();
    
    // Set up button pins as inputs with pull-up resistors
    MOVEMENT_BUTTON_DDR &= ~(1 << MOVEMENT_BUTTON);  // Set as input
    MOVEMENT_BUTTON_PORT |= (1 << MOVEMENT_BUTTON);  // Enable pull-up
    
    STOP_BUTTON_DDR &= ~(1 << STOP_BUTTON);  // Set as input
    STOP_BUTTON_PORT |= (1 << STOP_BUTTON);  // Enable pull-up
    
    DOOR_BUTTON_DDR &= ~(1 << DOOR_BUTTON);  // Set as input
    DOOR_BUTTON_PORT |= (1 << DOOR_BUTTON);  // Enable pull-up
    
    UART_send_string("System ready. Press buttons to send commands to slave:\r\n");
    UART_send_string("- Pin 2: Start elevator movement (M)\r\n");
    UART_send_string("- Pin 3: Stop elevator (S)\r\n");
    UART_send_string("- Pin 4: Open/close door (O)\r\n");
    
    // Print initial button states for debugging
    UART_send_string("Initial button states: ");
    UART_send_char((MOVEMENT_BUTTON_PIN & (1 << MOVEMENT_BUTTON)) ? '1' : '0');
    UART_send_char((STOP_BUTTON_PIN & (1 << STOP_BUTTON)) ? '1' : '0');
    UART_send_char((DOOR_BUTTON_PIN & (1 << DOOR_BUTTON)) ? '1' : '0');
    UART_send_string("\r\n");
    
    while (1) {
        // Read current button states
        uint8_t current_movement_state = (MOVEMENT_BUTTON_PIN & (1 << MOVEMENT_BUTTON)) ? 1 : 0;
        uint8_t current_stop_state = (STOP_BUTTON_PIN & (1 << STOP_BUTTON)) ? 1 : 0;
        uint8_t current_door_state = (DOOR_BUTTON_PIN & (1 << DOOR_BUTTON)) ? 1 : 0;
        
        // Check MOVEMENT button (detect falling edge: 1->0)
        if (current_movement_state == 0 && prev_movement_state == 1) {
            _delay_ms(20);  // Debounce
            // Recheck after debounce
            if ((MOVEMENT_BUTTON_PIN & (1 << MOVEMENT_BUTTON)) == 0) {
                UART_send_string("Movement button pressed\r\n");
                send_command('M');
                _delay_ms(200);  // Prevent multiple triggers
            }
        }
        
        // Check STOP button (detect falling edge: 1->0)
        if (current_stop_state == 0 && prev_stop_state == 1) {
            _delay_ms(20);  // Debounce
            // Recheck after debounce
            if ((STOP_BUTTON_PIN & (1 << STOP_BUTTON)) == 0) {
                UART_send_string("Stop button pressed\r\n");
                send_command('S');
                _delay_ms(200);  // Prevent multiple triggers
            }
        }
        
        // Check DOOR button (detect falling edge: 1->0)
        if (current_door_state == 0 && prev_door_state == 1) {
            _delay_ms(20);  // Debounce
            // Recheck after debounce
            if ((DOOR_BUTTON_PIN & (1 << DOOR_BUTTON)) == 0) {
                UART_send_string("Door button pressed\r\n");
                send_command('O');
                _delay_ms(200);  // Prevent multiple triggers
            }
        }
        
        // Update previous states
        prev_movement_state = current_movement_state;
        prev_stop_state = current_stop_state;
        prev_door_state = current_door_state;
        
        // Small delay
        _delay_ms(10);
    }
    
    return 0;
}

// Send a command to the slave device
void send_command(char cmd) {
    UART_send_string("Sending command: ");
    UART_send_char(cmd);
    UART_send_string("\r\n");
    
    if (TWI_start()) {
        UART_send_string("TWI started\r\n");
        
        if (TWI_write((SLAVE_ADDRESS << 1) | 0)) {  // SLA+W
            UART_send_string("SLA+W sent\r\n");
            
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