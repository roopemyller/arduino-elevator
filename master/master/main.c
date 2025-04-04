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

// Button pins - Using actual Arduino Mega pin mappings
// PL5 = Digital Pin 44
// PL3 = Digital Pin 46
// PL1 = Digital Pin 48
#define MOVEMENT_BUTTON_PORT PORTL
#define MOVEMENT_BUTTON_PIN PINL
#define MOVEMENT_BUTTON_DDR DDRL
#define MOVEMENT_BUTTON PL5

#define STOP_BUTTON_PORT PORTL
#define STOP_BUTTON_PIN PINL
#define STOP_BUTTON_DDR DDRL
#define STOP_BUTTON PL3

#define DOOR_BUTTON_PORT PORTL
#define DOOR_BUTTON_PIN PINL
#define DOOR_BUTTON_DDR DDRL
#define DOOR_BUTTON PL1

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