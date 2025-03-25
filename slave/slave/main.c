/*
 * slave.c
 *
 * Created: 25.3.2025 14.19.19
 * Author : roope
 
###################################
##  THIS IS CODE FOR THE SLAVE  ##
###################################
 */ 

/*
#########################################################

ALL HERE BELOW FROM WEEK10 EXC - I2C / TWI COMMUNICATIONS

#########################################################
*/

#define F_CPU 16000000UL
#define FOSC 16000000UL
#define BAUD 9600

#define MYUBRR (FOSC/16/BAUD)-1

#define SLAVE_ADDRESS 0b1010111

#include <avr/io.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

//USART initiation p.206
static void USART_init(uint16_t ubrr) {
	UBRR0H = (unsigned char) (ubrr >> 8);   //datasheet p.206
	UBRR0L = (unsigned char) ubrr;          //datasheet p.206
	UCSR0B |= (1 << RXEN0) | (1 << TXEN0);  //datasheet p.206
	UCSR0C |= (1 << USBS0) | (3 << UCSZ00); //datasheet p.221 and p.222
}

//datasheet p.207
static void USART_Transmit(unsigned char data, FILE *stream){
	/* Wait until the transmit buffer is empty*/
	while(!(UCSR0A & (1 << UDRE0))) //datasheet p.207, p. 219
	{
		;
	}

	/* Puts the data into a buffer, then sends/transmits the data */
	UDR0 = data;
}

static char USART_Receive(FILE *stream) //datasheet p.210, 219
{
	/* Wait until the transmit buffer is empty*/
	while(!(UCSR0A & (1 << RXC0)))
	{
		;
	}

	/* Get the received data from the buffer */
	return UDR0;
}

// Stream functions for UART
FILE uart_output = FDEV_SETUP_STREAM(USART_Transmit, NULL, _FDEV_SETUP_WRITE);
FILE uart_input = FDEV_SETUP_STREAM(NULL, USART_Receive, _FDEV_SETUP_READ);

int main(void)
{

	USART_init(MYUBRR);
	
	stdout = &uart_output;
	stdin = &uart_input;
	
	// Init the TWI Slave
	TWCR |= (1 << TWEA) | (1 << TWEN);
	TWCR &= ~(1 << TWSTA) & ~(1 << TWSTO);
	
	TWAR = 0b10101110; // 7-bit slave address and 1 write bit (LSB)

	char twi_receive_data[20];
	char test_char_array[16];
	uint8_t twi_index = 0;
	uint8_t twi_status = 0;

	while (1){
		
		while(!(TWCR & (1 << TWINT))){;} // Wait for TWINT Flag set
		
		twi_status = (TWSR & 0xF8); //Check value of TWI status register
		
		TWCR |= (1 << TWINT) | (1 << TWEA) | (1 << TWEN); // Clear TWINT bit in TWCR to start transmission of address
		
		while(!(TWCR & (1 << TWINT))){;} // Wait for TWINT Flag set
		
		twi_status = (TWSR & 0xF8); // Check value of TWI status register
		
		// Check slave receiver status codes
		if ((twi_status == 0x80) || (twi_status == 0x90)) {
			twi_receive_data[twi_index] = TWDR;
			twi_index++;
			}else if ((twi_status == 0x88) || (twi_status == 0x98)) {
			twi_receive_data[twi_index] = TWDR;
			twi_index++;
			}else if (twi_status == 0xA0) {
			TWCR |= (1 << TWINT);
		}
		
		if(20 <= twi_index){
			printf(twi_receive_data);
			twi_index = 0;
		}
	}
}


