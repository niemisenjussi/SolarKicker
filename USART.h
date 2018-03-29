#include <string.h>
#include <stdio.h>	
#include <util/delay.h>	
#include <avr/io.h>	

#define USART_FREQ 16000000UL

__attribute__((always_inline)) inline static uint8_t USART_Receive0(void) {
	while ( !(UCSR0A & (1<<RXC0)) );
	return UDR0;
}
/*
__attribute__((always_inline)) inline static uint8_t USART_Receive1(void) {
	while ( !(UCSR1A & (1<<RXC1)) );
	return UDR1;
}

__attribute__((always_inline)) inline static uint8_t USART_Receive2(void) {
	while ( !(UCSR2A & (1<<RXC2)) );
	return UDR2;
}

__attribute__((always_inline)) inline static uint8_t USART_Receive3(void) {
	while ( !(UCSR3A & (1<<RXC3)) );
	return UDR3;
}*/


int put_char0(char c, FILE *stream);
/*
int put_char1(char c, FILE *stream);
int put_char2(char c, FILE *stream);
int put_char3(char c, FILE *stream);
*/
void USART0_Flush( void );
/*
void USART1_Flush( void );
void USART2_Flush( void );
void USART3_Flush( void );
*/
void USART_Transmit0( unsigned char data );
/*
void USART_Transmit1( unsigned char data );
void USART_Transmit2( unsigned char data );
void USART_Transmit3( unsigned char data );*/
/* unsigned char USART_Receive0(void) __attribute__((always_inline));
unsigned char USART_Receive1(void) __attribute__((always_inline));
unsigned char USART_Receive2(void) __attribute__((always_inline));
unsigned char USART_Receive3(void) __attribute__((always_inline)); */


unsigned char uart_getchar(void);
void USART_Init(FILE *port0, uint32_t br0);
void waitResponce(const char *string, uint8_t length);

//static FILE mystdout2 = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

//#endif
