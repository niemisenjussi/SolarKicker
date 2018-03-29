#include "USART.h"
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

FILE serial_port0 = FDEV_SETUP_STREAM(put_char0, NULL, _FDEV_SETUP_WRITE);
//FILE serial_port1 = FDEV_SETUP_STREAM(put_char1, NULL, _FDEV_SETUP_WRITE);
//FILE serial_port2 = FDEV_SETUP_STREAM(put_char2, NULL, _FDEV_SETUP_WRITE);
//FILE serial_port3 = FDEV_SETUP_STREAM(put_char3, NULL, _FDEV_SETUP_WRITE);



int put_char0(char c, FILE *stream){
	 //if (c == '\n') put_char0('\r', stream);
	 loop_until_bit_is_set(UCSR0A, UDRE0);
	 UDR0 = c;
	 return 0;
}
/*
int put_char1(char c, FILE *stream){
	 if (c == '\n') put_char1('\r', stream);
	 loop_until_bit_is_set(UCSR1A, UDRE1);
	 UDR1 = c;
	 return 0;
}

int put_char2(char c, FILE *stream){
	 //if (c == '\n') put_char2('\r', stream); //Disable this dummy thing
	 loop_until_bit_is_set(UCSR2A, UDRE2);
	 UDR2 = c;
	 return 0;
}

int put_char3(char c, FILE *stream){
	 if (c == '\n') put_char3('\r', stream);
	 loop_until_bit_is_set(UCSR3A, UDRE3);
	 UDR3 = c;
	 return 0;
}
*/
void USART_Init(FILE *port0, uint32_t br0){
	uint32_t MYUBRR = 0;
	//int MYUBRR = 16; // 16 = 115200, 34 = 57600
	if (port0 != NULL){
		MYUBRR = F_CPU/(8*br0)-1;
		//DEbug serial
		DDRD |= 0x02; //TXD0 output
		DDRD &= ~(1<<0); //RXD0 input
		UBRR0H = (uint8_t)(MYUBRR>>8);
		UBRR0L = (uint8_t)MYUBRR;
		UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0); //|(1<<TXCIE0);
		UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
		UCSR0A |= (1<<U2X0);
		*port0 = serial_port0;
		//if (defaultport == 0){
		stdout = &serial_port0; //Required for printf init
		//}
	}/*
	if (port1 != NULL){
		MYUBRR = F_CPU / (8*br1) - 1;
		//Unused port currently
		DDRD |= 0x08; //TXD1 output
		UBRR1H = (uint8_t)(MYUBRR>>8);
		UBRR1L = (uint8_t)MYUBRR;
		UCSR1B |= (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1);//|(1<<TXCIE2);
		UCSR1C |= (1<<UCSZ11)|(1<<UCSZ10);
		UCSR1A |= (1<<U2X1);
		*port1 = serial_port1;
		if (defaultport == 1){
			stdout = &serial_port1; //Required for printf init
		}
	}
	if (port2 != NULL){
		MYUBRR = F_CPU / (8*br2) - 1;
		DDRH |= 0x02; //TXD2 output
		UBRR2H = (uint8_t)(MYUBRR>>8);
		UBRR2L = (uint8_t)MYUBRR;
		UCSR2B |= (1<<RXEN2)|(1<<TXEN2)|(1<<RXCIE2);//|(1<<TXCIE2);
		UCSR2C |= (1<<UCSZ21)|(1<<UCSZ20);
		UCSR2A |= (1<<U2X2);
		*port2 = serial_port2;
		if (defaultport == 2){
			stdout = &serial_port2; //Required for printf init
		}
	}
	if (port3 != NULL){
		MYUBRR = F_CPU / (8*br3) - 1;
		//GPS serial
		//DDRH |= 0x02; //TXD2 output
		UBRR3H = (uint8_t)(MYUBRR>>8);
		UBRR3L = (uint8_t)MYUBRR;
		UCSR3B |= (1<<RXEN3)|(1<<TXEN3)|(1<<RXCIE3);//|(1<<TXCIE2);
		UCSR3C |= (1<<UCSZ31)|(1<<UCSZ30);
		UCSR3A |= (1<<U2X3);
		*port3 = serial_port3;
		if (defaultport == 3){
			stdout = &serial_port3; //Required for printf init
		}
	}*/
}

void USART_Transmit0( unsigned char data ){
	while ( !( UCSR0A & (1<<UDRE0)) );
	UDR0 = data;
}/*
void USART_Transmit1( unsigned char data ){
	while ( !( UCSR1A & (1<<UDRE1)) );
	UDR1 = data;
}
void USART_Transmit2( unsigned char data ){
	while ( !( UCSR2A & (1<<UDRE2)) );
	//_delay_us(500);
	UDR2 = data;
}
void USART_Transmit3( unsigned char data ){
	while ( !( UCSR3A & (1<<UDRE3)) );
	//_delay_us(500);
	UDR3 = data;
}
*/

/* unsigned char USART_Receive0(void){
	while ( !(UCSR0A & (1<<RXC0)) );
	return UDR0;
}

unsigned char USART_Receive1(void){
	while ( !(UCSR1A & (1<<RXC1)) );
	return UDR1;
}

unsigned char USART_Receive2(void){
	while ( !(UCSR2A & (1<<RXC2)) );
	return UDR2;
}

unsigned char USART_Receive3(void){
	while ( !(UCSR3A & (1<<RXC3)) );
	return UDR3;
} */

/*void USART_Transmit(unsigned char data){   
	while ( !( UCSR0A & (1<<UDRE0)) );
	UDR0 = data;
}*/

void USART0_Flush( void ){
	unsigned char dummy;
	while ( UCSR0A & (1<<RXC0) ){
		dummy = UDR0;
		if (dummy){}
	}
}
/*void USART1_Flush( void ){
	unsigned char dummy;
	while ( UCSR1A & (1<<RXC1) ){
		dummy = UDR1;
		if (dummy){}
	}
}
void USART2_Flush( void ){
	unsigned char dummy;
	while ( UCSR2A & (1<<RXC2) ){
		dummy = UDR2;
		if (dummy){}
	}
}
void USART3_Flush( void ){
	unsigned char dummy;
	while ( UCSR3A & (1<<RXC3) ){
		dummy = UDR3;
		if (dummy){}
	}
}
*/

unsigned char uart_getchar(void)
{
    while( !(UCSR0A & (1<<RXC0)) );
    return(UDR0);
}


void waitResponce(const char *string, uint8_t length){
	cli();
	uint8_t match=0;
	while(length != match){
		if (USART_Receive0() == pgm_read_byte(string+match)){
			match ++;
		}
		else{
			match = 0;
		}
	}
	USART0_Flush();
	sei();
}









