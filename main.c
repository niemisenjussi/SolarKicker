#include <avr/io.h>    /* Device specific declarations */
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "i2c_master.h"
#include <stdlib.h>

#include <ds18b20.h>

//Debug led control for seeing that main loop is running
#define LED_DDR DDRB
#define LED_PORT PORTB
#define LED_PIN PINB

#define LED1_PIN_NUM PB5
#define LED2_PIN_NUM PB4
#define LED3_PIN_NUM PB3
#define LED4_PIN_NUM PB0

#define DISABLE_LED1 LED_PORT |= (1<<LED1_PIN_NUM);
#define ENABLE_LED1 LED_PORT &= ~(1<<LED1_PIN_NUM);
#define TOGGLE_LED1 LED_PORT ^= (1<<LED1_PIN_NUM);

#define DISABLE_LED2 LED_PORT |= (1<<LED2_PIN_NUM);
#define ENABLE_LED2 LED_PORT &= ~(1<<LED2_PIN_NUM);
#define TOGGLE_LED2 LED_PORT ^= (1<<LED2_PIN_NUM);

#define DISABLE_LED3 LED_PORT |= (1<<LED3_PIN_NUM);
#define ENABLE_LED3 LED_PORT &= ~(1<<LED3_PIN_NUM);
#define TOGGLE_LED3 LED_PORT ^= (1<<LED3_PIN_NUM);

#define DISABLE_LED4 LED_PORT |= (1<<LED4_PIN_NUM);
#define ENABLE_LED4 LED_PORT &= ~(1<<LED4_PIN_NUM);
#define TOGGLE_LED4 LED_PORT ^= (1<<LED4_PIN_NUM);


//Define PUSH button ports and pins
#define BUTTON_DDR DDRD
#define BUTTON_PORT PORTD
#define BUTTON_PIN PIND

#define BUTTON1_PIN_NUM PD4 //This is PCINT20 pin
#define BUTTON2_PIN_NUM PD3 //This is INT1 pin
#define BUTTON3_PIN_NUM PD2 //This is INT0 pin


//ADC 
#define DAC_ADC_PORT PORTC
#define DAC_ADC_DIR  DDRC
#define DAC_ADC_PIN  PC4 //PORTC pin 4
#define DAC_ADC_CHANNEL 4 //ADC channel 4
#define AVCC_REF 0x40
#define INT2V56_REF 0xC0
#define INT1V1_REF 0xC0


//PWM definitions
#define OC0A 0x80 //Enable PWM channel 1
#define OC0B 0x20 //Enable PWM channel 2
#define DAC_HYSTERESIS 12

//TC0
#define PWM_TCCRA TCCR0A
#define PWM_TCCRB TCCR0B
#define PWM_OCRA OCR0A
#define FAST_PWM 0x03
#define PRESCALER 0x01 //PWM frequency divider

#define DAC_PORT PORTD
#define DAC_DIR  DDRD
#define DAC_IN_PORT  PIND
#define DAC_PIN PD6  //6 pin = OC0A, 5 pin = OC0B

//TC2
/*
#define PWM_TCCRA TCCR2A
#define PWM_TCCRB TCCR2B
#define PWM_OCRA OCR2A
#define FAST_PWM 0x03
#define PRESCALER 0x01 //PWM frequency divider

#define DAC_PORT PORTB
#define DAC_DIR  DDRB
#define DAC_IN_PORT  PINB
#define DAC_PIN PB3  //OC2A
*/

//Serial port 
#define BUFSIZE 0xFF //Ring buffer size. Use 8 bit length so it has automatic overflow handling

//ADC stuff
float initADC(void);
uint16_t GetRAWVoltage(uint8_t Channel, uint8_t ADCRange);
float GetVoltage(uint8_t Channel, uint8_t ADCRange, uint8_t numofsamples);

//PWM stuff
void initPWM(uint8_t pwmvalue1, uint8_t pwmvalue2, uint8_t pwmvalue3);
void shutdownPWM(uint8_t channel);
void decreasePWM(uint8_t step);
void increasePWM(uint8_t step);
void setDAC_PWM(uint8_t value);
void shutdownDAC(void);

//Serial port functions
unsigned char uart_getchar(void);
int put_char0(char c, FILE *stream);
void USART_Transmit0( unsigned char data );
void USART0_Flush( void );
void USART_Init(FILE *port0, uint32_t br0);
unsigned char uart_getchar(void);
FILE serial_port0 = FDEV_SETUP_STREAM(put_char0, NULL, _FDEV_SETUP_WRITE);

__attribute__((always_inline)) inline static uint8_t USART_Receive0(void) {
    while ( !(UCSR0A & (1<<RXC0)) );
    return UDR0;
}

__attribute__((always_inline)) inline static void clearBuffer(volatile char *buffer,uint8_t len){
    for(uint8_t i=0;i<len;i++){    *buffer++ = '\0';}
}

float readFloat(uint8_t start, uint8_t stop);
void read_until_line_end(void);
void parseCommands(void);

//Common functions
void initCommonPorts(void);
float readTemperature(void);
void initTimer(void);
    
//UI stuff
void printMenu(void);

//##### Global variables #####
volatile uint8_t pwm_init_done = 0;
volatile uint8_t command_received = 0;
volatile float dac_set_value = 0.0f;
volatile uint8_t ticktimer = 0;
volatile uint8_t last_change = 0;
volatile uint8_t dac_running = 1; //By default DAC is running

//Serial ring buffer
volatile uint8_t ring_write = 0;
volatile uint8_t ring_read = 0;
volatile char buffer[BUFSIZE+1];

//ADC globals
const uint8_t INT1V1OFFSET = 21;
volatile uint16_t AVCC_VOLTAGE = 0;
volatile uint8_t current_reference = 0;

//PWM globals 
volatile uint8_t PWM_value = 0;

volatile uint8_t button3_pressed = 0;

//####### Interrupt vectors #######
/*ISR(USART_RX_vect) {
    volatile char rchar = USART_Receive0();
    if (rchar == '\n' || rchar == '\r'){
       command_received = 1; 
    }
    else{
        buffer[ring_write++] = rchar;    
    }
}

//Button3 Interrupt
ISR(INT0_vect){    
    if ((BUTTON_PIN & 0x04) == 0x00 && button3_pressed == 0){
        if (dac_running > 0){ //DAC is running => Stop it
            fprintf_P(&serial_port0, PSTR("DAC stopped\n"));
            DISABLE_LED4
            dac_running = 0;
        }
        else{ //Dac is not running, restore session
            fprintf_P(&serial_port0, PSTR("restoring DAC to "));
            fprintf(&serial_port0, "%.0fmv\n", dac_set_value);
            ENABLE_LED4
            dac_running = 1;    
        }
        button3_pressed    = 1;
    }
    else{
        button3_pressed = 0;            
    }
}

//Button2 Interrupt
ISR(INT1_vect){
    if ((BUTTON_PIN & 0x08) == 0){
        ENABLE_LED2
    }
    else{
        DISABLE_LED2    
    }
}

//Button3 Interrupt
ISR(PCINT2_vect){
    if ((BUTTON_PIN & 0x10) == 0){
        ENABLE_LED1
    }
    else{
        DISABLE_LED1    
    }
}

////Tick timer interrupt
ISR(TIMER2_COMPA_vect){
    ticktimer ++;
    last_change ++;
}
*/

uint8_t mcp3424_address = 0xD0;
uint8_t _resolution;
uint8_t _mode;
uint8_t _cfgbyte;
uint8_t _PGA;
uint8_t _buffer[5] = {0,0,0,0,0};


void MCP3424_SetConfig(unsigned char channel, unsigned char resolution, unsigned char gain){
    unsigned char PGAgain = 0;
    unsigned char sampleRate = 0;
    unsigned char conversionModeBit = 1; //1=Continuous, 0=One Shot
    unsigned char channelBits = 0; //0 = Channel 1
    channelBits = channel - 1; //zero based
    switch(gain) {
        case('8'):{
                PGAgain = 0x03;
                break;
        }
        case('4'):{
                PGAgain = 0x02;
                break;
        }
        case('2'):{
                PGAgain = 0x01;
                break;
        }
        case('1'):{
                PGAgain = 0x00;
                break;
        }
        default:{
            PGAgain = 0x00;
        }
    }

    switch(resolution) {
        case(18):{
                sampleRate = 0x03; //3.75 sps (18 bits), 3 bytes of data
                break;
        }
        case(16):{
                sampleRate = 0x02; //2 bytes of data,
                break;
        }
        case(14):{
                sampleRate = 0x01; //2 bytes of data
                break;
        }
        case(12):{
                sampleRate = 0x00; //240 SPS (12 bits), 2 bytes of data
                break;
        }
        default:{
                sampleRate = 0x00;
        }
    }
    _resolution = resolution;
    _PGA = gain;
    
    unsigned char config = PGAgain;
    config = config | (sampleRate << 2);
    config = config | (conversionModeBit << 4);
    config = config | (channelBits << 5);
    config = config | (1 << 7); //write a 1 here to initiate a new conversion in One-shot mode
    _cfgbyte = config;
    
    //fprintf(&serial_port0, "config:0x%x\n",config);
	i2c_start(mcp3424_address);
	uint8_t status = i2c_write(config);
    
	i2c_stop();
}

void newConversion(){
    i2c_start(mcp3424_address);
    uint8_t status = i2c_write((_cfgbyte+128));
    //fprintf(&serial_port0, "newConversion status:%d\n",status);
    i2c_stop();
}

uint8_t isConversionFinished(){
    uint8_t _requestedByte = 4;
    if(_resolution!=18){
        _requestedByte = 3;
    } 
   
    i2c_start(mcp3424_address | I2C_READ);
	//Read all conversion bits
	for (uint16_t i = 0; i < _requestedByte; i++){
		i2c_read_ack();
	}
    uint16_t loops = 0;
    while((i2c_read_ack() & 0x80) > 1){ //read until ready bit is low
        loops ++;
    }
   // fprintf(&serial_port0, "loops:%d  ", loops);
   // i2c_read_nack();
	//data[(length-1)] = i2c_read_nack();
	i2c_stop();
    
    uint8_t readstate = i2c_receive(mcp3424_address, _buffer, _requestedByte);
    
    //Wire.requestFrom(_adresse, _requestedByte);

  //  uint8_t _i = 0;

   // while(Wire.available()) _buffer[_i++]=Wire.read();
    //fprintf(&serial_port0, "rs:%d b[0]:0x%x b[1]:0x%x b[2]:0x%x b[3]:0x%x b[4]:0x%x\n",readstate, _buffer[0], _buffer[1],_buffer[2],_buffer[3],_buffer[4]);
    
    //uint8_t readybit = (_buffer[_requestedByte-1] & 0b10000000)>>7;
    //uint8_t channel = (_buffer[_requestedByte-1] & 0b01110000)>>4;
    //fprintf(&serial_port0, "ready:%d channel:%d\n", readybit, channel);
    return (_buffer[_requestedByte-1] & 0b10000000);
    
}


int32_t measure(){
    int32_t _resultat = 0;
    while(isConversionFinished()==1);
    switch (_resolution){
        case 12:{
             int32_t temp = (_buffer[0] & 0x0F);
             int32_t temp2 = (_buffer[1] & 0xFF);
             _resultat = (temp << 8) | temp2; 
             //temp = _buffer[0] & 0x80;
             //_resultat |= (temp << 24);
             //_resultat = _resultat*1000.0/_PGA;
             break;
        }
        case 14:{
            int32_t temp = (_buffer[0] & 0xBF);
            int32_t temp2 = (_buffer[1] & 0xFF);
            _resultat = (temp << 8) | temp2; 
            //temp = _buffer[0] & 0x80;
            //_resultat |= temp << 24;
            //_resultat = _resultat*250/_PGA;
             break;
        }
        case 16:{
            int32_t temp = (_buffer[0] & 0x7F);
            int32_t temp2 = (_buffer[1] & 0xFF);
            _resultat = (temp << 8) | temp2; 
            //temp = _buffer[0] & 0x80;
            //_resultat |= (temp << 24);
            //_resultat = _resultat*62.5/_PGA;
            break; 
        }
        case 18:{ //max value 524287
            int32_t temp =  (_buffer[0] & 0x01);
            int32_t temp2 = (_buffer[1] & 0xFF);
            int32_t temp3 = (_buffer[2] & 0xFF);
            _resultat = (temp << 16) + (temp2 <<8) + temp3; 
            //fprintf(&serial_port0, "temp:%d temp2:0%d temp3:0%d\n",_buffer[0], _buffer[1], _buffer[2]);
            //temp = _buffer[0] & 0x80;
            //_resultat |= (temp << 24);
            ///_resultat = _resultat*15.625/_PGA;
            break;
        }
    }
    return _resultat;
}

void generalCallLatch(void){
    i2c_start(mcp3424_address);
    uint8_t status = i2c_write(0x06);
    i2c_stop();
}

int main (int argc, char *argv[])
{
    _delay_ms(500);
    USART_Init(&serial_port0, 115200);
    initCommonPorts();
    i2c_init();
    USART0_Flush();    
    initADC();
    //initTimer();
    //
    
    //_delay_ms(2000);
    shutdownPWM(1);
    shutdownPWM(2);
    shutdownPWM(3);
    
    ASSR = 0x20; //Enable async mode Timer 2 Ext oscillator
    TCNT2 = 0;
    TCCR2B = 0x07; //1= /1, 0x02 = /8, 0x03= /32, 0x04 =/64, 0x05= /128, 0x06 = /256, 0x07 = /1024
    
    //AVCC = 4.52V
    
    GTCCR = 0x00; 
    
    fprintf_P(&serial_port0, PSTR("AVCC Voltage:"));
    fprintf(&serial_port0, "%d",AVCC_VOLTAGE);
    fprintf_P(&serial_port0, PSTR("mV\n"));
    
   // for (uint8_t i = 0; i<32; i++){
    fprintf(&serial_port0, "+14.6V Rail voltage:%.2fmV\n", GetVoltage(0x00, AVCC_REF, 16)*21); //14.54V 
   //}
    
    //_delay_ms(1);
    //fprintf(&serial_port0, "TCNT2:%d\n", TCNT2);
    //_delay_ms(10);
    //fprintf(&serial_port0, "TCNT2:%d\n", TCNT2);
    
    //printf_P(&serial_port0, PSTR("i2c_init\n"));
    
    
    MCP3424_SetConfig(1,18,1);
    
    //i2c_start(0xC0); // add Address bits on top off that
    //i2c_write(0x03); // set pointer to X axis MSB
    //i2c_stop();
    
    //Channel1 = VCC 5/1K jako => 5V/6K = 0.833V => 5V VCC
    //Channel2 = PANEL voltage 100K/1K jako => 
    //Channel4 = Panel Current 10K/10K jako => puolet jännitteestä. CH- kytketty VCC/2 => TODO korjaa kytkentä suoraan VIOUT=>CH4+
    
    
    //Measure current sensor lowest voltage when no current flows through resistors
    int32_t curoffset = 0;
    fprintf(&serial_port0, "Calibrating current sensor zero-point...\n");
    MCP3424_SetConfig(4,18,1);
    for (uint8_t i=0; i< 50; i++){ //4111111
                                   //330000
        int32_t zerovolt = measure();
       // if (zerovolt > curoffset){
        curoffset += zerovolt;   
       // }
        //fprintf(&serial_port0, "zerovolt: %ld\n",zerovolt);
    }
    curoffset /= 50;
    fprintf(&serial_port0, "Current sensor offset: %d\n",curoffset);
    
    
    /*fprintf(&serial_port0, "Calibrating CH3 ZEro volt...\n");
    MCP3424_SetConfig(3,18,1);
    for (uint8_t i=0; i< 20; i++){
        fprintf(&serial_port0, "CH3 zerovolt: %ld\n",measure());
    }*/
    
    
    
    initPWM(10,0,0);
    shutdownPWM(2);
    shutdownPWM(3);
    
    uint8_t ch = 2;
    uint16_t voltoffset = 3605;
    
    //MCP3424_SetConfig(ch,18,1);
    while(1){     
        if (command_received == 1){
            parseCommands();
            command_received = 0;
        }
        //Start conversion (without ROM matching)
		ds18b20convert( &PORTC, &DDRC, &PINC, ( 1 << 3 ), NULL );
        ds18b20convert( &PORTC, &DDRC, &PINC, ( 1 << 4 ), NULL );
        ds18b20convert( &PORTC, &DDRC, &PINC, ( 1 << 5 ), NULL );

		//Delay (sensor needs time to perform conversion)
		
        MCP3424_SetConfig(2,18,1); //CHANNEL 2 = Panel Voltage meter 100/1 resistor divider
        float rawvolt = measure();
        float Voltage = rawvolt;
        //if (Voltage < 0.0){
            //voltoffset = voltoffset + Voltage; //Add offset to current offset
        //}
        float measuredVoltage = (Voltage)/628.4f;
        //fprintf(&serial_port0, "CH%d Sample:%d Voltage:%ld volt:%f TCNT2:%d\n",2, samples, Voltage, measured, TCNT2);
        
        MCP3424_SetConfig(4,18,1);
        float Current = measure()-curoffset;
       // Current = measure()-curoffset;
        //if (Current < 0.0){
            //curoffset = curoffset + Current; //Add offset to current offset
        //}
        float measuredCurrent = (Current)/1.385; //4092000L/262143L*Voltage;
        
        fprintf(&serial_port0, "VoltageRAW:%.2f Voltage:%.2fV CurrentRAW:%.2f Current:%.2fmA\n",rawvolt, measuredVoltage, Current, measuredCurrent);
        
        // 16.253V 1.67 Ohm 1A
        // VoltageRAW = 12448, Voltage 194350  => vähennetään offset 12448 - 3605 = 8843
        // CurrentRAW = 2280, Current 35650 => 2280 - 937 = 1343
        
        // 32V 1.989A
        // VoltageRAW = 21485, Voltage 335320 => vähennetään offset 21485-3605 = 17880
        // CurrentRAW = 3670, Current  57350 => 3670 - 937 = 2733
        
        //0V 0A
        // VoltageRAW = 3605, Voltage 56276 
        // CurrentRAW = 937, Current 14600
        
        
        
        /*for (uint8_t samples = 0; samples <3; samples ++){
            //fprintf(&serial_port0, "\n\nCH:%d START\n",ch);
            MCP3424_SetConfig(2,18,1);
            uint32_t Voltage = measure();
            float measured = 15.609800758*Voltage; //4092000L/262143L*Voltage;
            fprintf(&serial_port0, "CH%d Sample:%d Voltage:%ld volt:%f TCNT2:%d\n",2, samples, Voltage, measured, TCNT2);
        }
        for (uint8_t samples = 0; samples <3; samples ++){
            //fprintf(&serial_port0, "\n\nCH:%d START\n",ch);
            MCP3424_SetConfig(4,18,1);
            uint32_t Voltage = measure();
            float measured = 15.609800758*Voltage; //4092000L/262143L*Voltage;
            fprintf(&serial_port0, "CH%d Sample:%d Voltage:%ld volt:%f TCNT2:%d\n",4, samples, Voltage, measured, TCNT2);
        }*/
        //ch ++;
        //if (ch > 4){
        //    ch = 1;   
        //}
        int16_t temp = 0;
        int16_t temp2 = 0;
        int16_t temp3 = 0;
        //Read temperature (without ROM matching)
        ds18b20read( &PORTC, &DDRC, &PINC, ( 1 << 3 ), NULL, &temp );
        ds18b20read( &PORTC, &DDRC, &PINC, ( 1 << 4 ), NULL, &temp2 );
        ds18b20read( &PORTC, &DDRC, &PINC, ( 1 << 5 ), NULL, &temp3 );
        float temp_calc = temp;
        temp_calc /= 20.0;
        
        float temp_calc2 = temp2;
        temp_calc2 /= 20.0;
        
        float temp_calc3 = temp3;
        temp_calc3 /= 20.0;
       // fprintf(&serial_port0, "TEMP:%.2f TEMP2:%.2f TEMP3:%.2f\n",temp_calc, temp_calc2, temp_calc3);
    }
    
    //Internal 1.1V => 1.0-1.2V 
    //Internal 2.56V => 2.33-2.79V
    
    
    
    //sei();
    fprintf_P(&serial_port0, PSTR("Reading ADC reference Voltage and Temperature\n"));
    fprintf_P(&serial_port0, PSTR("1V1 Reference Voltage:"));
    fprintf(&serial_port0, "%d",GetRAWVoltage(0x1E, AVCC_REF));
    fprintf_P(&serial_port0, PSTR(" RAW ADC\n"));
    
    fprintf_P(&serial_port0, PSTR("GND Reference Voltage:"));
    fprintf(&serial_port0, "%d",GetRAWVoltage(0x1F, AVCC_REF));
    fprintf_P(&serial_port0, PSTR(" RAW ADC\n"));
    
    while(1){
        fprintf_P(&serial_port0, PSTR("1V1 Reference Voltage:"));
        fprintf(&serial_port0, "%d",GetRAWVoltage(0x1E, AVCC_REF));
        fprintf_P(&serial_port0, PSTR(" RAW ADC\n"));
        _delay_ms(500);
    }
    fprintf_P(&serial_port0, PSTR("1V1 Reference Voltage:"));
    fprintf(&serial_port0, "%d",GetRAWVoltage(0x1E, AVCC_REF));
    fprintf_P(&serial_port0, PSTR(" RAW ADC\n"));
    
    //fprintf_P(&serial_port0, PSTR("GND Reference Voltage:"));
    //fprintf(&serial_port0, "%d",GetRAWVoltage(0x1F, AVCC_REF));
    //fprintf_P(&serial_port0, PSTR(" RAW ADC\n"));
    
    //fprintf_P(&serial_port0, PSTR("Chip Temperature:"));
    //fprintf(&serial_port0, "%f Degrees\n",readTemperature());
    
    fprintf_P(&serial_port0, PSTR("AVCC Voltage:"));
    fprintf(&serial_port0, "%d",AVCC_VOLTAGE);
    fprintf_P(&serial_port0, PSTR("mV\n"));
    
    printMenu();
    
    //initPWM(0);
    ///shutdownDAC(); //Disable PWMd
    //GTCCR = 0x00; //Start Counter
    
    //initPWM(10);
    while(1){}
    
    
    uint8_t setdone = 1;
    float lasterror = 9999.0f;
    while(1){
        //Handle serial commands always first
        if (command_received == 1){
            parseCommands();
            command_received = 0;
        }

    }
}

void initCommonPorts(void){
    //Init all led ports to input
    /*LED_DDR |= (1<<LED1_PIN_NUM);
    LED_DDR |= (1<<LED2_PIN_NUM);
    LED_DDR |= (1<<LED3_PIN_NUM);
    LED_DDR |= (1<<LED4_PIN_NUM);
    DISABLE_LED1
    DISABLE_LED2
    DISABLE_LED3
    DISABLE_LED4

    //All buttons as input
    BUTTON_DDR &= ~(1<<BUTTON1_PIN_NUM);
    BUTTON_DDR &= ~(1<<BUTTON2_PIN_NUM);
    BUTTON_DDR &= ~(1<<BUTTON3_PIN_NUM);
    
    //Disable pull-up resistors
    BUTTON_PORT &= ~(1<<BUTTON1_PIN_NUM);
    BUTTON_PORT &= ~(1<<BUTTON2_PIN_NUM);
    BUTTON_PORT &= ~(1<<BUTTON3_PIN_NUM); 
    */
    //EICRA = 0x05; //Any level will generate interrupt request on INT0 and INT1
    //EIMSK |= (1<<INT1)|(1<<INT0); //Activate INT0 and INT1 interrupts
    
    //For Button 1 need PCIE activation
    //PCICR |= (1<<PCIE2); //PCINT[23:16] //Activate any change interrupt on PCINT16-23
    //PCMSK2 |= (1<<PCINT20); //Enable only PCINT20 pin interrupts
}

void initTimer(void){
    TCCR2A |= (1<<WGM21); //CTC mode
    TCCR2B = 0x07; //1024 divider
    OCR2A = 156; //100,1602Hz timer
    TCNT2 = 0; //Clear counter
    
    //Interrupt configuration, OCR2A Compare Match
    TIMSK2 = 0x02; //Enable OCIEA
}


float readTemperature(void){
    //From datasheet
    // -45 = 242mV   -45=>+24 = 69 Delta =>  72mV   =>1,043478mv/Degree
    // +24 = 314mV   +24=>+85 = 61 Delta =>  66mV   =>1,082mV/Degree
    // +85 = 380mV 
    //Average 1,06 mV per degree.
    //This Chip has +70mV offset
    float voltage = GetVoltage(0x08, INT1V1_REF, 8);
    float temp =  ((voltage-242.0f)/1.06f) - 112;
    return temp;
}

void parseCommands(void){
    float value = readFloat(ring_read, ring_write);
    ring_read = ring_write; //Reset Ring buffer
    fprintf(&serial_port0, "Readed value:%f\n",value);
    uint8_t pwm = value;
    initPWM(pwm, pwm, pwm);
}

/*
    Reads floating point value from ringbuffer. 
    Function must be used because serial data is stored in to ring buffer
*/
float readFloat(uint8_t start, uint8_t stop){
    char buf[10] = {'\0','\0','\0','\0','\0','\0','\0','\0','\0','\0'};
    for (uint8_t i=0; i<10; i++){
        buf[i] = buffer[start++];
        if (start == stop){
           i = 10;
        }
    }
    return atof(buf);
}

void printMenu(void){
    fprintf_P(&serial_port0, PSTR("\n\nAtmega328P PWM DAC v1.0\n"));    
    fprintf_P(&serial_port0, PSTR("# Enter wanted DAC voltage and hit enter\n"));    
    fprintf_P(&serial_port0, PSTR("# Value can be between:0.0 - "));
    fprintf(&serial_port0, "%d mV\n",AVCC_VOLTAGE);
}


void initPWM(uint8_t pwmvalue1, uint8_t pwmvalue2, uint8_t pwmvalue3){
    TCCR0A = (0xA0 + 0x03); //A + B channels FAST PWM, UPDATE OCR0 at TOP
    DDRB |= (1<<PB4);
    PORTB &= ~(1<<PB4);
    DDRB |= (1<<PB3);
    PORTB &= ~(1<<PB3);
    TCCR0B = 0x01; //1 divider    
    OCR0A = pwmvalue1;
    OCR0B = pwmvalue2;
    
    TCCR1A = (0x80 + 0x01); //A channel only, 8-bit mode
    DDRD |= (1<<PD5);
    PORTD &= ~(1<<PD5);
    TCCR1B = 0x01 + 0x08; //1 divider    
    OCR1AL = pwmvalue3;
    OCR1AH = 0x00;
    
    // 16.169V 1.67 Ohm
    // 9.682035928 A 100% duty cycle
    // 0.09765625 × 9.682035928 = 0.9455A 
}

void shutdownPWM(uint8_t channel){
    if (channel == 1){
        TCCR0A &= ~(1<<0x80); //Channel A disable
        DDRB &= ~(1<<PB3);
        PORTB &= ~(1<<PB3);
        OCR0A = 0;
    }
    else if (channel == 2){
        TCCR0A &= ~(1<<0x40); //Channel B disable
        DDRB &= ~(1<<PB4);
        PORTB &= ~(1<<PB4);
        OCR0B = 0;
    }
    else if (channel == 3){
        TCCR1A &= ~(1<<0x80); //Channel B disable
        DDRD &= ~(1<<PB5);
        PORTD &= ~(1<<PB5);
        OCR1AL = 0;
    }
}


void increasePWM(uint8_t step){
    TOGGLE_LED2
    if (PWM_value+step < 0xFF){
        PWM_value += step;    
    }
    else{
        PWM_value = 0xFF;
    }
    setDAC_PWM(PWM_value);
}
void decreasePWM(uint8_t step){
    TOGGLE_LED3
    if (PWM_value-step > 0x00){
        PWM_value -= step;    
    }
    else{
        PWM_value = 0x00;
    }
    setDAC_PWM(PWM_value);
}

void setDAC_PWM(uint8_t value){
    if (pwm_init_done == 0){
        initPWM(value,value,value);
    }
    else{
        PWM_OCRA = value; //Init PWM to zero    
    }
    last_change = 0;
}

void shutdownDAC(void){
    DAC_PORT &= ~(1<<DAC_PIN);
    PWM_TCCRA = 0x00; //Clear register _TCCRA_value;
    PWM_TCCRB = 0x00;//Clear register TCCRB_value;
    PWM_OCRA = 0x00; //Set PWM value to Zero
    pwm_init_done = 0;
    DISABLE_LED2
    DISABLE_LED3
    //PWM_value = 0;
} 


float initADC(void){
    fprintf_P(&serial_port0, PSTR("Init ADC\n"));
    DAC_ADC_DIR = 0x00;  //All pins to input
    DAC_ADC_PORT = 0x00; //Pull-ups off
    DIDR0 = 0xFF; //DIsable digital input/output buffers
    _delay_ms(500);
    uint16_t ref = GetRAWVoltage(0x1E, AVCC_REF); //First measurement is always garbage
    //Loop until we get two same values in a row
    uint8_t maxwait = 0;
    while(maxwait < 200){
        _delay_ms(1000);
        uint16_t ref2 = GetRAWVoltage(0x1E, AVCC_REF); 
        fprintf_P(&serial_port0, PSTR("Calibrating AVCC_REF...\n"));
        if (ref == ref2){
            fprintf_P(&serial_port0, PSTR("ADC init DONE!\n"));
            ref = GetRAWVoltage(0x1E, AVCC_REF);
            AVCC_VOLTAGE = (1100.0f/(ref-INT1V1OFFSET)) * 1023;
            //AVCC_VOLTAGE = 4640
            fprintf(&serial_port0, "1.1V REF voltageRAW:%d 1.1V\n", ref);
            return ref;
        }
        ref = ref2;
        maxwait ++;
    }
    
    fprintf_P(&serial_port0, PSTR("ADC Calibration FAILED! 5000mW used as AVCC\n"));
    AVCC_VOLTAGE = 5000.0f;
    return 5000.0f;
}

uint16_t GetRAWVoltage(uint8_t Channel, uint8_t ADCRange){
    ADMUX = Channel+ADCRange;
    
    //If ADCRange changes, we must wait until external Capacitor is filled up
    if (current_reference != ADCRange){
        ADCSRA = 0b11000111; // 128 divider => 125KHz clock when 16MHz osc
        do {} while (bit_is_set(ADCSRA,6));
        _delay_ms(50);    
        current_reference = ADCRange;
    }
    
    ADCSRA = 0b11000111;  //128 clock divider
    do {} while (bit_is_set(ADCSRA,6));
    return ADCW;     
}

float GetVoltage(uint8_t Channel, uint8_t ADCRange, uint8_t numofsamples){
    float val = 0.0f;
    uint8_t samples = 0;
    GetRAWVoltage(Channel, ADCRange); //First measurement might be garbage
    _delay_ms(5);
        
    while(samples < numofsamples){
        val += GetRAWVoltage(Channel, ADCRange);
        samples++;
    }
    val = val/numofsamples;
    
    if (ADCRange == AVCC_REF){
        return (AVCC_VOLTAGE/1023.0f)*val;
    }
    else{
        return (1100.0f/1023.0f)*val;
    }
}
    
int put_char0(char c, FILE *stream){
     //if (c == '\n') put_char0('\r', stream);
     loop_until_bit_is_set(UCSR0A, UDRE0);
     UDR0 = c;
     return 0;
}

void USART_Init(FILE *port0, uint32_t br0){
    clearBuffer(buffer, BUFSIZE); //Initiaze Ring buffer with empty values /0
    
    uint32_t MYUBRR = 0;
    if (port0 != NULL){
        MYUBRR = F_CPU/(8*br0)-1;
        DDRD |= 0x02; //TXD0 output
        DDRD &= ~(1<<0); //RXD0 input
        UBRR0H = (uint8_t)(MYUBRR>>8);
        UBRR0L = (uint8_t)MYUBRR;
        UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0); //|(1<<TXCIE0);
        UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
        UCSR0A |= (1<<U2X0);
        *port0 = serial_port0;
        stdout = &serial_port0; //Required for printf init
    }
    
}
void USART_Transmit0( unsigned char data ){
    while ( !( UCSR0A & (1<<UDRE0)) );
    UDR0 = data;
}

void USART0_Flush( void ){
    unsigned char dummy;
    while ( UCSR0A & (1<<RXC0) ){
        dummy = UDR0;
        if (dummy){}
    }
}

unsigned char uart_getchar(void)
{
    while( !(UCSR0A & (1<<RXC0)) );
    return(UDR0);
}

//Serial data parsing

/*
    Reads ring buffer until it finds linefeed, carrier return or ring buffer is empty.
*/
void read_until_line_end(void){
    while(buffer[ring_read] != '\n' &&  buffer[ring_read] != '\r'  && ring_read != ring_write){
        ring_read++;
    }
}



