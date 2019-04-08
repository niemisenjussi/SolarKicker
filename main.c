#include <avr/io.h>    /* Device specific declarations */
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "i2c_master.h"
#include <stdlib.h>

#include <ds18b20.h>


#define DISABLE_LED1 LED_PORT |= (1<<LED1_PIN_NUM);
#define ENABLE_LED1 LED_PORT &= ~(1<<LED1_PIN_NUM);
#define TOGGLE_LED1 LED_PORT ^= (1<<LED1_PIN_NUM);

#define AVCC_REF 0x40
#define INT2V56_REF 0xC0
#define INT1V1_REF 0xC0

//Serial port 
#define BUFSIZE 0xFF //Ring buffer size. Use 8 bit length so it has automatic overflow handling

//Command modes
#define CURVE_MEASUREMENT 1
#define NORMAL_MODE 0

//Command chars
#define CURVE_MEASUREMENT_COMMAND 'C'
#define NORMAL_MODE_COMMAND 'N'

//ADC stuff
float initADC(void);
uint16_t GetRAWVoltage(uint8_t Channel, uint8_t ADCRange);
float GetVoltage(uint8_t Channel, uint8_t ADCRange, uint8_t numofsamples);

//PWM stuff
void initPWM(uint8_t pwmvalue1, uint8_t pwmvalue2, uint8_t pwmvalue3);
void shutdownPWM(uint8_t channel);
void mppt_control(float current_power, float voltage);
void setPWM(uint8_t channel, uint16_t pwm);


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

uint8_t isConversionFinished(void);
void newConversion(void);
int32_t measure(void);

float readFloat(uint8_t start, uint8_t stop);
void read_until_line_end(void);
void parseCommands(void);

//Common functions
void initTimer(void);

//##### Global variables #####
volatile uint8_t pwm_init_done = 0;
volatile uint8_t command_received = 0;

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

//RTC incremented counter
volatile uint8_t ticktimer = 0;
volatile uint8_t ready_send = 0;

#define ADCBITS 16 //Remember to adjust scaling functions 

//####### Interrupt vectors #######
ISR(USART0_RX_vect) {
    volatile char rchar = USART_Receive0();
    if (rchar == '\n' || rchar == '\r'){
        command_received = 1; 
    }
    else{
        buffer[ring_write++] = rchar;    
    }
}


////Tick timer interrupt
//ISR(TIMER2_COMPA_vect){

//Once per second timer
ISR(TIMER2_OVF_vect){
//fprintf_P(&serial_port0, PSTR("T2CA\n"));
    //ticktimer ++;
    //if (ticktimer > 125){
    ready_send = 1;
    //}
}


uint8_t mcp3424_address = 0xD0;
uint8_t _resolution;
uint8_t _mode;
uint8_t _cfgbyte;
uint8_t _PGA;
uint8_t _buffer[5] = {0,0,0,0,0};

#define UP 1
#define DOWN 2
#define MPPT_STEPS_1 9
#define MPPT_STEPS_2 9
#define MPPT_STEPS_3 11 
#define MPPT_STEPS_4 9
#define MIN_PWM 1 //minimum pulse width max 256
#define MIN_PWM3 8 //minimum pwm3 channel value, this is 10 bit channel

#define MPPT_LOW_VOLTAGE 77.0
#define MPPT_HIGH_VOLTAGE 95.0

uint8_t mppt_steps[4] = {MPPT_STEPS_1, MPPT_STEPS_2, MPPT_STEPS_3, MPPT_STEPS_4};
uint8_t mppt_kicker[4] = {15, 16, 20, 25};
volatile int16_t pwm_master = 20;
volatile uint8_t mppt_dir = UP;
volatile uint8_t mppt_step = 0;
volatile uint8_t mppt_high = 0;
volatile float mppt_peak_power = 0;

volatile uint8_t current_command = 0;


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
    if (status == 0){

    }
	i2c_stop();
}

void newConversion(){
    i2c_start(mcp3424_address);
    uint8_t status = i2c_write((_cfgbyte+128));
    if (status == 0){

    }
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
    if (readstate == 0){

    }
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
    if (status == 0){

    }
    i2c_stop();
}

void adjust_PWM(int8_t amount){
    uint8_t pwm1 = 0;
    uint8_t pwm2 = 0;
    uint16_t pwm3 = 0;
    if ((pwm_master + amount) < 0){
        pwm_master = 1;
    }
    else if ((pwm_master + amount) > (0x3FF+0xFF+0xFF)){//0x2FD){
        pwm_master = 0x3FF+0xFF+0xFF;//0x2FD;
    }
    else{ 
        pwm_master += amount;
    }

    if (pwm_master >= (0x3FF+0xFF)){
        pwm3 = 0x3FF;
        pwm2 = 0xFF;
        pwm1 = pwm_master - (0x3FF+0xFF);//0x1FE;
        if (pwm1 < MIN_PWM){
            pwm1 = MIN_PWM;
        }
    }
    else if (pwm_master >= 0x3FF){
        pwm3 = 0x3FF;
        pwm2 = pwm_master - 0x3FF;
        pwm1 = 0x00;
        if (pwm2 < MIN_PWM){
            pwm2 = MIN_PWM;
        }
    }
    else{
        pwm3 = pwm_master;
        pwm2 = 0x00;
        pwm1 = 0x00;
        if (pwm3 < MIN_PWM3){
            pwm3 = MIN_PWM3;
            pwm_master = pwm3;
        }
    }
    //fprintf(&serial_port0, "PWM1:%d, PWM2:%d, PWM3:%d\n", pwm1, pwm2, pwm3);
    setPWM(1, pwm1);
    setPWM(2, pwm2);
    setPWM(3, pwm3);
}

/*
    uint16_t pwm_master = 0;
    uint8_t mppt_dir = 0;
    uint8_t mppt_step = 0;
    uint8_t mppt_high = 0;
    float mppt_peak_power = 0;
*/

void mppt_control(float current_power, float voltage){
    uint8_t mppt_step_lookup = 0;
    if (pwm_master >= 0x3FF){ //When using last two PWM channels
        mppt_step_lookup = 3;
    }
    else if (pwm_master >= 0x2FF){ //10bit mode 
        mppt_step_lookup = 2;
    }
    else if (pwm_master >= 0x1FF){ //10bit mode
        mppt_step_lookup = 1;
    }
    else{
        mppt_step_lookup = 0; //10bit mode
    }

    //Make sure we are inside MPPT tracking range before fine tuning
    if (voltage < MPPT_LOW_VOLTAGE){
        if (voltage < MPPT_LOW_VOLTAGE - 20.0 && current_power > 60){
            adjust_PWM(-90);
        }
        else if (voltage < MPPT_LOW_VOLTAGE - 10.0 && current_power > 40){
            adjust_PWM(-45);
        }
        else if (voltage < MPPT_LOW_VOLTAGE - 5.0 && current_power > 20){
            adjust_PWM(-15);
        }
        else{
            adjust_PWM(-4);
        }
        mppt_dir = DOWN;
        mppt_step = 0;
        mppt_peak_power = current_power;
    }
    else if (voltage > MPPT_HIGH_VOLTAGE){
        if (voltage > MPPT_HIGH_VOLTAGE + 20.0 && current_power > 60){ //Over 10volts over high limit
            adjust_PWM(90);
        }
        else if (voltage > MPPT_HIGH_VOLTAGE + 10.0 && current_power > 40){ //Over 10volts over high limit
            adjust_PWM(45);
        }
        else if (voltage < MPPT_HIGH_VOLTAGE + 5.0 && current_power > 20){
            adjust_PWM(15);
        }
        else{
            adjust_PWM(4);
        }
        mppt_dir = UP;
        mppt_step = 0;
        mppt_peak_power = current_power;
    }
    else{
        if (mppt_dir == UP){ //Climing uphill
           // fprintf(&serial_port0,"Uphill CP:%.2f mppt:%.2f\n", current_power, mppt_peak_power);
            adjust_PWM(1);
            if (mppt_step < mppt_steps[mppt_step_lookup] ){ //Track maximum power point, n.sample window
                if (current_power > mppt_peak_power){
                    mppt_step = 0; //Reset points if higher point detected, idea is to find highest point
                    mppt_peak_power = current_power;
                }
                mppt_step ++;
            }
            else{ //All samples collected, do adjustment
                mppt_dir = DOWN; //
                mppt_step = 0;
                mppt_peak_power = current_power; //set new maxpoint to current poin;
            }
        }
        else{ //Going Downhi
           // fprintf(&serial_port0, "Downhill CP:%.2f mppt:%.2f\n", current_power, mppt_peak_power);
            adjust_PWM(-1);
            if (mppt_step < mppt_steps[mppt_step_lookup] ){
                if (current_power > mppt_peak_power){
                  //  fprintf(&serial_port0, "highest power found\n");
                    mppt_step = 0;
                    mppt_peak_power = current_power;
                }
                mppt_step ++;
            }
            else{ //We have found maximum point and enough off 
                mppt_dir = UP;
                mppt_step = 0;
                mppt_peak_power = current_power; //set new maxpoint to current point
            }
        }
    }
}

int main (int argc, char *argv[])
{
    _delay_ms(500);
    USART_Init(&serial_port0, 115200);
    //initCommonPorts();
    clearBuffer(buffer, BUFSIZE);
    i2c_init();
    USART0_Flush();    
    initADC();
    initTimer();
    sei();  
    //Init PWM and start counter, 0 will shutdown all 
    
    fprintf_P(&serial_port0, PSTR("AVCC Voltage:"));
    fprintf(&serial_port0, "%d",AVCC_VOLTAGE);
    fprintf_P(&serial_port0, PSTR("mV\n"));
    
    fprintf(&serial_port0, "+14.6V Rail voltage:%.2fmV\n", GetVoltage(0x00, AVCC_REF, 16)*21); //14.54V 
    //i2c_start(0xC0); // add Address bits on top off that
    //i2c_write(0x03); // set pointer to X axis MSB
    //i2c_stop();
    
    //Channel1 = VCC 5/1K jako => 5V/6K = 0.833V => 5V VCC
    //Channel2 = PANEL voltage 100K/1K jako => 
    //Channel4 = Panel Current 10K/10K jako => puolet jännitteestä. CH- kytketty VCC/2 => TODO korjaa kytkentä suoraan VIOUT=>CH4+

    shutdownPWM(1);
    shutdownPWM(2);
    shutdownPWM(3);
    _delay_ms(100);
    


    MCP3424_SetConfig(1, ADCBITS, 1);
    //Measure current sensor lowest voltage when no current flows through resistors
    int32_t curoffset = 99999;
    fprintf(&serial_port0, "Calibrating current sensor zero-point...\n");
    MCP3424_SetConfig(4, ADCBITS, 1);
    fprintf_P(&serial_port0, PSTR("Waiting 5 seconds before measurement\n"));
    _delay_ms(5000);
    fprintf_P(&serial_port0, PSTR("Taking 50 samples, then 50 to average\n"));
    for (uint8_t i=0; i< 100; i++){ //4111111
        int32_t zerovolt = measure();
        if (i > 49){
            //curoffset += zerovolt;
            fprintf(&serial_port0, "Offset%d: %ld\n", i, zerovolt);
            if (curoffset > zerovolt){
                curoffset = zerovolt;
            }
        }
    }
    
    /*if (curoffset > 1400){
        curoffset = 1400; //well known value
    }*/
    if (curoffset > 205){
        curoffset = 205; //Well known value. 18Bit mode 1400 or lower
    }

    //curoffset /= 50;
    fprintf(&serial_port0, "Current sensor offset: %ld\n",curoffset);
    
    
    initPWM(0,0,0);
    setPWM(1,0);
    setPWM(2,0);
    setPWM(3,0);
    GTCCR = 0x00; 
    
    /*fprintf(&serial_port0, "Calibrating CH3 ZEro volt...\n");
    MCP3424_SetConfig(3,18,1);
    for (uint8_t i=0; i< 20; i++){
        fprintf(&serial_port0, "CH3 zerovolt: %ld\n",measure());
    }*/
   /* 
    for (uint8_t d = 0; d <32; d++){
        MCP3424_SetConfig(2,18,1); //CHANNEL 2 = Panel Voltage meter 100/1 resistor divider
        float measuredVoltage18 = (measure())/628.4f;
         
        MCP3424_SetConfig(2,16,1); //CHANNEL 2 = Panel Voltage meter 100/1 resistor divider
        float measuredVoltage16 = (measure())/628.4f;

        MCP3424_SetConfig(2,14,1); //CHANNEL 2 = Panel Voltage meter 100/1 resistor divider
        float measuredVoltage14 = (measure())/628.4f;

        fprintf(&serial_port0, "18bit:%.2f 16bit:%.2f 14bit:%.2f\n",measuredVoltage18, measuredVoltage16, measuredVoltage14);
    }*/
    
   // uint8_t ch = 2;
   // uint16_t voltoffset = 3605;
    
    //MCP3424_SetConfig(ch,18,1);

    float mosfet_temp1 = 0;
    float mosfet_temp2 = 0;
    float mosfet_temp3 = 0;
    //uint8_t pwmstate = 1;
    //uint8_t round = 0;
    //uint8_t dir = 0;
    //float lastpower = 0;
    float power = 0;
    fprintf_P(&serial_port0, PSTR("READY\n"));
    uint8_t tempmeasurement = 0;
    

    uint16_t kicker = 0; 
    //sei(); //Start interrupts
    //Convert temperatures initial
    ds18b20convert( &PORTC, &DDRC, &PINC, ( 1 << 3 ), NULL );
    ds18b20convert( &PORTC, &DDRC, &PINC, ( 1 << 4 ), NULL );
    ds18b20convert( &PORTC, &DDRC, &PINC, ( 1 << 5 ), NULL );
    
    uint16_t mppt_pwm_store = 0; //This is used to store master_pwm when training is happening
    //uint16_t current_curve_pwm = 0;
    uint8_t curve_mode = 0; //not running
   
    uint8_t adjust_delay = 0;
    while(1){
        //initPWM(pwm1, pwm2, pwm3);
        if (command_received == 1){
            parseCommands();
            command_received = 0;
        }
        
		//Delay (sensor needs time to perform conversion)
	    	
        MCP3424_SetConfig(2,ADCBITS,1); //CHANNEL 2 = Panel Voltage meter 100/1 resistor divider
        float measuredVoltage = (measure())/628.4f*4.0f;
        
        MCP3424_SetConfig(4,ADCBITS,1);
        float Current = measure()-curoffset;
        
        float measuredCurrent = (Current)/1.555f*4.0f;   //1.385 this was 
        power = measuredVoltage * (measuredCurrent/1000);

        if (power < 0){
            power = power * -1;
        }
        if (tempmeasurement > 20){
            int16_t temp = 0;
            int16_t temp2 = 0;
            int16_t temp3 = 0;

            ds18b20read( &PORTC, &DDRC, &PINC, ( 1 << 3 ), NULL, &temp );
            ds18b20read( &PORTC, &DDRC, &PINC, ( 1 << 4 ), NULL, &temp2 );
            ds18b20read( &PORTC, &DDRC, &PINC, ( 1 << 5 ), NULL, &temp3 );
            mosfet_temp1 = temp;
            mosfet_temp2 = temp3;
            mosfet_temp3 = temp2;

            //temperature must be divided by two and by default its 10x too much
            mosfet_temp1 /= 20.0;
            mosfet_temp2 /= 20.0;
            mosfet_temp3 /= 20.0;
            tempmeasurement = 0; //Reset counter

            ds18b20convert( &PORTC, &DDRC, &PINC, ( 1 << 3 ), NULL );
            ds18b20convert( &PORTC, &DDRC, &PINC, ( 1 << 4 ), NULL );
            ds18b20convert( &PORTC, &DDRC, &PINC, ( 1 << 5 ), NULL );
        }

        //Send data once per second
        if (ready_send == 1){
            fprintf_P(&serial_port0, PSTR("S;")); //Send Set mark so this is recorded
            ready_send = 0;
        }
        else if (curve_mode == 1){
            fprintf_P(&serial_port0, PSTR("C;"));
        }

        fprintf(&serial_port0, "%.2f;%.2f;%.2f;%d;",measuredVoltage, measuredCurrent, power, pwm_master);
        fprintf(&serial_port0, "%.2f;%.2f;%.2f\n",mosfet_temp1, mosfet_temp2, mosfet_temp3);

        //Curve measuremen mode
        if (current_command == CURVE_MEASUREMENT){
            if (curve_mode == 0){
                fprintf_P(&serial_port0, PSTR("CURVE_START\n"));
                mppt_pwm_store = pwm_master;
                curve_mode = 1;
                pwm_master = 0;
            }
            else if (curve_mode == 1){
                if (pwm_master < 0x3FF){ //First channel is 10bit => 4 times resolution
                    adjust_PWM(8);
                }
                else if (pwm_master < (0x3FF+0xFF+0xFF)){ //Last two uses 8bit resolution
                    adjust_PWM(2); //plus one
                }
                else{
                    current_command = NORMAL_MODE;
                    curve_mode = 0;
                    pwm_master = mppt_pwm_store;
                    adjust_PWM(0); //Set values
                    fprintf_P(&serial_port0, PSTR("CURVE_STOP\n"));
                }
            }
        }
        else{
            if (adjust_delay > 0){ //Faster response. This was 3
                mppt_control(power, measuredVoltage);
                adjust_delay = 0;
            }
            
            uint8_t mppt_kicker_amount = 0;
            if (pwm_master >= 0x3FF){
                mppt_kicker_amount = 3;
            }
            else if (pwm_master >= 0x2FF){
                mppt_kicker_amount = 2;
            }
            else if (pwm_master >= 0x1FF){
                mppt_kicker_amount = 1;
            }
            
            if (kicker == 1450){
                adjust_PWM(mppt_kicker[mppt_kicker_amount]); //Kick panel so we dont get stuck 
            }
            else if (kicker > 2900){
                adjust_PWM(mppt_kicker[mppt_kicker_amount] * -1); //Kick down
                kicker = 0;
            }
            
            kicker ++;
            tempmeasurement ++;
            adjust_delay ++;
        }

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
    }
    
}

void initTimer(void){
    ASSR = 0x20; //Enable async mode Timer 2 Ext oscillator
    TCCR2A = 0x00; //Normal mode
    //TCCR2A |= (1<<WGM21); //CTC mode
    TCCR2B = 0x05; //128 divider => 32.768 clock => 1 per sec
    //OCR2A = 125; //125Hz timer, this goes excatly 125 times 15625
    TCNT2 = 0; //Clear counter
    
    //Interrupt configuration, OCR2A Compare Match
    TIMSK2 = 0x01; //Enable Overflow interrupt enable
}


void parseCommands(void){
    if (buffer[ring_read] == CURVE_MEASUREMENT_COMMAND){
        current_command = CURVE_MEASUREMENT;
    }
    //float value = readFloat(ring_read, ring_write);
    ring_read = ring_write; //Reset Ring buffer
    //fprintf(&serial_port0, "Readed value:%f\n",value);
    //uint8_t pwm = value;
    //initPWM(pwm, pwm, pwm);
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


void initPWM(uint8_t pwmvalue1, uint8_t pwmvalue2, uint8_t pwmvalue3){
    TCCR0A = (0xA0 + 0x03); //A + B channels FAST PWM, UPDATE OCR0 at TOP
    DDRB |= (1<<PB4);
    PORTB &= ~(1<<PB4);
    DDRB |= (1<<PB3);
    PORTB &= ~(1<<PB3);
    TCCR0B = 0x02; //1=1 divider , 2=8 divider
    OCR0A = pwmvalue1;
    OCR0B = pwmvalue2;
    
    DDRD |= (1<<PD5);
    PORTD &= ~(1<<PD5);
    
    //8-bit mode
    //TCCR1A = (0x80 + 0x01); //A channel only, 8-bit mode Fast PWM
    //TCCR1B = 0x01 + 0x08; //1 divider FAST PWM 
    
    //10-bit mode
    TCCR1A = (0x80 + 0x03); //A channel only, 10-bit mode Fast PWM
    TCCR1B = 0x01 + 0x08; //1 divider FAST PWM 
    
    OCR1AL = pwmvalue3;
    OCR1AH = 0x00;
    
    // 16.169V 1.67 Ohm
    // 9.682035928 A 100% duty cycle
    // 0.09765625 × 9.682035928 = 0.9455A 
}

void setPWM(uint8_t channel, uint16_t pwm){
    if (channel == 1){
        if (pwm == 0){
            TCCR0A &= ~(1<<7); //Channel A disable
            DDRB |= (1<<PB3); //Output
            PORTB &= ~(1<<PB3); //Pull down
            return;
        }
        TCCR0A |= (1<<7); //Enable Channel A
        uint8_t pwmval = pwm;
        OCR0A = pwmval;
    }
    else if (channel == 2){
        if (pwm == 0){
            TCCR0A &= ~(1<<5); //Channel B disable
            DDRB |= (1<<PB4);
            PORTB &= ~(1<<PB4);
            return;
        }
        TCCR0A |= (1<<5);
        uint8_t pwmval = pwm;
        OCR0B = pwmval;
    }
    else if (channel == 3){
        if (pwm == 0){
            TCCR1A = 0x00;
            DDRD |= (1<<PD5);
            PORTD &= ~(1<<PD5);
            return;
        }
        TCCR1A = (0x80 + 0x03);
        OCR1A = pwm;
       //OCR1AL = pwm;
       //OCR1AH = 0x00;
    }
}

void shutdownPWM(uint8_t channel){
    if (channel == 1){
        TCCR0A &= ~(1<<7); //Channel A disable
        TCCR0B = 0x00;
        DDRB &= ~(1<<PB3);
        PORTB &= ~(1<<PB3);
        OCR0A = 0x00;
    }
    else if (channel == 2){
        TCCR0A &= ~(1<<6); //Channel B disable
        TCCR0B = 0x00;
        DDRB &= ~(1<<PB4);
        PORTB &= ~(1<<PB4);
        OCR0B = 0x00;
    }
    else if (channel == 3){
        //TCCR1A &= ~(1<<7); //Channel B disable
        TCCR1A = 0x00;
        TCCR1B = 0x00;
        DDRD &= ~(1<<PD5);
        PORTD &= ~(1<<PD5);
        OCR1AL = 0x00;
        OCR1AH = 0x00;
        TCNT1 = 0x00;
    }
}



float initADC(void){
    fprintf_P(&serial_port0, PSTR("Init ADC\n"));
    DDRA = 0x00; 
    PORTA = 0x00; //Pull-ups off
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



