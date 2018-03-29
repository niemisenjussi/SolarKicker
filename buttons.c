#include "buttons.h"

#define AUTO_MANUAL_SWITCH_DIR DDRD 
#define AUTO_MANUAL_SWITCH_PORT PORTD
#define AUTO_MANUAL_SWITCH_INPUT PIND
#define AUTO_MANUAL_SWITCH_PIN 2

#define TILT_UP_DIR DDRB
#define TILT_UP_PORT PORTB
#define TILT_UP_INPUT PINB
#define TILT_UP_PIN 2

#define TILT_DOWN_DIR DDRB
#define TILT_DOWN_PORT PORTB
#define TILT_DOWN_INPUT PINB
#define TILT_DOWN_PIN 1

#define TURN_LEFT_DIR DDRB
#define TURN_LEFT_PORT PORTB
#define TURN_LEFT_INPUT PINB
#define TURN_LEFT_PIN 0

#define TURN_RIGHT_DIR DDRD
#define TURN_RIGHT_PORT PORTD
#define TURN_RIGHT_INPUT PIND
#define TURN_RIGHT_PIN 4

void initButtons(void){
    AUTO_MANUAL_SWITCH_DIR &= ~(1<<AUTO_MANUAL_SWITCH_PIN); //Input
    AUTO_MANUAL_SWITCH_PORT &= ~(1<<AUTO_MANUAL_SWITCH_PIN); //Pulldown
    
    TILT_UP_DIR &= ~(1<<TILT_UP_PIN); //Input
    TILT_UP_PORT &= ~(1<<TILT_UP_PIN); //Pulldown
    
    TILT_DOWN_DIR &= ~(1<<TILT_DOWN_PIN); //Input
    TILT_DOWN_PORT &= ~(1<<TILT_DOWN_PIN); //Pulldown
    
    TURN_LEFT_DIR &= ~(1<<TURN_LEFT_PIN); //Input
    TURN_LEFT_PORT &= ~(1<<TURN_LEFT_PIN); //Pulldown
    
    TURN_RIGHT_DIR &= ~(1<<TURN_RIGHT_PIN); //Input
    TURN_RIGHT_PORT &= ~(1<<TURN_RIGHT_PIN); //Pulldown   
}

uint8_t readAutoManualState(void){
    if (AUTO_MANUAL_SWITCH_INPUT & (1<<AUTO_MANUAL_SWITCH_PIN)){
        return 1;
    }
    else{
        return 0;
    }
}

uint8_t readTiltButtonState(void){
    if (TILT_UP_INPUT & (1<<TILT_UP_PIN)){
        return 1;
    }
    else if (TILT_DOWN_INPUT & (1<<TILT_DOWN_PIN)){
        return 2;
    }
    else{
        return 0;
    }
}

uint8_t readTurnButtonState(void){
    if (TURN_LEFT_INPUT & (1<<TURN_LEFT_PIN)){
        return 1;
    }
    else if (TURN_RIGHT_INPUT & (1<<TURN_RIGHT_PIN)){
        return 2;
    }
    else{
        return 0;
    }
}




 
