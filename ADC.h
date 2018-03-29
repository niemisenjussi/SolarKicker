//#include <avr/io.h> 

uint32_t GetOverSampledVoltage(uint8_t Sensor, uint8_t ADCRange);
uint16_t AVGVoltage(uint8_t Sensor, uint8_t ADCRange, uint8_t num_of_samples);
uint16_t GetVoltage(uint8_t Sensor, uint8_t ADCRange);
uint32_t Actuator_ADC(uint8_t Sensor, uint8_t ADCRange);
void initADC(void);
