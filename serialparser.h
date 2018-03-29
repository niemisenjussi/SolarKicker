#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>

void printerr(void);
void read_until_line_end(void);
void initSerialParser(FILE *serialport);
void parseCommands(void);
uint16_t readInt16(uint8_t start, uint8_t stop);
double readFloat(uint8_t start, uint8_t stop);
uint8_t findParameter(char startchar, char stopchar, char secondstopchar, uint8_t maxlength, uint8_t *start, uint8_t *stop);


