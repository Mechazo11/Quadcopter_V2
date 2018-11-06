#include "Arduino.h"
#ifndef _myVarometer_h
#define _myVarometer_h



#define ADDRESS 0x77 //0x77

// There is a beautiful flow diagram
//desciribing how many variables and how to get 
// the most reliable data
uint32_t D1 = 0;
uint32_t D2 = 0;
int64_t dT = 0;
int32_t TEMP = 0;
int64_t OFF = 0; 
int64_t SENS = 0; 
int32_t P = 0;
uint16_t C[7]; 

float Temperature;
float Pressure;
const float sea_press = 1013.25;

void myVaro_setup();
void get_height();
float getAltitude(float Pressure, float Temperature);
long getVal(int address, byte code);
void initial(uint8_t address);

#endif
