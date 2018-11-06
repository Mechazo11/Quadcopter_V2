#include "Arduino.h"
#ifndef _myMagnetometer_h
#define _myMagnetometer_h

// Objects and Instances
MPU6050 accelgyro;
HMC5883L mag;

int16_t mx; 
int16_t my; 
int16_t mz;

void mymag_setup();
void get_yaw();

#endif
