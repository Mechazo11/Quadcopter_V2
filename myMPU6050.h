/*
///////////////////////
  // constants or parameters:
  // timeConstant - bandwidth of filter (1/sec). Need to tune this to match sensor performance.
  // T - iteration rate of the filter (sec)
  ///////////////////////
  // variables:
  // int_x1 (filterTerm[0]) - input to the first integrator (deg/sec/sec)
  // int_x2 (filterTerm[1]) - input to the second integrator (deg/sec)
  // int_y1 (filterTerm[2]) - output of the first integrator (deg/sec). This needs to be saved each iteration
  //////////////////////
  // inputs:
  // gyro - gyro output (deg/sec)
  // accel - accelerometer input to filter (deg)
  // x_accel - accelerometer output in x-axis (g)
  // z_accel - accelerometer output in z-axis (g)
  // accel_ang - derived angle based on arctan(x_accel,z_accel), (deg)
  //////////////////////
  // outputs:
  // filter - complementary filter output (and output of second integrator), (deg)
  //            This also needs to be saved each iteration.
  ///////////////////////

*/

#include "Arduino.h"
#ifndef _myMPU6050_h
#define _myMPU6050_h
#define timeConstant 0.001
//#define timeConstant (double)((micros() - timer) / 1000000)
#define filter_diff 0.075


// Create the Kalman instances
Kalman kalmanX; 
Kalman kalmanY;


/* IMU Data */
int16_t accX, accY, accZ;
//int16_t tempRaw;
int16_t gyroX, gyroY, gyroZ;

double accXangle, accYangle; // Angle calculate using the accelerometer
double gyroXangle, gyroYangle; // Angle calculate using the gyro
double compAngleX, compAngleY; // Stores Pitch and Roll for both first order and second order filter
double kalAngleX, kalAngleY; // Calculate the angle using a Kalman filter
double filter_xterm[3] = {0,0,0};// Second Order Complementary Filter Varialbes
double filter_yterm[3] = {0,0,0};
float a = 0.850;// Ratio of gyro reading that will be taken
double s_kalAngleX;
double s_kalAngleY;

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

void mpu6050_setup();
void getPR(); // Function prototype to get pitch and roll

#endif
