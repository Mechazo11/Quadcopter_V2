#include "Arduino.h"
#ifndef _myPID_h
#define _myPID_h

// PID tuning parameters for stabilization mode
double Kp=0.0; // P value for pitch,roll
double Ki=0.0; // I value for pitch, roll
double Kd=0.0;

double zKp=Kp; // P value for yaw
double zKi=Ki; // I value for yaw
double zKd=Kd; // D value for yaw

double pcrr=0; // Callibration offset for pitch
double rcrr=0; // Callibration offset for roll

unsigned long  newtime;
unsigned long  oldtime;
double dt;
double d1;
double d2;

double hpitch=0; // Desired pitch
double hroll=0; // Desired roll
double hyaw=0; // Desired Heading
double halt=0; // Initial height

double npitch=0.0;
double nroll=0.0;
  

//Define Variables for Pitch Axis
double perror=0, pOutput=0, perrsum=0, perrlast=0.0;
//Define Variables for Roll Axis
double rerror=0, rOutput=0, rerrsum=0, rerrlast=0.0;
//Define Variables for Yaw Axis
double yerror=0, yOutput=0, yerrsum=0, yerrlast=0.0;



double low_pitch=0.0; 
double low_pitch_sum=0.0;
double low_pitch_last=0.0;

double low_roll=0.0; 
double low_roll_sum=0.0;
double low_roll_last=0.0;

double low_dir=0.0;
double low_sum=0.0;
double low_last=0.0;


void quad_home(); // This function will set the home values
// at the very beginning
void PID_compute();

#endif
