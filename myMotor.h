#include "Arduino.h"
#ifndef _myMotor_h
#define _myMotor_h

const int motA1=7;
const int motA2=8;
const int motA3=9;
const int motA4=10;

unsigned int pwm_motA1; // Variable to contain servo value for A1
unsigned int pwm_motA2; // Variable to contain servo value for A2
unsigned int pwm_motA3; // Variable to contain servo value for A3
unsigned int pwm_motA4; // Variable to contain servo value for A4

int x1;
int x2;
int y1;
int y2;
int xrate;
int yrate;
int damp=0;


void motor_setup();
void motor_update();
void my_yaw();
void stabilize(); // Function which will run my
void throttle();
int find_xp();
int find_xn();
int find_y();


#endif
