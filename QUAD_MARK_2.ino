/*
              Project: Quadcopter Mark 2
              Author: Azmyin Md. Kamal
              Date: 9th September 2014
              Version: 1.0

Goals of Version 1
1) Receive commands from NRF24L01 connected to a UNO
2) Self-Align Pitch and Roll using Kalman Filter
and a parallel PID 
3) Control Throttle and Yaw using UNO
4) Avoid obstacles using Sonar sensor
5) Achieve a simplistic Altitude hold using Varometer

Notes
1) KalmanAngleX is Yaw 
2) KalmanAngleY is Roll
3) Magnetometers and Altimeters data
will be more used to find the relative difference
between the past and newest data
to find the difference or for yaw control 
the target data minus the new data to get the 
throttle signal required to drive it upto that
position
4) CONNECTIONS: nRF24L01 Modules See:
http://arduino-info.wikispaces.com/Nrf24L01-2.4GHz-HowTo
   i - GND
   ii - VCC 3.3V !!! NOT 5V
   iii - CE to Arduino pin 49
   iv - CSN to Arduino pin 53
   v - SCK to Arduino pin 52
   vi - MOSI to Arduino pin 51
   vii - MISO to Arduino pin 50
   viii - Not required for this experiment

5) The debug define lines will toggel the debug codes
6) I will lock the orientaiton of quad between -15 to 15 degree
for both pitch and roll
*/

// Libraries and Definitions


#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <HMC5883L.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "Kalman.h"
#include "myMPU6050.h"
#include "myMagnetometer.h"
#include "myVarometer.h"
#include "nrf_receive.h"
#include "myMotor.h"
#include "myPID.h"

#define Debug_GYROACCEL
//#define Debug_Compass
//#define Debug_Varometer
//#define Debug_NRF
#define Debug_Motor
//#define Debug_Home
void rate();
// whose output will be four PWM values based on
// nrf commands and so on
void show_debug();

double h; // Stores the height of sensor in meter
double h_cm;// Stores height of sensor in cm

double dir; // Gets the heading in degrees
int i1=0;
unsigned long time1=0;
unsigned long time2=0;
unsigned long dtime=0;

boolean t_p_flag=false;// Throttle plus flag
boolean t_m_flag=false;
boolean dyaw=false; // Flag to control the direction of the yaw

int ystick; // Stores the new value of Y stick
int xstick; // Stores the new value of X stick

int dthrot=2; // Change this to increase or decrease the value by which throttle will be changed
int pwm_dyaw=2; // Change this value to change the strength of manual yaw correction
int gthrot=90; // Global Throttle value
int con=90;// turn this into a flag and use it at the very beginning

int mot_min=110; // This has been found from experiment in the PID loops
int mot_max=180;
int takeoff=120;
int yaw_loop;
boolean f1=false;

//Smoothing 
double p_smooth(double sensVal, float filterVal, double smoothedVal);
double r_smooth(double sensVal, float filterVal, double smoothedVal);
float p_fil=0.5;
float r_fil=p_fil;
const int samplerate=500; // Number of sample that will be taken in callibration phase
void setup()
{
Serial.begin(115200);
Wire.begin();
nrf_setup(); // Sets up nrf24l01 with MEGA in receive mode
mymag_setup(); // Sets up HMC588L compass
mpu6050_setup();// Sets up MPU6050 according to TJK's code
myVaro_setup(); // Sets up MS5611 Varometer
Serial.println("Calibrating..");
quad_home(); // Home values
Serial.println("End Of Callibration");
motor_setup(); // Sets up motors
}
// End of setup()

void loop()
{
//time1=millis();
getPR(); // Get pitch and roll
//s_kalAngleX=p_smooth(kalAngleX,p_fil,s_kalAngleX); 
//s_kalAngleY=r_smooth(kalAngleY,r_fil,s_kalAngleY);
//time2=millis();
//Serial.print("Loop time needed: "); Serial.println(dtime=time2-time1);
get_yaw();// Get yaw
//get_height(); // Gets altitdue and temperature

nrf_getcmd();// Get command for rate or stabilize mode
show_debug();
throttle();

if (data[0]==0x01)
  { 
    rate();
  }
if (data[0]==0x00)
  {
    stabilize();
  }

motor_update();
}
// End of main loop


double p_smooth(double sensVal, float filterVal, double smoothedVal)
{
smoothedVal=(sensVal*(1-filterVal))+(smoothedVal*filterVal);
return smoothedVal;
}

double r_smooth(double sensVal, float filterVal, double smoothedVal)
{
smoothedVal=(sensVal*(1-filterVal))+(smoothedVal*filterVal);
return smoothedVal;
}

void stabilize()
{
if (data[1]==0x02&&(data[2]>=500&&data[2]<=510))// X and Y at center
  {
          t_p_flag=false;
          t_m_flag=false;
          PID_compute();
          //motor_update(); 
  }
}
// End of stabilize()

void rate()
{
xrate=0;
yrate=0;
  
if (data[0]==0x01&&data[1]==0xA1) // Y stick change
{
   ystick=data[2];
   if (ystick>511) // YStick to up
     {
       yrate=find_yp();
       pwm_motA1=gthrot-yrate;
       pwm_motA2=gthrot-yrate;
       pwm_motA3=gthrot+yrate;
       pwm_motA4=gthrot+yrate;
     }
   else if (ystick<450)// Ystick to down
     {
       yrate=find_yn();
       pwm_motA1=gthrot+yrate;
       pwm_motA2=gthrot+yrate;
       pwm_motA3=gthrot-yrate;
       pwm_motA4=gthrot-yrate;
     }
        else 
        {
        }
   

//Motors cannot shutdown here
if (pwm_motA1<mot_min) pwm_motA1=mot_min;
if (pwm_motA2<mot_min) pwm_motA2=mot_min;
if (pwm_motA3<mot_min) pwm_motA3=mot_min;
if (pwm_motA4<mot_min) pwm_motA4=mot_min;
}             
// Y stick not changed lets check for X
if (data[0]==0x01&&data[1]==0xA2) // X stick change
{
        xstick=data[2];
        if (xstick>511) // XStick to right
         {
          xrate=find_xp();
          pwm_motA1=gthrot-xrate;
          pwm_motA2=gthrot+xrate;
          pwm_motA3=gthrot-xrate;
          pwm_motA4=gthrot+xrate;
         }
        else if (xstick<450)// Xstick to left
        {
          xrate=find_xn();
          pwm_motA1=gthrot+xrate;
          pwm_motA2=gthrot-xrate;
          pwm_motA3=gthrot+xrate;
          pwm_motA4=gthrot-xrate;
        }
        else 
        {
        }

//Motors cannot shutdown here
if (pwm_motA1<mot_min) pwm_motA1=mot_min;
if (pwm_motA2<mot_min) pwm_motA2=mot_min;
if (pwm_motA3<mot_min) pwm_motA3=mot_min;
if (pwm_motA4<mot_min) pwm_motA4=mot_min;

}//end of Xstick

// Yaw controls
if (data[1]==0xA5&&data[2]==1)// Yaw plus, rotate right incrase power to C.W motors
      {
          dyaw=true;
          my_yaw();
      }
if (data[1]==0xA6&&data[2]==0)// Yaw minus, 
     {
        dyaw=false;
        my_yaw();
     }
// End of Yaw controls   

//Serial.print("M1: "); Serial.println(pwm_motA1);
//Serial.print("M2: "); Serial.println(pwm_motA2);
//Serial.print("M3: "); Serial.println(pwm_motA3);
//Serial.print("M4: "); Serial.println(pwm_motA4);
}//End of rate()


void show_debug()
{
  #ifdef Debug_GYROACCEL
  //Serial.print("Pitch= "); Serial.print(s_kalAngleX); Serial.print("\t");
  //Serial.print("Roll= "); Serial.print(s_kalAngleY); Serial.print("\t");
  //Serial.print("Pitch= "); Serial.print(kalAngleX); Serial.print("\t");
  //Serial.print("Roll= "); Serial.print(kalAngleY); Serial.print("\t");
  Serial.print("Pitch= "); Serial.print(compAngleX); Serial.print("\t");
  Serial.print("Roll= "); Serial.print(compAngleY); Serial.print("\t");
  Serial.println();
  #endif
  #ifdef Debug_Compass
  Serial.print("heading: "); // A tab is already printed out before
  Serial.print(dir);
  Serial.println();
  #endif
  #ifdef Debug_Varometer
  Serial.println();
  Serial.print("Actual TEMP= "); Serial.print(Temperature);
  Serial.print("\t");
  Serial.print("Actual ALTITUDE= ");
  Serial.print(h);
  Serial.print("m");  
  Serial.print("\r\n");  
  #endif
  #ifdef Debug_NRF
  Serial.print("Mode: "); Serial.print(data[0]);
  Serial.println();
  Serial.print("Element: "); Serial.print(data[1]);
  Serial.println();
  Serial.print("Strenght: "); Serial.print(data[2]);
  Serial.println();
  #endif
}
// End of show_debug()
