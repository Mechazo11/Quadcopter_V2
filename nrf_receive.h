#include "Arduino.h"
#ifndef _nrf_receive_h
#define _nrf_receive_h


//********* PIN CONSTANT DECLERATION ******//
#define CE_PIN  49
#define CSN_PIN 53

RF24 radio(CE_PIN, CSN_PIN); // "radio" here is the object

const uint64_t pipe = 0xE8E8F0F0E1LL;
// This defines the pipe addressed to be used
// Note this is the same pipe address as that was incorporated 
// into UNO. 
// Both the radio MUST communicate in the same 
// pipe for this experiment to work


int data[3]; // This is an array that will be used
//to pass commands from transmitter to the receiver
// Command, Element and Strenght
void nrf_getcmd();
void nrf_setup();
void nrf_shutdown();

#endif
