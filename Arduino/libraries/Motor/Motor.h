// custom class for motor control on the test platform.

#ifndef MOTOR_H
#define MOTOR_H

#include <inttypes.h>
#include "Arduino.h"

class Motor
{
	public:
	
	Motor();	// constructor
	
	byte setting() const; // access the last setting given to the motor
	
	int attach(byte aPin, byte bPin, byte pwmPin);	// set the hardware connections for the motor
	void set(byte newSetting);	// set the output of the motor
	
	private:
	
	byte pinA;
	byte pinB;
	byte pinPWM;
	boolean attached;
	byte currentSetting;
};

class Actuator
{
	public:
	
	Actuator();	// constructor
	
	int position() const; // access the current length in mm
	byte target() const; // access the last order given to the actuator
	
	int attach(byte aPin, byte bPin, byte pwmPin);	// set the hardware connections for the actuator
	void set(byte newSetting);	// set the target for the actuator
	
	private:
	
	byte pinA;
	byte pinB;
	byte pinPWM;
	boolean attached;
	int currentPosition;
	byte currentTarget;
	
	
	
	
	
	

#endif