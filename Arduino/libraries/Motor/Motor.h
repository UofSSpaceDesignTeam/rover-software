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
	
	int attach(byte aPin, byte bPin, byte pwmPin);	// for official motor controller
	int attach(byte dirPin, byte pwmPin);	// for test platform
	void set(byte newSetting);	// set the output of the motor
	
	private:
	
	byte pinA;
	byte pinB;
	byte pinDir;
	byte pinPwm;
	boolean attached;
	boolean testPlatform;
	byte currentSetting;
};


class Actuator
{
	public:
	
	Actuator();	// constructor
	
	int position() const; // access the current length in mm
	int target() const; // access the last order given to the actuator
	
	int attach(byte aPin, byte bPin, byte pwmPin);	// set the hardware connections for the actuator
	void set(byte newSetting);	// set the target for the actuator
	
	void refresh(); // check on the actuator if it is moving
	
	private:
	
	byte pinA;
	byte pinB;
	byte pinPWM;
	boolean attached;
	boolean moving;
	byte currentTarget;
};


#endif