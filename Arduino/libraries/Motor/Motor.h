// custom class for motor control on the test platform.

#ifndef MOTOR_H
#define MOTOR_H

#include <stdio.h>
#include "Arduino.h"
#include <inttypes.h>

class Motor
{
	public:
	
	Motor();	// constructor
	
	byte setting() const; // access the last setting given to the motor
	
	void attach(byte aPin, byte bPin, byte pwmPin);	// for official motor controller
	void attach(byte dirPin, byte pwmPin);	// for test platform
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
	
	Actuator(int throwLength);	// constructor
	
	int position() const; // access the current length in mm
	
	void attach(byte aPin, byte bPin, byte movePin);	// set the hardware connections for the actuator
	void set(int newSetting);	// set the target for the actuator
	void halt(); // stops the actuator
	void calibrate(int fullIn, int fullOut); // calibration for potentiometers
	void refresh(); // check on the actuator if it is moving
	
	
	private:
	
	byte pinA;
	byte pinB;
	byte pinMove;
	boolean attached;
	boolean moving;
	int currentTarget;
	int inValue;
	int outValue;
	int length;
};


#endif