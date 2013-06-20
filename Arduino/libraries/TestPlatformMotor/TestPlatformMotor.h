// custom class for motor control on the test platform.

#ifndef TESTPLATFORMMOTOR_H
#define TESTPLATFORMMOTOR_H

#include <inttypes.h>
#include "Arduino.h"

class TestPlatformMotor
{
	public:
	
	TestPlatformMotor();	// constructor
	
	byte setting() const; // accessors
	
	int attach(byte dirPin, byte pwmPin); // mutators
	void set(byte newSetting);
	
	private:
	
	byte directionPin;
	byte speedPin;
	boolean attached;
	byte currentSetting;
};

#endif