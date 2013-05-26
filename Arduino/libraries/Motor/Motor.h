// custom class for motor control emulating the servo library.

#ifndef motor_h
#define motor_h

#include <inttypes.h>
#include "Arduino.h"

class Motor
{
	public:
	
	Motor();	// constructor
	
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