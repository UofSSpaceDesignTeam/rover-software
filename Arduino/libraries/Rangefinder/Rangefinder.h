// custom class for motor control on the test platform.

#ifndef RANGEFINDER_H
#define RANGEFINDER_H

#include <inttypes.h>
#include "Arduino.h"

// sensor type is 1 for short range sensor (not existing yet)
// 2 for medium range sensor
// 3 for long range sensor (not existing yet)

class Rangefinder
{
	public:
	
	Rangefinder(byte sensorType);	// constructor
	
	uint16_t readRaw() const; // read the output directly
	
	int attach(byte analogPin, byte powerPin);	// define connections
	void groupOn();
	void groupOff();

	private:
	
	byte pinPower;
	byte pinData;
	byte type;
	boolean attached;

};

uint16_t convertToDistance(uint16_t data, byte sensorType);  // outputs mm


#endif