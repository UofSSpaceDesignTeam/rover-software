// custom class for load cells

#ifndef LOADCELL_H
#define LOADCELL_H

#include <inttypes.h)
#include "Arduino.h"


class loadCell
{
	public:
	
	loadCell();	// constructor
	
	uint16_t readRaw() const;	// direct read from analog pin
	
	int attach(byte posAnalogPin, byte negAnalogPin, byte powerPin);	// set up connections
	void groupOn();
	void groupOff();
	
	private:
	
	bype pinPower;
	byte posPinData;
	byte negPinData;
	boolean attached;
	
};

uint16_t convertToForce(uint16_t netData);	// outputs force

#endif