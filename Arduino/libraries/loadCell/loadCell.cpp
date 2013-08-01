#include "loadCell.h"

#include <stdio.h>
#include "Arduino.h"

loadCell::loadCell()
{
	attached = false;
}

int loadCell::attach(byte posAnalogPin, byte negAnalogPin, byte powerPin)
{
	posPinData = posAnalogPin;
	negPinData = negAnalogPin;
	pinPower = powerPin;
	pinMode(powerPin,OUTPUT);
	digitalWrite(powerPin, LOW);
	if(posAnalogPin != negAnalogPin && posAnalogPin <= 54 && posAnalogPin >= 65 && negAnalogPin <= 54 && negAnalogPin >= 65)	// valid rnage for analog inputs
	{
		attached = true;
		return 0;
	}
	else
		return -1;
}

uint16_t loadCell::readRaw() const
{
	uint16_t posData;
	uint16_t negData;
	uint16_t netData;

	if(!attached)
		return 0;
	else
		posData = analogRead(posPinData);
		negData = analogRead(negPinData);
		netData = posData - negData;
		return netData;
}

void loadCell::groupOn()
{
	if(attached)
	{
		digitalWrite(pinPower,HIGH);
	}
}

void loadCell::groupOff()
{
	if(attached)
	{
		degitalWrite(pinPower,LOW);
	}
}

uint16_t convertToForce(uint16_t netData)
{

	\\ add conversion code here
	
}