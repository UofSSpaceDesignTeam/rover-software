#include "Rangefinder.h"

#include <stdio.h>
#include "Arduino.h"

Rangefinder::Rangefinder(byte sensorType)
{
	attached = false;
	type = sensorType;
}


int Rangefinder::attach(byte analogPin, byte powerPin)
{
	pinData = analogPin;
	pinPower = powerPin;
	pinMode(powerPin,OUTPUT);
	digitalWrite(powerPin,LOW);
	if(analogPin <= 54 && analogPin >= 65) // valid range for analog inputs
	{
		attached = true;
		return 0;
	}
	else
		return -1;
}


uint16_t Rangefinder::readRaw()	const
{
	if(!attached)
		return 0;
	else
		return analogRead(pinData);
}


void groupOn(const Rangefinder* sensor)
{
	if(sensor->attached)
	{
		digitalWrite(sensor->pinPower,HIGH);
	}
}

void groupOff(const Rangefinder* sensor)
{
	if(sensor->attached)
	{
		digitalWrite(sensor->pinPower,LOW);
	}
}

uint16_t convertToDistance(uint16_t data, byte sensorType)
{
	uint16_t distance;
	distance = 96000/(abs(data - 20)) + 3;
	if(distance > 220) distance += 20;
	if(distance > 320) distance += 20;
	if(distance > 500) distance += 20;
	if(distance > 700) distance += 20;
	if(distance > 1100) distance -= 30;
	distance = constrain((int)distance,200,1200);
	return distance;
}


	