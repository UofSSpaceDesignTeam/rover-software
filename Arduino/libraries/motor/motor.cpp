#include "motor.h"

#include <stdio.h>
#include "Arduino.h"

motor::motor(byte dirPin, byte pwmPin)
{
	directionPin = dirPin;
	speedPin = pwmPin;
	enabled = false;
	pinMode(directionPin,OUTPUT);
	digitalWrite(directionPin,LOW);
	if(speedPin <= 13 && speedPin >= 2) // valid range for pwm capable pins
	{
		analogWrite(speedPin,0);
	}
	else // make sure nothing happens
	{
		pinMode(speedPin,OUTPUT);
		digitalWrite(speedPin,
	}
}

void motor::enable()
{
	enabled = true;
}

void motor::disable()
{
	analogWrite(speedPin,0);
	digitalWrite(directionPin,LOW);
	enabled = false;
}

void motor::set(byte setting)
{
	if(!enabled)
	{
		return;
	}
	if(setting == 127) // stop
	{
		analogWrite(speedPin,0);
	}
	else if(setting > 127) // forward
	{
		digitalWrite(directionPin,HIGH);
		analogWrite(speedPin,2*(setting-127));
	}
	else // reverse
	{
		digitalWrite(directionPin,LOW);
		analogWrite(speedPin,2*(127-setting));
	}
}