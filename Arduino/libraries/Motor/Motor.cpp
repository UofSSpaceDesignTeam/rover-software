#include "TestPlatformMotor.h"

#include <stdio.h>
#include "Arduino.h"
#include <inttypes.h>

TestPlatformMotor::TestPlatformMotor()
{
	attached = false;
	currentSetting = 127;
}

byte TestPlatformMotor::setting() const
{
	return currentSetting;
}

int TestPlatformMotor::attach(byte dirPin, byte pwmPin)
{
	directionPin = dirPin;
	speedPin = pwmPin;
	pinMode(directionPin,OUTPUT);
	digitalWrite(directionPin,LOW);
	if(speedPin <= 13 && speedPin >= 2) // valid range for pwm capable pins
	{
		pinMode(speedPin,OUTPUT);
		analogWrite(speedPin,0);
		attached = true;
		return 0;
	}
	else 
		return -1;
}

void TestPlatformMotor::set(byte newSetting)
{
	if(!attached)
		return;
	currentSetting = newSetting;
	if(newSetting == 127) // stop
	{
		analogWrite(speedPin,0);
	}
	else if(newSetting > 127) // forward
	{
		digitalWrite(directionPin,LOW);
		analogWrite(speedPin,2*(newSetting-127));
	}
	else // reverse
	{
		digitalWrite(directionPin,HIGH);
		analogWrite(speedPin,2*(127-newSetting));
	}
}