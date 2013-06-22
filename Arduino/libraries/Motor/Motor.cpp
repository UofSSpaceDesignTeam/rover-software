#include "Motor.h"

#include <stdio.h>
#include "Arduino.h"
#include <inttypes.h>

Motor::Motor()
{
	attached = false;
	currentSetting = 127;
}

byte Motor::setting() const
{
	return currentSetting;
}

int Motor::attach(byte aPin, byte bPin, byte pwmPin)
{
	testPlatform = false;
	pinA = aPin;
	pinMode(aPin,OUTPUT);
	digitalWrite(aPin,LOW);
	pinB = bPin;
	pinMode(bPin,OUTPUT);
	digitalWrite(bPin,LOW);
	if(pinPwm <= 13 && pinPwm >= 2) // valid range for pwm capable pins
	{
		pinMode(pinPwm,OUTPUT);
		analogWrite(pinPwm,0);
		attached = true;
		return 0;
	}
	else
		return -1;
}

int Motor::attach(byte dirPin, byte pwmPin)
{                                                     
	testPlatform = true;
	pinDir = dirPin;
	pinMode(pinDir,OUTPUT);
	digitalWrite(pinDir,LOW);
	if(pinPwm <= 13 && pinPwm >= 2) // valid range for pwm capable pins
	{
		pinMode(pinPwm,OUTPUT);
		analogWrite(pinPwm,0);
		attached = true;
		return 0;
	}
	else 
		return -1;
}

void Motor::set(byte newSetting)
{
	if(!attached)
		return;
	currentSetting = newSetting;
	if(newSetting == 127) // stop
	{
		analogWrite(pinPwm,0);
		if(!testPlatform)
		{
			digitalWrite(pinA,LOW);
			digitalWrite(pinB,LOW);
		}
	}
	else if(newSetting > 127) // forward
	{
		if(testPlatform)
		{
			digitalWrite(pinDir,LOW);
		}
		else
		{
			digitalWrite(pinA,LOW);
			digitalWrite(pinB,HIGH);
		}
		analogWrite(pinPwm,2*(newSetting-127));
	}
	else // reverse
	{
		if(testPlatform)
		{
			digitalWrite(pinDir,HIGH);
		}
		else
		{
			digitalWrite(pinA,HIGH);
			digitalWrite(pinB,LOW);
		}
		analogWrite(pinPwm,2*(127-newSetting));
	}
}