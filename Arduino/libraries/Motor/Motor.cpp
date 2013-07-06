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

void Motor::attach(byte aPin, byte bPin, byte pwmPin)
{
	testPlatform = false;
	pinA = aPin;
	pinMode(aPin,OUTPUT);
	digitalWrite(aPin,LOW);
	pinB = bPin;
	pinMode(bPin,OUTPUT);
	digitalWrite(bPin,LOW);
	pinPwm = pwmPin;
	pinMode(pinPwm,OUTPUT);
	analogWrite(pinPwm,0);
	attached = true;

}

void Motor::attach(byte dirPin, byte pwmPin)
{                                                     
	testPlatform = true;
	pinDir = dirPin;
	pinMode(pinDir,OUTPUT);
	digitalWrite(pinDir,LOW);
	pinPwm = pwmPin;
	pinMode(pinPwm,OUTPUT);
	analogWrite(pinPwm,0);
	attached = true;
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
	else if(newSetting < 127) // reverse
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


Actuator::Actuator(int throwLength)
{
	length = throwLength;
	attached = false;
	moving = false;
	inValue = 963; // default calibration
	outValue = 57;
}

int Actuator::position() const
{
	return constrain(map(analogRead(0),inValue,outValue,0,length),0,length);
}

void Actuator::attach(byte aPin, byte bPin, byte movePin)
{
	pinA = aPin;
	pinMode(pinA,OUTPUT);
	digitalWrite(pinA,LOW);
	pinB = bPin;
	pinMode(pinB,OUTPUT);
	digitalWrite(pinB,LOW);
	pinMove = movePin;
	pinMode(pinMove,OUTPUT);
	digitalWrite(pinMove,LOW);
	attached = true;
}

void Actuator::set(int newSetting)
{
	if(!attached)
		return;
	int currentPosition = position();
	if(abs(currentPosition - newSetting) > 3) // not "close enough" already
	{
		currentTarget = newSetting;
		moving = true;
		if(newSetting > currentPosition) // start moving out
		{
			digitalWrite(pinA,LOW);
			digitalWrite(pinB,HIGH);
		}
		else	// start moving in
		{
			digitalWrite(pinA,HIGH);
			digitalWrite(pinB,LOW);
		}
		digitalWrite(pinMove,HIGH);
	}
	else
		halt();
}

void Actuator::halt()
{
	if(!attached)
		return;
	digitalWrite(pinMove,LOW);
	digitalWrite(pinA,LOW);
	digitalWrite(pinB,LOW);
	moving = false;
}
		
void Actuator::calibrate(int fullIn, int fullOut)
{
	inValue = fullIn;
	outValue = fullOut;
}
	
void Actuator::refresh()
{
	if(!moving)
		return;
	else
		set(currentTarget);
}