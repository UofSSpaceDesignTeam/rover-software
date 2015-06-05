// Code for the Arduino controlling the motor drivers.
// Written for Arduino Micro.
#include <Servo.h>
#include <Wire.h>

#define TIMEOUT 750
#define CMD_HEADER 0xF7
#define CMD_TRAILER 0xF8
#define I2C_ADDRESS 0x09

enum command_type // instructions from the Pi
{
	STOP, // stop all motors
	SET_SPEED, // set desired speed for drill / elevator
	SET_LASER // Turn on/off desired laser
};

typedef struct
{	
	byte header;
	byte type; // actually used as enum command_type, that's ok
	int d1; // 16-bit signed
	int d2;
	byte csum; // sum of cmd_type, d1, d2
	byte trailer;
} command;


/*
	Wiring connections.
	Each motor only takes a servo pulse
	Two Lasers have an enable HIGH, one enable LOW
*/

// pin connetions for motor
const byte l_pwr[] = {4, 6, 5};
byte drillPin = 10;
byte elevPin = 11;

// servo stuff for motors
Servo drillMotor;
short drillSpeed = 0;
short desiredDrillSpeed = 0;
Servo elevMotor;
short elevSpeed = 0;
short desiredElevSpeed = 0;

// control data
volatile short l_cmd[] = {0, 0, 0}; // commanded lasers

// state information
unsigned long timeout;
volatile command cmd;
byte* cmdPointer = (byte*)(&cmd);
volatile byte cmdCount = 0;
volatile bool newCommand = false;


void receiveEvent(int count);
void processCommand();
void updateDrill();
void updateElev();
void stopAll();
/*
	Turns on a single laser
	index: which laser, [0, 1, 2]
	value: on or off [0, 1]
*/
void setLaser(byte index, short value);

void setup()
{
	Serial.begin(9600);
	Wire.begin(I2C_ADDRESS);
	Wire.onReceive(receiveEvent);
	
	pinMode(drillPin, OUTPUT);
	pinMode(elevPin, OUTPUT);
	for(int i = 0; i < 3; i++)
		pinMode(l_pwr[i], OUTPUT);
	
	drillMotor.attach(drillPin);
	elevMotor.attach(elevPin);
	stopAll();
		
	timeout = millis();
}

void loop()
{
	if(newCommand)
	{
		processCommand();
		newCommand = false;
	}
	
	updateDrill();
	updateElev();
	
	if(millis() - timeout > TIMEOUT)
	{
		Serial.println("TO");
		stopAll();
		timeout = millis();
	}
	for(int i = 0; i < 3; i++)
		setLaser(i, l_cmd[i]);
	delay(50);
}

void processCommand()
{
	switch(cmd.type)
	{
		case STOP:
		stopAll();
		break;
		
		case SET_SPEED:
		desiredDrillSpeed = constrain(cmd.d1, -255, 255);
		desiredElevSpeed = constrain(cmd.d2, -255, 255);
		timeout = millis();
		break;
		
		case SET_LASER:
		Serial.println("laser");
		if(cmd.d1 > 3 || cmd.d1 < 0)
			break;
		if(cmd.d2 > 1 || cmd.d1 < 0)
			break;
		l_cmd[cmd.d1 - 1] = cmd.d2;
		timeout = millis();
		Serial.print(cmd.d1);
		Serial.print(", ");
		Serial.println(cmd.d2);
		break;
	}
}

void updateDrill()
{
	if(desiredDrillSpeed == 0)
	{
		drillMotor.writeMicroseconds(1500);
		drillSpeed = 0;
		return;
	}
	else if(abs(desiredDrillSpeed - drillSpeed) < 5)
		drillSpeed = desiredDrillSpeed;
	else if(desiredDrillSpeed > drillSpeed)
		drillSpeed += 5;
	else
		drillSpeed -= 5;
	drillMotor.writeMicroseconds(map(drillSpeed, -255, 255, 900, 2100));
}
	
void updateElev()
{
	if(desiredElevSpeed == 0)
	{
		elevMotor.writeMicroseconds(1500);
		elevSpeed = 0;
		return;
	}
	else if(abs(desiredElevSpeed - elevSpeed) < 5)
		elevSpeed = desiredElevSpeed;
	else if(desiredElevSpeed > elevSpeed)
		elevSpeed += 5;
	else
		elevSpeed -= 5;
	elevMotor.writeMicroseconds(map(elevSpeed, -255, 255, 900, 2100));
}

void setLaser(byte index, short value)
{
	if(index == 2)
		digitalWrite(l_pwr[index], !value);
	else
		digitalWrite(l_pwr[index], value);
}

void stopAll()
{
	Serial.println("stopped");
	drillSpeed = 0;
	desiredDrillSpeed = 0;
	elevSpeed = 0;
	desiredElevSpeed = 0;
	drillMotor.writeMicroseconds(1500);
	elevMotor.writeMicroseconds(1500);
}

void receiveEvent(int count)
{
	if(newCommand)
		return;
	while(Wire.available())
	{
		byte in = Wire.read();
		
		if(cmdCount == 0) // wait for header
		{
			if(in == CMD_HEADER)
			{
				cmdPointer[cmdCount] = in;
				cmdCount++;
			}
			continue;
		}
		
		if(cmdCount < sizeof(command)) // add middle bytes
		{
			cmdPointer[cmdCount] = in;
			cmdCount++;
		}
		
		if(cmdCount == sizeof(command)) // check for complete
		{
			if(in == CMD_TRAILER)
			{
				byte csum = cmd.type + cmd.d1 + cmd.d2;
				if(csum == cmd.csum)
					newCommand = true;
			}
			cmdCount = 0;
		}
	}
}


