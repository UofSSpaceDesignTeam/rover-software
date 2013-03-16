// a class to handle movement operations of the rover.
// Please, try to use a sensor object for data acquisition instead of duplicating functionality :)
// TODO: add useful things, determine how to report skid detection?

#ifndef DRIVE_H
#define DRIVE_H

#include "Arduino.h" // message interface for motor controllers
#include "Sensor.h" // interface for data aquisition

using namespace std;

class Drive
{

	public:
	
	// constructor
	
	Drive(const Arduino &arduino, const Sensor &wheel, const Sensor &imu, const Sensor &bumper);
	
	// basic movement actions, return false on failure
	
	bool move(int speed); // backward = negative speed
	
	bool turn(int speed);
	
	bool stopAll();
	
	bool setMotor(int motor, int speed); // mostly for auto navigation
	
	// movement data, return -1 on failure
	
	int speed(); // absolute average speed for all wheels
	
	int motorSpeed(int motor); // individual rotation rate
	
	
	private:
	
	// stuff goes here?
	
}

// other stuff goes here?

#endif
	
	