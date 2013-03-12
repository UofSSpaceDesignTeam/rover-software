// a class to handle operation of the digging and dumping mechanisms.
// Please, try to use a sensor object for data acquisition instead of duplicating functionality :)
// TODO: add useful things

#ifndef MINE_H
#define MINE_H

#include "Arduino.h" // message interface for motor controllers
#include "Sensor.h" // interface for data acquisition

using namespace std;

class Mine
{

	public:
	
	// constructor
	
	Mine(const Arduino &arduino, const Sensor &bucket, const Sensor &hopper);
	
	// mining actions, return false on failure
	
	bool startBucket(int rpm);
	
	bool stopBucket();
	
	bool moveHopper(unsigned int height);
	
	bool tiltHopper(unsigned int angle);
	
	bool stopAll();
	
	// mining data, return -1 if unavailable
	
	int bucketSpeed();
	
	int hopperHeight();
	
	int hopperTilt();
	
	int hopperWeight();
	
	
	private:
	
	// stuff goes here?
	
}

// other stuff goes here?

#endif
	
	