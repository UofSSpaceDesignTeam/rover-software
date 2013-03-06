// a class to abstract data gathered from sensors connected to an arduino, using a message protocol
// similar to the base station. Use the arduino object for actual communication.

#ifndef SENSOR_H
#define SENSOR_H

#include "Arduino.h" // the message interface

using namespace std;

class Sensor
{

	public:
	
	// constructor and destructor
	
	Sensor(const Arduino &arduino, int numdata, int refreshtime);
	
	~Sensor(); //  gotta destroy dynamic data
	
	// communication functions, return false on failure
	
	bool configure(char* code, char* data); // send a command through arduino object and wait for reply
	
	bool update(); // get new state information from sensor
	
	unsigned long age(); // time since last update
	
	int* data;
	
	
	private:
	
	unsigned long lastupdate; // timestamp of last update
	
}

// other stuff goes here?

#endif
	
	