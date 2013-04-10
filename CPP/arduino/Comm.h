// a class to handle two-way message based communication with the control station.
// Please, try to use a sensor object for data acquisition instead of duplicating functionality :)
// TODO: find socket and wifi libraries to use, develop message protocol 

#ifndef COMM_H
#define COMM_H

#include "linux wifi library" // need one of these
#include "linux socket library" // need one of these too
#include "Sensor.h" // interface for data aquisition

using namespace std;

class Comm
{

	public:
	
	// constructor
	
	Comm(Socket connection);
	
	// communication functions
	
	int sendMessage(char* code, char* data); // send message and wait for reply
	
	bool checkMessage(); // see if we have messages waiting
	
	int processMessage(); // parse next message and send an appropriate reply
	
	// communication data
	
	int commCheck(int timeout); // send a test message and return ping time
	
	private:
	
	// stuff goes here?
	
}

// other stuff goes here?

#endif
	
	