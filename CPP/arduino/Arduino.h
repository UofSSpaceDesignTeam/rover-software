// a class to handle two-way message based communication with the arduino board using same protocol
// as wifi communications. This is the backbone of many other classes.
// TODO: find serial libraries to use, develop message protocol 
//

#ifndef ARDUINO_H
#define ARDUINO_H

#include "linux serial library" // need one of these

using namespace std;

class Arduino
{

	public:
	
	// constructor
	
	Arduino(Serial portname);
	
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
	
	
