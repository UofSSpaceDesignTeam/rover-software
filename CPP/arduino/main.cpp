//
// This is the top-level code running on the rover. This needs to have its own thread
// and should never use blocking calls. Keeping the main loop fast is a high priority.

#include "Arduino.h"
#include "Sensor.h"
#include "Comm.h"
#include "Mine.h"
#include "Drive.h"

#include <cstdlib>

using namespace std;

bool error = false;
char* errormsg = "Unknown error";
bool shutdown = false;

int main()
{

	// initialize everything up here
	
	if(error)
	{
		cerr << errormsg;
		return -1;
	}
	
	while(!shutdown && !error)
	{
		
		// main execution loop
		
	}
	
	if(error)
	{
		cerr << errormsg;
		return -1;
	}
	
	return 0;
}