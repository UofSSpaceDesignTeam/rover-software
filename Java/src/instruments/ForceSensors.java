package instruments;

import environment.Situation;
import messaging.ArduinoMessageHandler;
import messaging.MessageProtocol;

/**
 * A class to represent force sensors, //TODO
 * 
 * 		//Being used in the prototype for the ultrasonic sensors
 */

public class ForceSensors extends Sensor
{
	
	/**
	 * The constructor for the ForceSensors
	 * @param attachedArduino	The Arduino that the force sensors are attached to
	 * @param forceSensorsToControl		The number of force sensors this class controls
	 */
	public ForceSensors(ArduinoMessageHandler attachedArduino, Situation[] s, int forceSensorsToControl)
	{	
		super(attachedArduino, MessageProtocol.ID1_FORCE, s, forceSensorsToControl);
	}
	
	/**
	 * The method to return the last values of force recorded 
	 * @return	The integer array representing the last packet transfered from the Arduino
	 */
	public int[] getLastForceValues()
	{
		//We should have 2 bytes of data per force sensor
		int[] toReturn = new int[this.size()];
		byte[] arrayAsBytes = new byte[this.size()*2];
		arrayAsBytes = super.cache.peek().getValue();
		//TODO check to make sure returned byte array from cache is as expected
		for (int i = 0; i < this.size(); i++)
		{
			toReturn[i] = ( (int) (arrayAsBytes[i*2]) * 256 ) + ( (int) arrayAsBytes[(i*2)+1] );
		}
		return toReturn;
	} 
	
}
