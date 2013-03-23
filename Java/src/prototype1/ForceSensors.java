package prototype1;

/**
 * A class to represent force sensors, //TODO
 * 
 * 		//Being used in the prototype for the ultrasonic sensors
 */

public class ForceSensors extends Sensor
{

	/**
	 * The field to store the number of force sensors controlled by this class
	 */
	private int numberOfForceSensors;
	
	/**
	 * The constructor for the ForceSensors
	 * @param attachedArduino	The Arduino that the force sensors are attached to
	 * @param forceSensorsToControl		The number of force sensors this class controls
	 */
	public ForceSensors(Arduino attachedArduino, int forceSensorsToControl)
	{
		super(attachedArduino, MessageProtocol.ID1_FORCE);
		this.numberOfForceSensors = forceSensorsToControl;
	}
	
	/**
	 * The method to return the last values of force recorded 
	 * @return	The integer array representing the last packet transfered from the Arduino
	 */
	public int[] getLastForceValues()
	{
		//We should have 2 bytes of data per force sensor
		int[] toReturn = new int[this.numberOfForceSensors];
		byte[] arrayAsBytes = new byte[this.numberOfForceSensors*2];
		arrayAsBytes = super.cache.peek();
		//TODO check to make sure returned byte array from cache is as expected
		for (int i = 0; i < this.numberOfForceSensors; i++)
		{
			toReturn[i] = ( (int) (arrayAsBytes[i*2]) * 256 ) + ( (int) arrayAsBytes[(i*2)+1] );
		}
		return toReturn;
	} 
	
}
