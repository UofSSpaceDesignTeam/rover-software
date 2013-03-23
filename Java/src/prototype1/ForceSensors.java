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
	ForceSensors(Arduino attachedArduino, int forceSensorsToControl)
	{
		super(attachedArduino, MessageProtocol.ID1_FORCE);
		this.numberOfForceSensors = forceSensorsToControl;
	}
	
	
	
}
