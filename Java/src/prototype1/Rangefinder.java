package prototype1;

/**
 * @author dylan
 *	The rangefinders for the Robot.  We get 6 numbers back because the arduino
 *	handles the data and returns the valid one between the short and long rangefinder
 */
public class Rangefinder extends Sensor 
{
	/**
	 * 	the number of rangefinders attached to the arduino
	 */
	private int numRangefinders;
	
	/**
	 * @param numRFinders the number of rangefinders attached to the Arduino
	 * @param attachedArduino is the arduino that the rangefinder is physically on
	 */
	Rangefinder(int numRFinders, ArduinoMessageHandler attachedArduino)
	{
		super(attachedArduino,ID1_RANGEFINDER);
		this.numRangefinders = numRFinders;
	}
	
	/**
	 * Sends enable/disable to the rangefinder autoleveler and starts the rangefinders
	 */
	public enableRangefinders()
	{
		super.sendToArduino(super.id, MessageProtocol.ID2_ENABLE_DISABLE);
	}
	
	/**
	 * sends query to the arduino for data
	 * the arduino returns data in the form RD [data]12 bytes 2 bytes for each sensor
	 */
	public queryRangefinders()
	{
		super.sendToArduino(super.id, MessageProtocol.ID2_QUERY);
		//TODO deal with return message
	}
}
