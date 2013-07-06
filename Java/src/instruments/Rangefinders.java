package instruments;

import environment.Situation;
import messaging.ArduinoMessageHandler;
import messaging.MessageProtocol;

/**
 * @author dylan
 *	The rangefinders for the Robot.  We get 6 numbers back because the arduino
 *	handles the data and returns the valid one between the short and long rangefinder
 */
public class Rangefinders extends Sensor 
{
	
	/**
	 * @param numRFinders the number of rangefinders attached to the Arduino
	 * @param attachedArduino is the arduino that the rangefinder is physically on
	 */
	public Rangefinders(ArduinoMessageHandler attachedArduino, Situation[] s, int numOfRangeFinders)
	{
		super(attachedArduino,MessageProtocol.ID1_RANGEFINDER, s, numOfRangeFinders);
	}
	
	
	/**
	 * sends query to the arduino for data
	 * the arduino returns data in the form RD [data]12 bytes 2 bytes for each sensor
	 * @return 
	 */
	public void query()
	{
		byte[] byteToSend = new byte[1];
		byteToSend[0] = MessageProtocol.ID2_QUERY;
		super.sendToArduino(id, byteToSend);
	}
}
