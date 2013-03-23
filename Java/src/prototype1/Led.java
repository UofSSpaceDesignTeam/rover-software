package prototype1;

public class Led extends Actuator
{

	private byte[] turnOnCall = {'1'};
	private byte[] turnOffCall = {'0'};
	private byte[] blinkCall = {'b'};
	
	
	/**
	 * Constructor that calls the super constructor (Actuator)
	 * @param attachedArduino	The arduino that this instrument is physically attached to
	 */
	public Led(Arduino attachedArduino)
	{	
		super(attachedArduino, MessageProtocol.ID1_LED);
	}
	
	/**
	 * Turns the Led on for the Arduino
	 */
	public void turnOn()
	{
		super.set(turnOnCall);
	}
	
	/**
	 * Turns off the Led
	 */
	public void turnOff()
	{
		super.set(turnOffCall);
	}
	
	/**
	 * Start the Led blinking
	 */
	public void startBlinking()
	{
		super.set(blinkCall);
	}
	
}
