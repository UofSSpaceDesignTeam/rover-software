package prototype1;

public class Led extends Actuator
{

	
	private byte[] turnOnCall {'1'};
	private byte[] turnOffCall = {'0'};
	private byte[] blinkCall = {'b'};
	
	
	/**
	 * Constructor that calls the super constructor (Instrument)
	 * @param attachedArduino	The arduino that this instrument is physically attached to
	 */
	public Led(Arduino attachedArduino)
	{	
		super(attachedArduino, ID1_LED);
	}
	
	/**
	 * Turns the Led on for the Arduino
	 */
	public turnOn()
	{
		super.set(turnOnCall);
	}
	
	/**
	 * Turns off the Led
	 */
	public turnOff()
	{
		super.set(turnOffCall);
	}
	
	/**
	 * Start the Led blinking
	 */
	public startBlinking()
	{
		super.set(blinkCall);
	}
	
}
