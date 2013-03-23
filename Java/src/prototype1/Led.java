package prototype1;

public class Led extends Actuator
{
	
	/**
	 * Constructor that calls the super constructor (Instrument)
	 * @param attachedArduino	The arduino that this instrument is physically attached to
	 * @param thisId	the [ID1] byte that represents this instrument (M for motor; I for IMU, etc)
	 */
	public Led(Arduino attachedArduino, byte thisId)
	{
		super(attachedArduino, thisId);
	}
	
	
}
