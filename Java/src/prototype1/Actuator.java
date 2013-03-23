package prototype1;


/**
 * An abstract class to store functionality common to all actuators
 */

public abstract class Actuator extends Instrument
{
	/**
	 * Constructor that calls the super constructor (Instrument)
	 * @param attachedArduino	The arduino that this instrument is physically attached to
	 * @param thisId	the [ID1] byte that represents this instrument (M for motor; I for IMU, etc)
	 */
	Actuator(Arduino attachedArduino, byte thisId)
	{
		super(attachedArduino, thisId);
	}
	
	/**
	 * The method to set data to an actuator
	 * @param data	The data bytes to set; [B1]...[Bn]
	 */
	protected void set(byte[] data)
	{
		this.sendToArduino(MessageProtocol.ID2_SET, data);
	}
	
}
