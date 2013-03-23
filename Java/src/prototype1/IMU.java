package prototype1;

public class IMU extends Sensor
{
	
	private byte identifier;
	
	/**
	 * Constructor that calls the super constructor (Sensor)
	 * @param attachedArduino	The arduino that this instrument is physically attached to
	 */
	public IMU(Arduino attachedArduino)
	{
		super(attachedArduino, ID1_IMU);
	}
	
	/**
	 * Enable the IMU
	 */
	public void enableIMU()
	{
		this.setEnabled(true);
	}
	
	/**
	 * Disable the IMU
	 */
	public void disableIMU()
	{
		this.setEnabled(false);
	}
	
	
}
