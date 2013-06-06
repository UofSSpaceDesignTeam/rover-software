package prototype1;

public class IMU extends Sensor
{
	
	/**
	 * The field to store the number of IMUs  controlled by this class
	 */
	private int numIMUs;
	
	/**
	 * Constructor that calls the super constructor (Sensor)
	 * @param numIMUs the number of IMU's this class controls
	 * @param attachedArduino	The arduino that this instrument is physically attached to
	 */
	public IMU(int numIMUs, ArduinoMessageHandler attachedArduino)
	{
		super(attachedArduino, MessageProtocol.ID1_IMU);
		this.numIMUs = numIMUs;
	}
	
	/**
	 * @precond hasData()
	 * @return the last pitch values recorded for each IMU
	 */
	public int[] getLastPitchValues()
	{
		int[] toReturn = new int[numIMUs];
		//We should have 1 byte of data per IMU				
		
		byte[] arrayAsBytes = super.cache.peek();
		//TODO check to make sure returned byte array from cache is as expected
		for (int i = 0; i < this.numIMUs; i++)
		{
			toReturn[i] = (int) arrayAsBytes[2*i];
		}
		return toReturn;
	} 
	
	/**
	 * @precond hasData
	 * @return the last roll values recorded for each IMU
	 */
	public int[] getLastRollValues()
	{
		int[] toReturn = new int[numIMUs];
		//We should have 1 byte of data per IMU				
		
		byte [] arrayAsBytes = super.cache.peek();
		//TODO check to make sure returned byte array from cache is as expected
		for (int i = 0; i < this.numIMUs; i++)
		{
			toReturn[i] = (int) arrayAsBytes[2*i+1];
		}
		return toReturn;
	} 
	
}
