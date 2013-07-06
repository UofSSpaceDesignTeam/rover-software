package prototype1;

import ai.Situation;

public class IMU extends Sensor
{
	private static Situation[] toSingletonArray(Situation s){
		Situation[] ss = new Situation[1];
		ss[0] = s;
		return ss;
	}
	/**
	 * Constructor that calls the super constructor (Sensor)
	 * @param numIMUs the number of IMU's this class controls
	 * @param attachedArduino	The arduino that this instrument is physically attached to
	 */
	public IMU(ArduinoMessageHandler attachedArduino, Situation s)
	{
		super(attachedArduino, MessageProtocol.ID1_IMU, toSingletonArray(s), 1);
	}
	
	/**
	 * @precond hasData()
	 * @return the last pitch values recorded for each IMU
	 */
	public int[] getLastPitchValues()
	{
		int[] toReturn = new int[this.size()];
		//We should have 1 byte of data per IMU				
		
		byte[] arrayAsBytes = super.cache.peek().getValue();
		//TODO check to make sure returned byte array from cache is as expected
		for (int i = 0; i < this.size(); i++)
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
		int[] toReturn = new int[this.size()];
		//We should have 1 byte of data per IMU				
		
		byte [] arrayAsBytes = super.cache.peek().getValue();
		//TODO check to make sure returned byte array from cache is as expected
		for (int i = 0; i < this.size(); i++)
		{
			toReturn[i] = (int) arrayAsBytes[2*i+1];
		}
		return toReturn;
	} 
	
}
