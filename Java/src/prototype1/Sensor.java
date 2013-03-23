package prototype1;

import java.util.LinkedList;

/**
 * An abstract class to store functionality common to all the sensors
 *
 */
public abstract class Sensor extends Instrument {
	
	/**
	 * The maximum number of data packets that can be stored
	 * TODO fix this later
	 */
	public static final int MAX_CACHE_SIZE = 30;
	
	/**
	 * Field to store the data for this sensor
	 */
	protected byte[] thisData;
	
	/**
	 * Stores the most recent data from the sensor in reverse chronological order (oldest at the end)
	 * will only store MAX_CACHE_SIZE items
	 */
	protected LinkedList<byte[]> cache = new LinkedList<byte[]>();
	
	/**
	 * Constructor that calls the super constructor (Instrument)
	 * @param attachedArduino	The arduino that this instrument is physically attached to
	 * @param thisId	the [ID1] byte that represents this instrument (M for motor; I for IMU, etc)
	 */
	Sensor(Arduino attachedArduino, byte thisId)
	{
		
		super(attachedArduino, thisId);
	}
	
	/**
	 * Add data to the sensor object, this will be done at the data transfer rate specified, by the Arduino
	 * @param data	The data bytes [B1]...[Bn]
	 */
	public void addData(byte[] data) 
	{
		// check to make sure cache is not full
		// if it is then remove last item in the cache
		if (cache.size() >= MAX_CACHE_SIZE)
		{
			cache.removeLast();
		}
		cache.addFirst(data);
	}
	
	/**
	 * Set the data transfer rate
	 * @param ms	the rate in milliseconds
	 */
	public void setDataTransferRate(int ms)
	{
		byte[] timeData = new byte[2];
		timeData[0] = (byte) (ms/256);
		timeData[1] = (byte) (ms%256);
		this.sendToArduino(MessageProtocol.ID2_TRANSFER_RATE, timeData);
	}
	
}
