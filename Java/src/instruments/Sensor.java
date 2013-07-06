package instruments;

import java.util.LinkedList;

import environment.Situation;

import messaging.ArduinoMessageHandler;
import messaging.MessageProtocol;
import messaging.TimestampedValue;



//TODO MAKE INSTANCE OF OBSERVABLE
/**
 * An abstract class to store functionality common to all the sensors
 *
 */
public abstract class Sensor extends ArduinoInstrumentArray {
	
	/**
	 * The maximum number of data packets that can be stored
	 * TODO fix this later
	 */
	public static final int MAX_CACHE_SIZE = 30;
	
	/**
	 * Stores the most recent data from the sensor in reverse chronological order (oldest at the end)
	 * will only store MAX_CACHE_SIZE items
	 */
	protected LinkedList<TimestampedValue<byte[]>> cache = new LinkedList<TimestampedValue<byte[]>>();
	
	/**
	 * Constructor that calls the super constructor (Instrument)
	 * @param attachedArduino	The arduino that this instrument is physically attached to
	 * @param thisId	the [ID1] byte that represents this instrument (M for motor; I for IMU, etc)
	 */
	Sensor(ArduinoMessageHandler attachedArduino, byte thisId, Situation[] s, int numOfSensors)
	{
		super(attachedArduino, thisId,s,numOfSensors);
	}
	
	
	
	
	
	/** 
	 * @return true if the cache has any data, false otherwise
	 */
	public boolean hasData()
	{
		return cache.size() != 0;
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
		cache.addFirst(new TimestampedValue<byte[]>(data));
		setChanged();
		notifyObservers();
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
