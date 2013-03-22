package prototype1;

import arduino.Arduino;

public abstract class Instrument 
{
	/**
	 * Store the ID of the sensor/actuator
	 */
	protected byte id;
	
	/**
	 * Store the Arduino that this instrument is attached to
	 */
	protected Arduino associatedArduino;
	
	/**
	 * Constructor for this class
	 * @param attachedArduino	The arduino that this instrument is physically attached to
	 * @param thisId	the [ID1] byte that represents this instrument (M for motor; I for IMU, etc)
	 */
	Instrument(Arduino attachedArduino, byte thisId)
	{
		id = thisId;
		associatedArduino = attachedArduino;
	}
	
	/**
	 * The method to send data to the arduino in the proper format
	 * @param id2	The [ID2] byte to send, representing the command
	 * @param data	The data array to send, representing [B1]...[Bn], where n is the length of the data sent
	 */
	protected void sendToArduino(byte id2, byte[] data)
	{
		int length = data.length;
		
		//Fill in the general data stream using the data passed in
		// #[ID1][ID2][DL1][DL2][B1]...[Bn][\n]
		byte enableMessage[] = new byte[length+6];
		
		enableMessage[0] = '#';
		enableMessage[1] = this.id;
		enableMessage[2] = id2;
		// TODO fix so uses MessageProtocol method
		enableMessage[3] = (byte) (length/256);
		enableMessage[4] = (byte) (length%256);
		
		int i = 0;
		for(i = 0; i < length; i++)
		{
			enableMessage[i+5] = data[i];
		}
		enableMessage[i] = '\n';
		
		associatedArduino.sendMessage(enableMessage);
	}
	
	/**
	 * Sets the instrument to be enabled or disabled
	 * @param enable	true to enable instrument, false to disable
	 */
	public void setEnabled(boolean enable)
	{
		//Fill in the data[] to send to the Arduino
		byte toSend[] = new byte[1];
		if(enable)
		{
			toSend[0] = '1';
		}
		else
		{
			toSend[0] = '0';
		}
		this.sendToArduino(MessageProtocol.ID2_ENABLE_DISABLE, toSend);
	}
	
}
