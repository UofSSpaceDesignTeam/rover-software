package prototype1;

import java.util.Observable;

import ai.Situation;


public abstract class Instrument extends Observable
{
	/**
	 * Store the ID of the sensor/actuator
	 */
	protected byte id;
	
	/**
	 * Store the Arduino that this instrument is attached to
	 */
	protected ArduinoMessageHandler associatedArduino;
	
	/**
	 * the instrument's situation (position and orientation)
	 */
	protected Situation situation;
	
	/**
	 * @return the situation
	 */
	public Situation getSituation() {
		return situation;
	}

	/**
	 * Constructor for this class
	 * @param attachedArduino	The arduino that this instrument is physically attached to
	 * @param thisId	the [ID1] byte that represents this instrument (M for motor; I for IMU, etc)
	 * @param s	the situation of the instrument
	 */
	Instrument(ArduinoMessageHandler attachedArduino, byte thisId, Situation s)
	{
		id = thisId;
		associatedArduino = attachedArduino;
		this.situation = s;
	}
	
	/**
	 * The method to send data to the arduino in the proper format
	 * @param id2	The [ID2] byte to send, representing the command
	 * @param data	The data array to send, representing [B1]...[Bn], where n is the length of the data sent
	 */
	protected void sendToArduino(byte id2, byte[] data)
	{		
		Message messageToSend = new Message(this.id, id2, data);
		associatedArduino.sendMessage(messageToSend);
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
			toSend[0] = MessageProtocol.DATA_BYTE_ENABLE;
		}
		else
		{
			toSend[0] = MessageProtocol.DATA_BYTE_DISABLE;
		}
		this.sendToArduino(MessageProtocol.ID2_ENABLE_DISABLE, toSend);
	}
	
}
