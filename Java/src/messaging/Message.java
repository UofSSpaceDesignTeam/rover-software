package messaging;

/**
 * 
 * Wrapper around the byte array that represents a message, with
 * some convenience methods
 *
 * This is where most of the protocol-specific behaviour is.
 * If the message protocol changes, this is the file to update.
 */
public class Message {
	byte[] message; 
	
	/**
	 * Constructor with the message byte array 
	 * To be used to by the Arduino to send data
	 * @param msg the byte array
	 * TODO check that message is valid
	 */
	public Message(byte[] msg)
	{
		this.message = msg;		
	}
	
	/**
	 * Constructor with the two ID bytes and the data 
	 * To be used to send data to the Arduino
	 * @param id1 first id byte
	 * @param id2 second id byte
	 * @param data the data to be sent
	 */
	public Message(byte id1, byte id2, byte[] data)
	{
		message = new byte[data.length + 2 + 2 + 1 + 1];
		message[0] = MessageProtocol.START_BYTE;
		message[1] = id1;
		message[2] = id2;
		message[3] = (byte) (data.length / 256);
	    message[4] = (byte) (data.length % 256);
	    for (int i = 0; i < data.length; i++){
	    	message[i+MessageProtocol.FIRST_DATA_BYTE] = data[i];
	    }
	    message[message.length - 1] = MessageProtocol.END_BYTE;		
	} 
	
	/**
	 * @return the entire message as a byte array
	 */
	public byte[] getMessage()
	{
		return this.message;
	}
	/**
	 * @return the first id byte
	 */
	public byte getID1()
	{
		return message[1];				
	}
	
	/**
	 * @return the second id byte
	 */
	public byte getID2()
	{
		return message[2];		
	}
	
	/**
	 * @return the data array
	 */
	public byte[] getData()
	{
		// figure out the data length
		int dataLength = MessageProtocol.dataLength(message);
		
		// write data array
		byte[] data = new byte[dataLength];
		for (int i = 0; i < dataLength; i++){
			data[i] = message[i + MessageProtocol.FIRST_DATA_BYTE];
		}
		
		return data;		
	}
	
	
	
}
