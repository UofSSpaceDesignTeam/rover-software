package prototype1;

/**
 * 
 * Wrapper around the byte array that represents a message, with
 * some convenience methods
 *
 */
public class Message {
	byte[] message; 
	
	/**
	 * Constructor with the message byte array
	 * @param msg the byte array
	 */
	public Message(byte[] msg){
		this.message = msg;		
	}
	
	/**
	 * Constructor with the two ID bytes and the data
	 * @param id1 first id byte
	 * @param id2 second id byte
	 * @param data the data to be sent
	 */
	public Message(byte id1, byte id2, byte[] data){
		message = new byte[data.length + 2 + 2 + 1 + 1];
		
	} 
	
	
	public byte getID1(){
				
	}
	
	public byte getID2(){
		
	}
	
	public byte[] getData(){
		
	}
	
	
	
}
