package prototype1;

/**
 * Stores constants and convenience methods related to the Messaging Protocol
 * so we don't have magic numbers all over our code
 *
 * Add things as you see fit; if you can think of more descriptive names, refactor.
 */
public abstract class MessageProtocol {
	// First Identifier Byte - Message Target
	// TODO add javadoc
	public static byte ID1_MOTORS = 'M';
	public static byte ID1_SERVOS = 'S';
	public static byte ID1_POWER = 'P';
	public static byte ID1_IMU = 'I';
	public static byte ID1_WHEEL = 'W';
	public static byte ID1_FORCE = 'F';
	public static byte ID1_LED = 'L';
	public static byte ID1_DEBUG = 'D';
	
	// Second Identifier Byte - Message Type
	public static byte ID2_ENABLE_DISABLE = 'E';
	public static byte ID2_DATA = 'D';
	public static byte ID2_SET = 'S';
	
	// message format constants	
	public static int FIRST_DATA_BYTE = 5;	
	
	// convenience methods for processing messages - prefer using these to manipulating the byte array yourself
	/**
	 * @param message the message to be processed
	 * @return the first ID byte of the message
	 * @precond valid(message)
	 */
	public static int getID1(byte[] message){
		return message[1];
	}
	
	/**
	 * @param message the message to be processed
	 * @return the first ID byte of the message
	 * @precond valid(message)
	 */
	public static int getID2(byte[] message){
		return message[2];
	}
	
	/**
	 * @param message the message to be processed
	 * @return the data length indicated in the message
	 * @precond valid(message)
	 */
	public static int getDataLength(byte[] message){
		return message[3]*256 + message[4];
	}
	
	
	
	public boolean valid(byte[] message){
		return true; //TODO actually check for validity
	}
	
}
