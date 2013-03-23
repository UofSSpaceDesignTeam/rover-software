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
	public static final byte ID1_MOTORS = 'M';
	public static final byte ID1_SERVOS = 'S';
	public static final byte ID1_POWER = 'P';
	public static final byte ID1_IMU = 'I';
	public static final byte ID1_WHEEL = 'W';
	public static final byte ID1_FORCE = 'F';
	public static final byte ID1_LED = 'L';
	public static final byte ID1_DEBUG = 'D';
	
	// Second Identifier Byte - Message Type
	public static final byte ID2_ENABLE_DISABLE = 'E';
	public static final byte ID2_DATA = 'D';
	public static final byte ID2_SET = 'S';
	public static byte ID2_TRANSFER_RATE = 'R';
	
	// message format constants	
	public static final int FIRST_DATA_BYTE = 5;	
	
	public static final int MAX_MSG_LENGTH = 1024;
	public static final byte START_BYTE = '#';
	public static final byte END_BYTE = '\n';
	
	
	
	
	public boolean valid(byte[] message)
	{
		return true; //TODO actually check for validity
	}
	
}
