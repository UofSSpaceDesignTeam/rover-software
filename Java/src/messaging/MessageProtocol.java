package messaging;

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
	public static final byte ID1_RANGEFINDER = 'R';
	
	// Second Identifier Byte - Message Type
	public static final byte ID2_ENABLE_DISABLE = 'E';
	public static final byte ID2_DATA = 'D';
	public static final byte ID2_SET = 'S';
	public static final byte ID2_QUERY = 'Q';
	public static final byte ID2_TRANSFER_RATE = 'R';	
	
	
	
	// message format constants	
	public static final int FIRST_DATA_BYTE = 5;	
	
	public static final int MAX_MSG_LENGTH = 1024;
	public static final byte START_BYTE = '#';
	public static final byte END_BYTE = '\n';
	
	public static final int DATA_BYTE_ENABLE = '1';
	public static final int DATA_BYTE_DISABLE = '0';
	
	public static final int DATA_LENGTH_1 = 3;
	public static final int DATA_LENGTH_2 = 4;
	
	public static int dataLength(byte[] message){
		return message[DATA_LENGTH_1] * 256 + message[DATA_LENGTH_2];
	}
	
	// where the ending newline should be based on data length
	public static int endByteIndex(byte[] message){
		return dataLength(message) + 5;
	}
	
	
	
	
	public boolean valid(byte[] message)
	{
		return true; //TODO actually check for validity
	}
	
}
