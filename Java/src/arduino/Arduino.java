package arduino;

import java.util.Observable;
import java.util.Observer;

public class Arduino implements Observer {
	TwoWaySerialComm port = new TwoWaySerialComm("/dev/ttyUSB0");
	
	public Arduino(){
		port.addObserver(this);
	}
	
	/**
	 * Sends a message to the hardware arduino
	 * @param message the byte array to send
	 */
	public void sendMessage(byte[] message){
		port.sendMessage(message);		
	}

	@Override
	public void update(Observable arg0, Object arg1) {
		byte [] message = (byte[]) arg1;
		
		// give this message to the right person
		System.out.println("Got a message starting with byte " + message[0]);
	}	
	
}
