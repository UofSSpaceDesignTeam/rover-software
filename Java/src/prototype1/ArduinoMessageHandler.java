package prototype1;

import java.util.Observable;
import java.util.Observer;
import java.util.HashMap;


public class ArduinoMessageHandler implements Observer {
	
	/**
	 * the serial port
	 */
	TwoWaySerialComm port;
	
	/**
	 * the sensors, indexed by the first ID byte
	 */
	HashMap<Byte,Sensor> sensors = new HashMap<Byte,Sensor>();
	
	
	
	public ArduinoMessageHandler(String serialPortURL){
		port = new TwoWaySerialComm(serialPortURL);
		port.addObserver(this);
	}
	
	/**
	 * Link a sensor to the arduino
	 * @param id1 the first id byte of the sensor (see MessageProtocol)
	 * @param sensor the sensor object
	 */
	public void addSensor(byte id1, Sensor sensor){
		sensors.put(id1, sensor);
	}
	
	/**
	 * Sends a message to the hardware arduino
	 * @param msg the byte array to send
	 */
	public void sendMessage(Message msg){
		port.sendMessage(msg.getMessage());		
	}

	@Override
	public void update(Observable arg0, Object arg1) {		
		byte [] bytes = (byte[]) arg1;
		
				
		Message msg = new Message(bytes);
		
		// debug output 
		System.out.print("Message from Arduino: ");		
		for (int i = 0; i < bytes.length; i++){			
			byte next = bytes[i];
			// check if byte is ascii readable
			/*if (next >= 32 && next <= 126)
				System.out.print((char) next);
			else */
				System.out.print("(" + next + ")");
				
		}
		
		System.out.println("");
		/*
		for (int i = 0; i < data.length; i++){
			System.out.print(data[i] + " ");
		}
		System.out.println("");
		*/
		
		// give this message to the right sensor		
		
		
		System.out.println("Got a message with ID " + (char) msg.getID1());
		
		Sensor target = sensors.get(msg.getID1());
		if (target != null){
			target.addData(msg.getData());
		}
		
	}	
	
}
