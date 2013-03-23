package arduino;

import java.util.Observable;
import java.util.Observer;
import java.util.HashMap;

import prototype1.Instrument;
import prototype1.Message;
import prototype1.Sensor;

public class Arduino implements Observer {
	
	/**
	 * the serial port
	 */
	TwoWaySerialComm port;
	
	/**
	 * the sensors, indexed by the first ID byte
	 */
	HashMap<Byte,Sensor> sensors = new HashMap<Byte,Sensor>();
	
	
	
	public Arduino(String serialPortURL){
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
		
		// give this message to the right sensor
		System.out.println("Got a message with ID " + msg.getID1());
		
		Sensor target = sensors.get(msg.getID1());
		if (target != null){
			target.addData(msg.getData());
		}
	}	
	
}
