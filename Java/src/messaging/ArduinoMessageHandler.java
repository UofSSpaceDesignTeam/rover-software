package messaging;

import gnu.io.CommPort;

import gnu.io.CommPortIdentifier;
import gnu.io.NoSuchPortException;
import gnu.io.PortInUseException;
import gnu.io.SerialPort;
import gnu.io.UnsupportedCommOperationException;

import instruments.Sensor;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Observable;
import java.util.Observer;
import java.util.HashMap;

public class ArduinoMessageHandler implements Observer {
	
	/**
	 * the communicator 
	 */
	TwoWayCommunicator comm;
	
	/**
	 * the sensors, indexed by the first ID byte
	 */
	HashMap<Byte,Sensor> sensors = new HashMap<Byte,Sensor>();
	
	
	
	public ArduinoMessageHandler(String serialPortURL){
		// connect to the serial port 
		CommPortIdentifier portIdentifier;
		try {
			portIdentifier = CommPortIdentifier.getPortIdentifier(serialPortURL);
		} catch (NoSuchPortException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			return;
		}
        if ( portIdentifier.isCurrentlyOwned() )
        {
            System.out.println("Error: Port is currently in use");
        }
        else
        {
            CommPort commPort;
			try {
				commPort = portIdentifier.open(this.getClass().getName(),2000);
			} catch (PortInUseException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
				return;
			}
            
            if ( commPort instanceof SerialPort )
            {
            	
                SerialPort serialPort = (SerialPort) commPort;
                try {
					serialPort.setSerialPortParams(115200,SerialPort.DATABITS_8,SerialPort.STOPBITS_1,SerialPort.PARITY_NONE);
				} catch (UnsupportedCommOperationException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
                
                
				try {
					InputStream in = serialPort.getInputStream();
					OutputStream out = serialPort.getOutputStream();
					
					// connect and start listening
	                comm = new TwoWayCommunicator(in, out);
	                comm.addObserver(this); 
	                
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
                
                
                
                             

            }
            else
            {
                System.err.println("Error: Only serial ports are handled by this example.");
            }
        }   		
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
		comm.sendMessage(msg.getMessage());		
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
			if (next >= 32 && next <= 126)
				System.out.print((char) next);
			else 
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
