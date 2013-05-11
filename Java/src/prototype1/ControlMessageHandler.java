package prototype1;

import java.net.ServerSocket;
import java.net.Socket;
import java.util.HashMap;
import java.util.Observable;
import java.util.Observer;


public class ControlMessageHandler implements Observer, Runnable
{
	/**
	 * the instruments, indexed by the first ID byte
	 */
	HashMap<Byte,Instrument> instruments = new HashMap<Byte,Instrument>();
	
	TwoWayCommunicator comm; 
	
	@Override
	public void run() {
		try
		{
			ServerSocket server = new ServerSocket(7050);
			Socket socket;
			
		
			while(true)
			{
				socket = server.accept();
				System.out.println("Accepted connection from " + socket.getInetAddress());
				comm = new TwoWayCommunicator(socket.getInputStream(), socket.getOutputStream());	
				comm.addObserver(this);
			}
		}catch(Exception E) {}		
	}
	
	
	/**
	 * Link an instrument to the control
	 * @param id1 the first id byte of the instrument (see MessageProtocol)
	 * @param instrument the instrument object
	 */
	public void addInstrument(byte id1, Instrument instrument){
		instruments.put(id1, instrument);
		System.out.println(instruments.size());
	}

	@Override
	public void update(Observable o, Object arg) 
	{
		byte [] bytes = (byte[]) arg;		
		Message msg = new Message(bytes);	
		
		
		for (int i = 0; i < bytes.length ; i++){
			System.out.print("(" + bytes[i]+ ")");
		}
		System.out.println("|");
		
		// handle the message		
		Instrument target = instruments.get(msg.getID1());		
		if(target != null)
		{
			
			if(msg.getID1() == MessageProtocol.ID1_MOTORS)
			{
				Motors motor = (Motors)target;
				
				if(msg.getID2() == MessageProtocol.ID2_ENABLE_DISABLE)
				{
					boolean enable = msg.getData()[0] == MessageProtocol.DATA_BYTE_ENABLE;
					System.out.println("Setting motors to : " + enable);
					motor.setEnabled(enable);
				}
				else if (msg.getID2() == MessageProtocol.ID2_SET)
				{					
					motor.setRotationSpeed( msg.getData());
				}
			}
		}

		
		// comm.sendMessage(msg);
		
	}



	
	
}





