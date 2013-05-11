package prototype1;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.PrintStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.HashMap;
import java.util.Observable;
import java.util.Observer;


public class ControlMessageHandler implements Observer
{
	/**
	 * the instruments, indexed by the first ID byte
	 */
	HashMap<Byte,Instrument> instruments = new HashMap<Byte,Instrument>();
	
	TwoWayCommunicator comm; 
	
	public ControlMessageHandler()
	{
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
	}

	@Override
	public void update(Observable o, Object arg) 
	{
		byte [] bytes = (byte[]) arg;		
		Message msg = new Message(bytes);
		
		System.out.println("got a message");
		
		
		
		// handle the message
		
		// comm.sendMessage(msg);
		
	}
	
}





