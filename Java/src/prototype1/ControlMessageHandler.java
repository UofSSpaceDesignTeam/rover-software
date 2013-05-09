package prototype1;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.PrintStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Observable;
import java.util.Observer;


public class ControlMessageHandler implements Observer
{
	
	
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
				comm = new TwoWayCommunicator(socket.getInputStream(), socket.getOutputStream());				
			}
		}catch(Exception E) {}
	}

	@Override
	public void update(Observable o, Object arg) 
	{
		byte [] bytes = (byte[]) arg;		
		Message msg = new Message(bytes);
		 
		// handle the message
		
		// comm.sendMessage(msg);
		
	}
	
}





