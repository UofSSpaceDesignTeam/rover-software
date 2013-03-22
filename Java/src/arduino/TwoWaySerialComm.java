package arduino;
import java.util.Observable;
import gnu.io.CommPort;
import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;

import java.io.FileDescriptor;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

import prototype1.MessageProtocol;

public class TwoWaySerialComm extends Observable
{
	/**
	 * 
	 * @param usbURL the url to the USB port we want to connect to
	 */
    public TwoWaySerialComm(String usbURL){
    	super();
    	try {
			connect(usbURL);
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
    }
       
    private SerialPort serialPort;
    
    void connect ( String portName ) throws Exception
    {
        CommPortIdentifier portIdentifier = CommPortIdentifier.getPortIdentifier(portName);
        if ( portIdentifier.isCurrentlyOwned() )
        {
            System.out.println("Error: Port is currently in use");
        }
        else
        {
            CommPort commPort = portIdentifier.open(this.getClass().getName(),2000);
            
            if ( commPort instanceof SerialPort )
            {
                SerialPort serialPort = (SerialPort) commPort;
                serialPort.setSerialPortParams(9600,SerialPort.DATABITS_8,SerialPort.STOPBITS_1,SerialPort.PARITY_NONE);
                
                InputStream in = serialPort.getInputStream();
                OutputStream out = serialPort.getOutputStream();
                
                this.serialPort = serialPort;
                
                (new Thread(new SerialReader(this, in))).start();                

            }
            else
            {
                System.err.println("Error: Only serial ports are handled by this example.");
            }
        }     
    }
    
    /** */
    public static class SerialReader implements Runnable 
    {
    	TwoWaySerialComm comm;
        InputStream in;
        
        
        public SerialReader (TwoWaySerialComm comm, InputStream in)
        {
        	this.comm = comm;
            this.in = in;            
        }
        
        public void run ()
        {
        	byte[] inBuffer = new byte[MessageProtocol.MAX_MSG_LENGTH];
        	
            int nextByte = -1;
            int nextIndex = 0;
            try
            {     	
            	
                while ((nextByte = this.in.read()) != -1)
                {
                	// add the byte to the buffer
                	inBuffer[nextIndex] = (byte) nextByte;
                	nextIndex++;
                	
                	// if end of message, send message to observers
                	if (nextByte == MessageProtocol.END_BYTE){
                		byte[] message = new byte[nextIndex];
                		for (int i = 0; i < nextIndex; i++){
                			message[i] = inBuffer[i];
                		}                		
                		comm.notifyObservers(message);
                	}
                	
                	// clear buffer
                	nextIndex = 0;                
                }
                
            }
            catch ( IOException e )
            {
                e.printStackTrace();
            }            
        }
    }   
    
     
    public void sendMessage(byte[] message){    	
    	OutputStream out;
		try {
			out = serialPort.getOutputStream();
			for (int i = 0 ; i < message.length; i++){				
				out.write(message[i]);			
			}
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
    }        
}