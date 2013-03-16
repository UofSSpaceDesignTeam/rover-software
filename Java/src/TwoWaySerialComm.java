import gnu.io.CommPort;
import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;

import java.io.FileDescriptor;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

public class TwoWaySerialComm
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
   
    private StringContainer received = new StringContainer();
    //private StringContainer toBeSent = new StringContainer();
    
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
                
                (new Thread(new SerialReader(in, received))).start();                

            }
            else
            {
                System.out.println("Error: Only serial ports are handled by this example.");
            }
        }     
    }
    
    /** */
    public static class SerialReader implements Runnable 
    {
        InputStream in;
        StringContainer out;
        
        public SerialReader ( InputStream in , StringContainer out)
        {
            this.in = in;
            this.out = out;
        }
        
        public void run ()
        {
            byte[] buffer = new byte[1024];
            int len = -1;
            try
            {            	
                while ( ( len = this.in.read(buffer)) > -1 )
                {
                	//System.out.println("Not end of file yet");
           
                    out.setString(out.getString() + new String(buffer,0,len));
                	//System.out.print(new String(buffer,0,len));
                }
            }
            catch ( IOException e )
            {
                e.printStackTrace();
            }            
        }
    }   
    
    // only call after an appropriate delay after the last writeMessage() call
    public String readMessage(){
    	String str = received.getString();    	
    	received.setString("");
    	return str;    	
    }
    
    public void writeMessage(String msg){    	
    	OutputStream out;
		try {
			out = serialPort.getOutputStream();
			for (int i = 0 ; i < msg.length(); i++){				
				out.write(msg.charAt(i));			
			}
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
    }
    
    
    
    
    public static void main ( String[] args )
    {
    	TwoWaySerialComm port = new TwoWaySerialComm("/dev/ttyUSB0"); 	
    
        
        port.writeMessage("Hi\n");
        
        
        try {
			Thread.sleep(200);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
        System.out.println("Reading Message: " + port.readMessage());
        
        
        
        
       
        
        
        
    }
}