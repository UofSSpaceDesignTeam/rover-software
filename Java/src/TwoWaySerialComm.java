import gnu.io.CommPort;
import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;

import java.io.FileDescriptor;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

public class TwoWaySerialComm
{
    public TwoWaySerialComm()
    {
        super();
    }
    
    private StringContainer received = new StringContainer();
    private StringContainer toBeSent = new StringContainer();
    
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
                
                (new Thread(new SerialReader(in, received))).start();
                (new Thread(new SerialWriter(out, toBeSent))).start();

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
            	System.out.println("Not end of file yet");
                while ( ( len = this.in.read(buffer)) > -1 )
                {
                	//System.out.println("Not end of file yet");
           
                    //out.setString(out.getString() + new String(buffer,0,len));
                	System.out.print(new String(buffer,0,len));
                }
            }
            catch ( IOException e )
            {
                e.printStackTrace();
            }            
        }
    }

    /** */
    public static class SerialWriter implements Runnable 
    {
        OutputStream out;
        StringContainer in;
        
        public SerialWriter ( OutputStream out, StringContainer in)
        {
            this.out = out;
            this.in = in;
        }
        
        public void run ()
        {
            try
            {                
                int c = 0;
                /*while ( ( c = System.in.read()) > -1 )
                {
                    this.out.write(c);
                }     */
                
            	while (true){
            		String str = in.getString();
            		if (str.length() != 0){
            			// write the first char
            			this.out.write(str.charAt(0));
            			System.out.println("Printed: " + str.charAt(0));
            			// keep the remainder of the string
            			in.setString(str.substring(1));
            		}
            	}
            }
            catch ( IOException e )
            {
                e.printStackTrace();
            }            
        }
    }
    
    public String readMessage(){
    	String str = received.getString();    	
    	received.setString(null);
    	return str;    	
    }
    
    public void writeMessage(String msg){
    	toBeSent.setString(msg);
    }
    
    
    
    
    public static void main ( String[] args )
    {
    	TwoWaySerialComm port = new TwoWaySerialComm(); 	
    	
        try
        {
            port.connect("/dev/ttyUSB0");
        }
        catch ( Exception e ) {    
            e.printStackTrace();
        }
        
        port.writeMessage("Hi\n");
        
        /*try {
			//Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}*/
        System.out.println(port.readMessage());
        
        
    }
}