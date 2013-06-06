package prototype1;

import java.util.Observable;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

/**
 * A general class to send and receive messages to a given pair of
 * input and output streams
 * @author usst
 *
 */
public class TwoWayCommunicator extends Observable
{
	/**
	 * The communicator's input stream
	 */
	private InputStream input;
	
	/**
	 * The communicator's output stream
	 */
	private OutputStream output;
	/**
	 * 
	 * @param in the input stream
	 * @param out the output stream
	 */
    public TwoWayCommunicator(InputStream in, OutputStream out)
    {
    	super();
    	input = in;
    	output = out;    
    	new Thread(new SerialReader(this, input)).start();  
    }
       
    
    /** */
    public static class SerialReader implements Runnable 
    {
    	TwoWayCommunicator comm;
        InputStream in;
        
        
        public SerialReader (TwoWayCommunicator comm, InputStream in)
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
            	int dataLength = 0;
            	int lastByteIndex = 0;
            	boolean dataLengthSet = false;
                while ((nextByte = this.in.read()) != -1)
                {
                	
                	
                	// add the byte to the buffer
                	inBuffer[nextIndex] = (byte) nextByte;    
                	
                	// if this is the last byte of the data length, set the data length
                	if (nextIndex == MessageProtocol.DATA_LENGTH_2){
                		dataLength = MessageProtocol.dataLength(inBuffer);
                		lastByteIndex = MessageProtocol.endByteIndex(inBuffer);
                		dataLengthSet = true;
                	}
                	
                	
                	nextIndex += 1;
                	
                	
                	
                	
                	// if end of message, send message to observer
                	if (dataLengthSet && nextIndex-1 == lastByteIndex){
	                	if (nextByte == MessageProtocol.END_BYTE){                		
	                		byte[] message = new byte[nextIndex];                		
	                		for (int i = 0; i < nextIndex; i++){
	                			message[i] = inBuffer[i];
	                		}      
	                		
	                		comm.setChanged();
	                		comm.notifyObservers(message);
	                		// clear buffer
	                    	nextIndex = 0; 
	                	} else {
	                		throw new RuntimeException("HORRIFIC ERROR");
	                	}
                	}   
                	
                	               
                }
                System.out.println("Stoppped Reading");  
                
            }
            catch ( IOException e )
            {
                e.printStackTrace();
            }            
        }
    }   
    
     
    public void sendMessage(byte[] message){    	    	
		try {			
			for (int i = 0 ; i < message.length; i++){				
				output.write(message[i]);					
			}
			output.flush();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
    }        
}