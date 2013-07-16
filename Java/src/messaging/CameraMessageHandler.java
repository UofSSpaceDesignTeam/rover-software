package messaging;

import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.Reader;
import java.util.Observable;

import com.google.gson.JsonIOException;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.google.gson.JsonSyntaxException;

public class CameraMessageHandler extends Observable implements Runnable
{
	
	private Process cameraProcess;
	private Reader cameraOut;
	private BufferedWriter cameraIn;
	private boolean stopped = false;
	/**
	 *  The current information from the camera with timestamp
	 */
	public JsonObject cameraData;
	
	public CameraMessageHandler(){
		ProcessBuilder pb = new ProcessBuilder("/home/fit-pc/workspace/openni_grabber-Debug@RoverPCL/jsonTest");
		
		try
		{
			cameraProcess = pb.start();
		} catch (IOException e)
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		InputStream instream = cameraProcess.getInputStream();		
		OutputStream outstream = cameraProcess.getOutputStream();
		
		cameraOut = new InputStreamReader(instream);
		cameraIn = new BufferedWriter(new OutputStreamWriter(outstream));
		
	}
	
	public void stop()
	{
		this.stopped = true;
	}
	
	public void resume()
	{
		this.stopped = false;
	}
	
	public void parse()
	{
		while(!stopped)
		{
				JsonParser inputReader = new JsonParser();
				try
				{
					if(cameraOut.ready())
					{
						cameraData = inputReader.parse(cameraOut).getAsJsonObject();
						this.setChanged();
						this.notifyObservers();						
						System.out.println(inputReader.parse(cameraOut).toString());
					}
				} catch (JsonIOException e)
				{
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (JsonSyntaxException e)
				{
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (IOException e)
				{
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
		}
	}
	
	/**
	 * @param args
	 * @throws IOException 
	 */
	public static void main(String[] args) throws IOException
	{
		CameraMessageHandler cmh = new CameraMessageHandler();
		cmh.cameraIn.write("bla\n");
		cmh.cameraIn.flush();
		
		
		

	}

	@Override
	public void run()
	{
		parse();
	}

}
