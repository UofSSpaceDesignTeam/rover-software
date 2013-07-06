package messaging;

import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.Reader;

import com.google.gson.JsonParser;
import com.google.gson.stream.JsonReader;

public class CameraMessageHandler
{
	
	private Process cameraProcess;
	private Reader cameraOut;
	private BufferedWriter cameraIn;
	
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
		parse();
		
	}
	
	public void parse()
	{
		while(true)
		{
				JsonParser inputReader = new JsonParser();
				System.out.println(inputReader.parse(cameraOut).toString());
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

}
