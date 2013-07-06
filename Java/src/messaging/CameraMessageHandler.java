package messaging;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;

import com.google.gson.stream.JsonReader;

public class CameraMessageHandler
{
	
	private Process cameraProcess;
	private BufferedReader cameraOut;
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
		
		cameraOut = new BufferedReader(new InputStreamReader(instream));
		cameraIn = new BufferedWriter(new OutputStreamWriter(outstream));
		
		
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
		System.out.println(cmh.cameraOut.readLine());
		
		

	}

}
