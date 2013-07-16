package instruments;

import java.util.LinkedList;
import java.util.Observable;
import java.util.Observer;

import com.google.gson.JsonObject;

import messaging.CameraMessageHandler;

import environment.Situation;

public class Camera extends Observable implements Observer
{
	/**
	 * The maximum number of data packets that can be stored
	 * TODO fix this later
	 */
	public static final int MAX_CACHE_SIZE = 30;
	
	private Situation situation;
	private CameraMessageHandler camera;
	private Thread parseThread;
	protected LinkedList<JsonObject> cache = new LinkedList<JsonObject>();
	
	public Camera()
	{
		camera = new CameraMessageHandler();
		parseThread = new Thread( camera, "Camera Thread");
		camera.addObserver(this);
		ResumeCamera();
	}
	
	/**
	 * @return the latest data from the camera
	 */
	public JsonObject getLatest()
	{
		return cache.getFirst();
	}
	
	/**
	 * @return the cache of camera data
	 */
	public LinkedList<JsonObject> getCache()
	{
		return cache;
	}
	
	/**
	 * stops the thread that is listening to the camera
	 */
	public void ShutdownCamera()
	{
		camera.stop();
	}
	
	/**
	 * Resume thread listening to camera
	 */
	public void ResumeCamera()
	{
		camera.resume();
		parseThread.start();
	}

	/**
	 * @return the situation
	 */
	public Situation getSituation()
	{
		return situation;
	}

	/**
	 * @param situation the situation to set
	 */
	public void setSituation(Situation situation)
	{
		this.situation = situation;
	}

	@Override
	public void update(Observable arg0, Object arg1)
	{
		if(cache.size() < MAX_CACHE_SIZE)
		{
			cache.addFirst(camera.cameraData);
		}
		else
		{
			cache.removeLast();
			cache.addFirst(camera.cameraData);
		}
		this.setChanged();
		this.notifyObservers();
		
	}
	

}
