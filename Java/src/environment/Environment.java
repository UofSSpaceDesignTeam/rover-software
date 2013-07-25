package environment;

import instruments.Camera;

import java.awt.geom.Dimension2D;
import java.awt.geom.Point2D;
import java.util.LinkedList;
import java.util.Observable;
import java.util.Observer;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;

public class Environment implements MapInterface, Observer
{
	Camera camera;
	public Environment()
	{
		camera = new Camera();
		camera.addObserver(this);
	}

	@Override
	public Dimension2D size()
	{
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Point2D robotPosition()
	{
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public double robotOrientation()
	{
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public LinkedList<Obstacle> obstacles()
	{
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void update(Observable arg0, Object arg1)
	{
		JsonObject object = camera.getLatest();
		System.out.println(object.getAsJsonArray("planes").toString());
		JsonArray planes = object.getAsJsonArray("planes");
		for(int i = 0; i< planes.size(); i++)
		{
			
		}
	}
	
	public static void main(String[] args)
	{
		Environment e = new Environment();
	}

}
