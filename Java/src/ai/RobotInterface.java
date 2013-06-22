package ai;

import java.awt.geom.Point2D;
import java.util.HashMap;

import prototype1.Instrument;
import prototype1.Sensor;


/*
 * IMPORTANT: How do we prevent the AI from changing the Robot's (and other RealWorldObjects') situations,
 * while allowing the data-driven backend to freely modify them?
 * 
 * We need to somehow give the AI a clean interface - for now, we handle by 
 */

/**
 * represents the robot
 * @author fit-pc
 *
 */
public interface RobotInterface {
	
	
	/*
	 * NAVIGATION INTERFACE
	 */
	
	/**
	 * commands robot to move in a straight line at the given speed
	 * @param speed the speed in meters per second (positive to go forward, negative to go backward)
	 */
	public void move(double speed);
	
	/**
	 * commands robot to spin in place at the given speed
	 * @param speed the speed in radians per second (positive = counterclockwise)
	 */
	public void spin(double speed);
	
	/**
	 * commands robot to move in a circle of the given radius with the given speed
	 * @param radius radius of curved path, in meters
	 * @param speed linear speed of robot in meters per second
	 */
	public void curve(double radius, double speed);
	
	
	
	
	
	
	
	
	
}
