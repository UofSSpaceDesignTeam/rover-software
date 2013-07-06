package ai;

import java.awt.geom.Point2D;

/**
 * stores a position and an orientation (in 2D, for now)
 * 
 * @author fit-pc
 *
 */
public class Situation {
	
	
	/**
	 * location in 2D space (measured in meters)
	 */
	private Point2D location;
	
	/**
	 * Angle from positive x-axis, in radians
	 */
	private double angle; 
	
	/**
	 * default constructor (position at origin)
	 */
	public Situation(){
		location = new Point2D.Double(0,0);
		angle = 0;
	}
	
	/**
	 * @return the location
	 */
	public Point2D getLocation() {
		return location;
	}

	/**
	 * @param location the location to set
	 */
	public void setLocation(Point2D location) {
		this.location = location;
	}

	/**
	 * @return the angle
	 */
	public double getAngle() {
		return angle;
	}

	/**
	 * @param angle the angle to set
	 */
	public void setAngle(double angle) {
		this.angle = angle;
	}

	
}
