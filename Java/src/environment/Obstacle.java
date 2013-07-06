package environment;

import java.awt.geom.Point2D;

public abstract class Obstacle {
	
	/**
	 * @return the position of the obstacle
	 */
	public abstract Point2D getPosition();

	/**
	 * @return the radius of the obstacle, in meters
	 */
	public abstract double getRadius();

}
