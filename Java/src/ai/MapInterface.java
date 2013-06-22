package ai;

import java.awt.geom.Dimension2D;
import java.awt.geom.Point2D;
import java.util.LinkedList;


public interface MapInterface {
	
	/*
	 * 	^	  |
	 * 	|	  |
	 * 	Y	  |
	 * 		  |
	 * Height |
	 * 		  |
	 * 		  |
	 * 		  |
	 *      (0,0)-------------------
	 *             Width  (X -> )
	 */
	
	/**
	 * @return the dimension of the map
	 */
	public Dimension2D size();
	
	/**
	 * @return the position of the robot on the map
	 */
	public Point2D robotPosition();
	
	/**
	 * @return the orientation of the robot (radians, from positive x-axis)
	 */
	public double robotOrientation();
	
	/**
	 * @return the known obstacles on the map
	 */
    public LinkedList<Obstacle> obstacles();
}
