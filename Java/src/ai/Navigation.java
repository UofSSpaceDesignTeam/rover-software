package ai;

import java.awt.geom.Dimension2D;
import java.awt.geom.Point2D;
import java.util.LinkedList;

import environment.MapInterface;
import environment.NavigationInterface;
import environment.Obstacle;

//If anyone sees something that isn't correct in JAVA, or just any kind of 
//advice, don't be afraid to point it out

/* Takes information from map to find ideal path for robot
 * Using straight lines and stopping to turn
 * Sends instructions to robot
 * No localization taken into account yet 
 */

public class Navigation{
	
	
	
	/** Main Method */
	//TODO fill in what main method needs and define its structure
	public static void main(){
		MapInterface map;
		NavigationInterface robot;
		
		
		
		
		
		//Uses interfaces to get values for initial robot position etc.
		Dimension2D mapsize = map.size();
		Point2D startP = map.robotPosition();
		double startO = map.robotOrientation();
		LinkedList<Obstacle> obstacles = map.obstacles();
		
		Point2D positionOfFirstObstacle = obstacles.get(0).getPosition();
		
		//TODO Define goal point based on map dimensions (get to top edge in middle?)
		
		
		//loop that tries to find path to goal from robot
		do {
			boolean obstaclegoal = findCheckPath (Point2D RobotPosition, Point2D Goal, double RobotOrien)
			if (obstaclegoal = true)
				//loop that finds path to node if obstacle in the way
				do {
					Point2D node = findNode ()
					boolean obstaclenode = findCheckPath (Point2D RobotPosition, Point2D node)	
				} while (obstaclenode = true)
		} while(obstaclegoal=true) 
	}

	/** Finds angle and path length, and also checks for obstacles */
	public static boolean findCheckPath (Point2D RobotPosition, Point2D Goal, double RobotOrien) {
		//TODO simple math to get distance between two point and angle to rotate
		map.
		
		boolean obstaclegoal;
		//TODO algorithm to check path for obstacles
		if obstaclegoal = false
			//TODO send path info to queue for Arduino
		return obstaclegoal;
	}
	
	
	/** Finds ideal "node" to move around obstacle by */
	//TODO fill in what findNode needs, does and returns
	public static Point2D findNode (Point2D RobotPosition, Point2D Goal) {
		
	}

}
