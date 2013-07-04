package ai;

import prototype1.IMU;
import prototype1.Motors;
import prototype1.Rangefinders;

/**
 * serves as the interface between the AI and the robot's components
 * @author fit-pc
 *
 */
public class Robot implements NavigationInterface {
	/**
	 * the IMU located on the body of the robot
	 */
	private IMU bodyIMU;
	
	private Rangefinders rangefinders;
	
	/**
	 * The motors controlling the wheels
	 */
	private Motors motors; 
	

	
	

	@Override
	public void move(double speed) {
		
	}

	@Override
	public void spin(double speed) {
		// TODO Auto-generated method stub
		
	}


	@Override
	public void curve(double radius, double speed) {
		// TODO Auto-generated method stub
		
	}
	
	
	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub

	}

}
