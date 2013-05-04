package maintests;

import java.util.Scanner;

import prototype1.Arduino;
import prototype1.ForceSensors;
import prototype1.Led;
import prototype1.MessageProtocol;
import prototype1.Motors;
import prototype1.ServoMotors;

public class MotorTest {

	/**
	 * @param args
	 */
	public static void main(String[] args) 
	{
		Arduino arduino = new Arduino("/dev/ttyUSB0");
		
		Motors motors = new Motors(arduino, 2);		
		motors.setEnabled(true);
		
		for (int i = 0; i < 10; i++)
		{
			System.out.println("Enter F for forward, R for reverse, S for Stop: ");
			Scanner in = new Scanner(System.in);
			String next = in.next();
			if (next.equals("F"))
			{
				motors.goForward();
			} else if (next.equals("R"))
			{
				motors.goReverse();
			} else if (next.equals("S"))
			{
				motors.stop();
			}
		}
	}

}
