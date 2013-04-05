package maintests;

import prototype1.Arduino;
import prototype1.Led;
import prototype1.ServoMotors;


public class Main {
	public static void main(String[] args) throws InterruptedException
	{
		Arduino arduino = new Arduino("/dev/ttyUSB0");
		
		Led led = new Led(arduino);
		
		ServoMotors servo = new ServoMotors(arduino, 1);
		servo.setEnabled(true);
		
		led.turnOff();
		Thread.sleep(200);
		
		int[] angleInDegrees = new int[1];
		angleInDegrees[0] =10;
		
		for (int i = 0; i < 10; i++)
		{
			servo.setRotationAngle(angleInDegrees);
			Thread.sleep(2000);
			angleInDegrees[0] += 10;
		}
		
		
	}
}
