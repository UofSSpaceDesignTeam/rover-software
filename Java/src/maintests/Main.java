package maintests;

import prototype1.Arduino;
import prototype1.Led;


public class Main {
	public static void main(String[] args) throws InterruptedException
	{
		Arduino arduino = new Arduino("/dev/ttyACM0");
		
		Led led = new Led(arduino);
		
		led.turnOff();
		Thread.sleep(200);
		
		for (int i = 0; i < 10; i++)
		{
			led.turnOn();
			Thread.sleep(1000);
			led.turnOff();
			Thread.sleep(1000);
		}
		
		
	}
}
