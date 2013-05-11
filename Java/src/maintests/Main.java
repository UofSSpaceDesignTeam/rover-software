package maintests;

import prototype1.ArduinoMessageHandler;
import prototype1.ControlMessageHandler;
import prototype1.Led;
import prototype1.MessageProtocol;
import prototype1.Motors;
import prototype1.ServoMotors;


public class Main {
	public static void main(String[] args) throws InterruptedException
	{
		ArduinoMessageHandler arduino = new ArduinoMessageHandler("/dev/ttyACM0");
		
		ControlMessageHandler control = new ControlMessageHandler();
		
		new Thread(control).start();
		
		Motors m = new Motors(arduino, 2);		
		control.addInstrument(MessageProtocol.ID1_MOTORS, m);
		
		Thread.sleep(100);
		
		
		m.setEnabled(true);
		
		
		
		
		
		/*Led led = new Led(arduino);
		
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
		
		*/
	}
}
