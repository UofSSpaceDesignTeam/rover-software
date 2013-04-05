package maintests;

import prototype1.*;

public class MainPrototype {


	public static void main(String[] args) throws InterruptedException
	{
		Arduino arduino = new Arduino("/dev/ttyUSB0");
		Led led = new Led(arduino);
		
		//Create a ServoMotors class controlling 1 servo
		ServoMotors servo = new ServoMotors(arduino, 1);
		servo.setEnabled(true);
		//Create a ForceSensors class controlling 1 (ultrasonic sensor)
		ForceSensors ultrasonic = new ForceSensors(arduino, 1);
		ultrasonic.setEnabled(true);
		
		// NOTE
		arduino.addSensor(MessageProtocol.ID1_FORCE, ultrasonic);
		
		led.startBlinking();
		Thread.sleep(1000);		
		for(int i = 0; i< 1000; i++)
		{
			//We know there is only 1 Force Sensor (ultrasonic), so we will get the integer array of distances
			int distanceInCentimeters[] = new int[1];
			
			//Make sure the returned force values are in the form of a 1-int array
			if (ultrasonic.getLastForceValues().length != 1)
			{
				//Error; we should only be getting 1 integers back
				System.out.println("Ultrasonic Error: Expected a 1-int array but reveived a " 
				+ ultrasonic.getLastForceValues().length + "-int array.");
			}
			
			distanceInCentimeters = ultrasonic.getLastForceValues();
			//distanceInCentimeters[0] *= 10;
			
			System.out.println(distanceInCentimeters[0]);
			
			//Set the ServoMotor to the appropriate rotation angle
			if(distanceInCentimeters[0] >= 180)
			{
				//Limit the distance to the max value
				distanceInCentimeters[0] = 180;
			}
			servo.setRotationAngle(distanceInCentimeters);
			Thread.sleep(100);
		}
		
		//Disable the devices
		servo.setEnabled(false);
		ultrasonic.setEnabled(false);
		led.turnOff();
	}

}
