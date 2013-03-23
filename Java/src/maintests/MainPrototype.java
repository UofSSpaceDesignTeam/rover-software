package maintests;

import prototype1.*;

public class MainPrototype {


	public static void main(String[] args)
	{
		Arduino arduino = new Arduino("/dev/ttyACM0");
		Led led = new Led(arduino);
		
		//Create a ServoMotors class controlling 1 servo
		ServoMotors servo = new ServoMotors(arduino, 1);
		//Create a ForceSensors class controlling 1 (ultrasonic sensor)
		ForceSensors ultrasonic = new ForceSensors(arduino, 1);
		
		for(int i = 0; i< 100; i++)
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
			
			//Set the ServoMotor to the appropriate rotation angle
			if(distanceInCentimeters[0] >= 180)
			{
				//Limit the distance to the max value
				distanceInCentimeters[0] = 180;
			}
			servo.setRotationAngle(distanceInCentimeters);
		}
		
	}

}
