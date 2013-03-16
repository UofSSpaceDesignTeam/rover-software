package prototype1;

import arduino.Arduino;



//The class to control a servo motor on a specified arduino
public class ServoMotor 
{
	/**
	 * Fields
	 */
	
	//The field to store this object's unique Identifier for communications to the Arduino
//	char uniqueIdentifier;
	
	//The field to store the Arduino that this motor is attached to 
	Arduino associatedArduino = new Arduino();
	
	
	/**
	 * Methods
	 */
	
	/**
	 * The constructor for the ServoMotor class
	 * @param newArduino - The arduino that this servo motor is installed on
//	 * @param identifier - The identifier associated to this servo motor
	 */
	ServoMotor(Arduino newArduino, char[] identifier)
	{
		//Set the identifier for this object
//		this.uniqueIdentifier = identifier;
		
		//Set the Arduino that this object will communicate with
		this.associatedArduino = newArduino;
		
		//Enable this servo motor
		byte[] enableMessage = new byte[7];
		//Fill in the data stream to enable this servo motor
		// #[S][E][0][1][1][\n]
		enableMessage[0] = '#';
		enableMessage[1] = 'S';
		enableMessage[2] = 'E';
		enableMessage[3] = 0;
		enableMessage[4] = 1;
		enableMessage[5] = 1;
		enableMessage[6] = '\n';
		
		associatedArduino.sendMessage(enableMessage);
	}
	
	/**
	 * Method to set the rotation angle of the servo motor
	 * @precond - angleInDegrees is between 0 and 180
	 * @param angleInDegrees
	 */
	private void setRotationAngle(int angleInDegrees)
	{
		//Fill in the data stream to set the position of this servo motor
		byte enableMessage[] = new byte[7];
		// #[S][P][0][1][angleInDegrees][\n]
		enableMessage[0] = '#';
		enableMessage[1] = 'S';
		enableMessage[2] = 'P';
		enableMessage[3] = 0;
		enableMessage[4] = 1;
		enableMessage[5] = (byte)angleInDegrees;
		enableMessage[6] = '\n';
		associatedArduino.sendMessage(setAngleMessage);
	}
	
	
}
