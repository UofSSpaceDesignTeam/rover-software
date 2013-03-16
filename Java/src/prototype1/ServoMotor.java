package prototype1;


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
		char[] enableMessage = new char[7];
		
		enableMessage[0] = '#';
		enableMessage[1] = 'S';
		enableMessage[2] = 'E';
		enableMessage[3] = (char)1/256;
		enableMessage[4] = (char)(1%256);
		enableMessage[5] = (char)(1%256);
		enableMessage[6] = '\n';
		
		associatedArduino.sendMessage()
	}
	
	
	private void setRotationAngle(int angleInDegrees)
	{
		
		associatedArduino.sendMessage();
	}
	
	
}
