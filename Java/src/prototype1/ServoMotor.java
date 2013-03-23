package prototype1;




//The class to control a servo motor on a specified arduino
public class ServoMotor extends Actuator
{
	/**
	 * The number of servo motors being controlled by this class (All the servo motors attached to the Arduino)
	 */
	int numberOfServos;
	
	/**
	 * The constructor for the ServoMotor class
	 * @param newArduino	The arduino that this servo motor is installed on
	 * @param servosToControl	The number of servos that the class will control
	 */
	ServoMotor(Arduino attachedArduino, int servosToControl)
	{	
		super(attachedArduino, MessageProtocol.ID1_SERVOS);
		this.numberOfServos = servosToControl;
	}
	
	/**
	 * Method to set the rotation angle of the servo motor
	 * @precond - angleInDegrees is between 0 and 180
	 * @param angleInDegrees
	 */
	private void setRotationAngle(int angleInDegrees)
	{
		//Fill in the data stream to set the position of this servo motor
		byte setMessage[] = new byte[1];
		
		//TODO error check for invalid rotation angle
		
		setMessage[0] = (byte)angleInDegrees;
		

	}
	
	
}
