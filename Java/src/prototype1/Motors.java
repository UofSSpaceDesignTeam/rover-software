package prototype1;


public class Motors extends Actuator {
	// speed constants TODO add javadoc
	public static final int SPEED_FULL_REVERSE = 0;
	public static final int SPEED_STOP = 127;
	public static final int SPEED_FULL_FORWARD = 255;	
	
	/**
	 * The number of motors being controlled by this class
	 */
	private int numMotors;
	
	/**
	 * The constructor for the Motors class
	 * @param newArduino  The arduino that this motor is installed on
	 * @param  motorsToControl	The number of motor that the class will control
	 */
	public Motors(Arduino attachedArduino, int motorsToControl)
	{	
		super(attachedArduino, MessageProtocol.ID1_MOTORS);
		this.numMotors =  motorsToControl;
	}
	
	/**
	 * Method to set the speed of each motor
	 * @precond - speed is an array of integers between 0 and 255,
	 *            one integer for each servo
	 * ( 0 is full reverse, 127 is full stop, 255 is full forward)
	 */
	public void setRotationSpeed(int speed[])
	{
		//TODO error check to make sure speed.length == this.numMotors
		
		//Fill in the data stream to set the position of this servo motor
		byte setMessage[] = new byte[speed.length];
		
		//TODO error check for invalid speed

		for (int i = 0; i < this.numMotors; i++)
		{
			setMessage[i] = (byte) speed[i];
		}
		super.set(setMessage);
	}
	
	// convenience methods TODO add javadoc
	public void goForward()
	{
		int[] speed = new int[numMotors];
		for (int i = 0; i < numMotors; i++)
		{
			speed[i] = SPEED_FULL_FORWARD;
		}
		setRotationSpeed(speed);
	}
	public void goReverse()
	{
		int[] speed = new int[numMotors];
		for (int i = 0; i < numMotors; i++)
		{
			speed[i] = SPEED_FULL_REVERSE;
		}
		setRotationSpeed(speed);
	}
	public void stop()
	{
		int[] speed = new int[numMotors];
		for (int i = 0; i < numMotors; i++)
		{
			speed[i] = SPEED_STOP;
		}
		setRotationSpeed(speed);
	}
}
