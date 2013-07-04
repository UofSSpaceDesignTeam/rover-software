package prototype1;

import ai.Situation;


public class Motors extends Actuator {
	// speed constants TODO add javadoc
	public static final int SPEED_FULL_REVERSE = 0;
	public static final int SPEED_STOP = 127;
	public static final int SPEED_FULL_FORWARD = 255;	
		
	/**
	 * The constructor for the Motors class
	 * @param newArduino  The arduino that this motor is installed on
	 * @param  motorsToControl	The number of motor that the class will control
	 */
	public Motors(ArduinoMessageHandler attachedArduino, Situation[] s,int motorsToControl)
	{	
		super(attachedArduino, MessageProtocol.ID1_MOTORS, s, motorsToControl);
	}
	
	/**
	 * Method to set the speed of each motor
	 * @precond - speed is an array of integers between 0 and 255,
	 *            one integer for each servo
	 * ( 0 is full reverse, 127 is full stop, 255 is full forward)
	 */
	public void setRotationSpeed(byte speed[])
	{
		//TODO error check to make sure speed.length == this.numMotors
		
		//Fill in the data stream to set the position of this servo motor
		
		
		//TODO error check for invalid speed

		super.set(speed);
	}
	
	// convenience methods TODO add javadoc
	public void goForward()
	{
		byte[] speed = new byte[this.size()];
		for (int i = 0; i < this.size(); i++)
		{
			speed[i] = (byte) SPEED_FULL_FORWARD;
		}
		setRotationSpeed(speed);
	}
	
	public void goReverse()
	{
		byte[] speed = new byte[this.size()];
		for (int i = 0; i < this.size(); i++)
		{
			speed[i] = (byte) SPEED_FULL_REVERSE;
		}
		setRotationSpeed(speed);
	}
	public void stop()
	{
		byte[] speed = new byte[this.size()];
		for (int i = 0; i < this.size(); i++)
		{
			speed[i] = (byte) SPEED_STOP;
		}
		setRotationSpeed(speed);
	}
}
