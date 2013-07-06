package environment;

import instruments.IMU;
import instruments.Motors;
import instruments.Rangefinders;

import java.util.Observable;
import java.util.Observer;

import messaging.ArduinoMessageHandler;
import messaging.ControlMessageHandler;
import messaging.MessageProtocol;


/**
 * serves as the interface between the AI and the robot's components
 * @author fit-pc
 *
 */
public class Robot implements Observer, NavigationInterface {
	
	/**
	 * the first arduino
	 */
	private ArduinoMessageHandler arduino;
	/**
	 * the IMU located on the body of the robot
	 */
	private IMU bodyIMU;
	
	private Rangefinders rangefinders;
	
	private static final int NUM_RANGE_FINDERS = 6;
	private static final int NUM_MOTORS = 2;
	
	/**
	 * The motors controlling the wheels
	 */
	private Motors motors; 
	
	public Robot(){
		arduino = new ArduinoMessageHandler("/dev/ttyACM1");
		bodyIMU = new IMU(arduino, new Situation());		
		arduino.addSensor(MessageProtocol.ID1_IMU, bodyIMU);
		bodyIMU.addObserver(this);
		
		Situation[] rangeSits = new Situation[NUM_RANGE_FINDERS];
		// set the locations of the rangefinders here
		rangefinders = new Rangefinders(arduino, rangeSits, NUM_RANGE_FINDERS);
		
		motors = new Motors(arduino, new Situation[NUM_MOTORS], NUM_MOTORS);
	}
	
	
	

	
	

	@Override
	public void move(double speed) {
		// convert speed to a byte, then call setRotation
	}

	@Override
	public void spin(double speed) {
		// TODO Auto-generated method stub
		
	}


	@Override
	public void curve(double radius, double speed) {
		// TODO Auto-generated method stub
		
	}
	
	


	@Override
	public void moveFullSpeed(Direction dir)
	{
		switch (dir){
			case FORWARD: {motors.goForward(); break;}
			case BACKWARD: {motors.goReverse(); break;}
		}
	}
	
	public static void main(String[] args) throws InterruptedException{
		Robot r = new Robot();
		Thread.sleep(1000);
		r.bodyIMU.setEnabled(true);
		Thread.sleep(1000);
		
		ControlMessageHandler control = new ControlMessageHandler();		
		new Thread(control).start();			
		control.addInstrument(MessageProtocol.ID1_MOTORS, r.motors);
	}
	
	@Override
	public void update(Observable arg0, Object arg1) {
		System.out.println("pitch " + bodyIMU.getLastPitchValues()[0] + " roll " + bodyIMU.getLastRollValues()[0]);	
	}
	
	

}
