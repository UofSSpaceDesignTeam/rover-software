package maintests;

import java.util.Observable;
import java.util.Observer;

import prototype1.ArduinoMessageHandler;
import prototype1.IMU;
import prototype1.MessageProtocol;

public class MotorTest implements Observer {

	/**
	 * @param args
	 */
	
	public static IMU imu; 
	public static void main(String[] args) 
	{
		ArduinoMessageHandler arduino = new ArduinoMessageHandler("/dev/ttyACM0");
		
		imu = new IMU(1, arduino);
		
		MotorTest m = new MotorTest();
		imu.addObserver(m);
		
		arduino.addSensor(MessageProtocol.ID1_IMU, imu);
		
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		imu.setEnabled(true);
		
		imu.setDataTransferRate(1);
		
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		
		try {
			Thread.sleep(10000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		
		/*
		Motors motors = new Motors(arduino, 2);	
		
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		
		motors.setEnabled(true);
		
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		for (int i = 0; i < 10; i++)
		{
			System.out.println("Enter F for forward, R for reverse, S for Stop: ");
			Scanner in = new Scanner(System.in);
			String next = in.next();
			if (next.equals("F"))
			{
				motors.goForward();
			} else if (next.equals("R"))
			{
				motors.goReverse();
			} else if (next.equals("S"))
			{
				motors.stop();
			}
		} */
	}

	@Override
	public void update(Observable arg0, Object arg1) {
		System.out.println("pitch " + imu.getLastPitchValues()[0] + " roll " + imu.getLastRollValues()[0]);	
		
	}

}
