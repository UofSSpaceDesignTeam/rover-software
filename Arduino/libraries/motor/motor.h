// custom class for motor control emulating the servo library.

#ifndef motor_h
#define motor_h

class motor
{
	public:
	
	motor(byte dirPin, byte pwmPin);
	void enable();
	void disable();
	void set(byte setting);
	
	private:
	
	byte directionPin;
	byte speedPin;
	boolean enabled;
}