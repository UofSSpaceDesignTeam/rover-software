#include <Wire.h>
#include <I2Cdev.h>
//#include <helper_3dmath.h>
#include <MPU6050.h>
//#include <MPU6050_6Axis_MotionApps20.h>
//#include <MPU6050_9Axis_MotionApps41.h>
#include <Servo.h>

MPU6050 imu(0x68);

int16_t ax, ay, az, pitch, roll, initialPitch, initialRoll;
Servo servo1;
int Position;
int servoPin=6;

void setup()
{
  Wire.begin();
  servo1.attach(servoPin);  
  Serial.begin(9600);

  Serial.println("Initializing...");
  imu.initialize();

  Serial.println("Testing connection...");
  if(imu.testConnection())
  {
    Serial.println("IMU connection successful");
  }
  else
  {
    Serial.println("IMU connection failed");
  }
  delay(50);
  Serial.println("Servo Position:  Roll:");
}

void loop() 
{
 imu.getAcceleration(&ax, &ay, &az);
 roll = (int)(degrees(atan2(ay,az)) - initialRoll);
 if(roll<180)
   MovePosition(90-roll);  
 if(roll>180) 
   MovePosition(90+roll);
 delay(100);    //for testing purposes. Remove after finished debugging.
 Serial.print("             ");
 Serial.println(roll);
}

int MovePosition(int p)
{
  Position=GetPosition();
  if(p!=Position)  
  {
    servo1.write(p);
  }
  Serial.print(Position);  
}

int GetPosition()  
{  
  return  servo1.read();
}
