//wiring connections: IMU: vdd->3.3v, gnd->gnd, vio->3.3v, sda->sda(pin 20), 
//scl->scl(pin 21) SERVO1: red->5v, gnd->gnd, yellow->digital pin 6  SERVO2: red->5v, gnd->gnd, yellow->digital pin 7
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 imu(0x68);
int16_t ax, ay, az, pitch, roll, initialPitch, initialRoll;
Servo servo1;
Servo servo2;
int Position1;
int Position2;

void setup()
{
  Wire.begin();
  servo1.attach(6); 
  servo2.attach(7); 
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
  Serial.println("Roll:  Servo 1 Pos: Servo 2 Pos: ");
}

void loop() 
{
 if(Serial.available())
   { 
     Serial.read();
     imu.getAcceleration(&ax, &ay, &az);
     roll = (int)(degrees(atan2(ay,az)) - initialRoll);
     if(roll<180)
       MovePosition(90-roll, 90+roll);  
     if(roll>180) 
       MovePosition(90+roll, 90-roll);
     delay(100);    //for testing purposes. Remove after finished debugging.
     Serial.print(roll);
     Serial.print("      ");
     
   }
}

int MovePosition(int p1, int p2)
{
  Position1=servo1.read();
  Position2=servo2.read();
  if(p1!=Position1)  
  {
    servo1.write(p1);
  }
  if(p2!=Position2)
  {
    servo2.write(p2);
  }
  Serial.print(Position1); 
  Serial.print("      ");
  Serial.println(Position2); 
}

