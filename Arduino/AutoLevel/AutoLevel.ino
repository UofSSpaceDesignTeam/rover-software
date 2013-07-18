//wiring connections: IMU: vdd->3.3v, gnd->gnd, vio->3.3v, sda->sda(pin 20), scl->scl(pin 21) SERVO: red->5v, gnd->gnd, yellow->digital pin 6
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 imu(0x68);
int16_t ax, ay, az, pitch, roll, initialPitch, initialRoll;
Servo servo1;
int Position;

void setup()
{
  Wire.begin();
  servo1.attach(6);  
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
 if(Serial.available())
   { 
     Serial.read();
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
}

int MovePosition(int p)
{
  Position=servo1.read();
  if(p!=Position)  
  {
    servo1.write(p);
  }
  Serial.print(Position);  
}

