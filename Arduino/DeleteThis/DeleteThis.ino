//wiring connections: IMU: vdd->3.3v, gnd->gnd, vio->3.3v, sda->sda(pin 20), 
//scl->scl(pin 21)  
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 imu(0x68);
int16_t ax, ay, az, pitch, roll, initialPitch, initialRoll;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;

Servo servo[] = {
  servo1, servo2, servo3, servo4, servo5, servo6};


void setup()
{
  imu.initialize();
  for(int i=0; i<6; i++)
  {
    servo[i].attach(2+i);
  }
}

void loop() 
{
 if(Serial.available())
   { 
     Serial.read();
     imu.getAcceleration(&ax, &ay, &az);
     roll = (int)(degrees(atan2(ay,az)) - initialRoll);
     pitch = (int)(degrees(atan2(ax,az)) - initialPitch); 
     level(roll, pitch);
   }
}


