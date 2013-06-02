
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 imu(0x68);

int16_t ax, ay, az, pitch, roll;

void setup()
{
  Wire.begin();
  Serial.begin(115200);

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
}

void loop()
{
  imu.getAcceleration(&ax, &ay, &az);

  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.println(az);
  Serial.print("\t");
  
  pitch = (int)(100*atan2(ay,az));
  roll = (int)(100*atan2(ax,az));
  Serial.print(pitch);
  Serial.print("\t");
  Serial.println(roll);
  delay(100);
}
