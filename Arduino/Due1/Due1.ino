#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>

#define DEBUG_REPLY "#D \o\o"
#define ERROR_REPLY "#E \o\o"

char id[2];

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(200);
  debug("It's alive!");
}

void loop()
{
  while(Serial.available())
  {
    if(Serial.read() == '#')
    {
      if(Serial.readBytes(id,2))
        parseMessage(id);
      else
      {
        Serial.flush();
        debug("malformed message / out of sync!");
      }
    }
  }
}

void parseMessage(char* id)
{
  Serial.flush();
}

void debug(char* message)
{
  Serial.print("#D ");
  Serial.write(0);
  Serial.write(sizeof(message)-1);
  Serial.println(message);
  
  char reply[4];
  Serial.readBytes(reply,4);
  
  //if(!strcmp(reply,DEBUG_REPLY))
     // bad/missing reply...
}

void error(char id, char code)
{
  Serial.print("#E ");
  Serial.write(0);
  Serial.write(4);
  Serial.print(id);
  Serial.println(code);
  
  char reply[4];
  Serial.readBytes(reply,4);
  
  //if(!strcmp(reply,ERROR_REPLY))
     // bad/missing reply...
}
  
