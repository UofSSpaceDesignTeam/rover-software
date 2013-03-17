#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>

#define DEBUG_REPLY "#D \o\o"
#define ERROR_REPLY "#E \o\o"

char id[2];
char len[2];
int msglength;
char msgdata[48];

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
    }
  }
}

void parseMessage()
{
  if(Serial.readBytes(len,2))
  {
    msglength = len[0] + len[1]*256;
    if(msglength > sizeof(msgdata))
    {
      error(id,"ML"); // msg too long
      return;
    }
    if(Serial.readBytes(msgdata,msglength) < msglength)
    {
      error(id,"MI"); // incomplete message
      return;
    }
    
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

void error(char* id, char* code)
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
  
