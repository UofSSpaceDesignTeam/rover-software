
// Required external libraries

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>

// Operating parameters

#define NUM_MOTORS 4;
#define NUM_ENCODERS 4;
#define NUM_IMU 1;
#define NUM_FORCE 4;

#define DATA_TIMEOUT 200;
#define REPLY_TIMEOUT 500;


// Global variables

char msgid[2]; // messaging stuff
char len[2];
int msglength;
char msgdata[48];

bool replyPending = false;
char replyid[2];
unsigned long replyTimer;


while (!Serial); // wait for port to open before starting

void setup()
{
  Serial.begin(115200); // primary serial port
  Serial1.begin(115200); // secondary serial port
  Serial.setTimeout(DATA_TIMEOUT);
  Serial1.setTimeout(DATA_TIMEOUT);
  debug("It's alive!");
}

void loop()
{
  
  while(Serial.available()) // check for new messages
  {
    if(Serial.read() == '#')
      parseMessage();
  }
  
  if(replyPending) // check for a reply
  {
    while(Serial1.available())
    {
      if(Serial.read() == '#')
        parseReply();
    }
  }
  
  // basic sensor monitoring stuff goes here.
  // this part should never exceed 50ms or so!
  
}

void parseMessage()
{
  if(!Serial.readBytes(msgid,2))
    debug("msg timeout, no ID");
  if(!Serial.readBytes(len,2))
    reply(
  
     
  
