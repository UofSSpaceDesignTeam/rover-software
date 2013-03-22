
// Required external libraries

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>

// Operating parameters

#define NUM_MOTORS 4
#define NUM_ENCODERS 4
#define NUM_IMU 1
#define NUM_FORCE 4

#define DATA_TIMEOUT 200
#define REPLY_TIMEOUT 500


// Global variables

char msgid[2]; // messaging stuff
char len[2];
int msglength;
char msgdata[48];

bool replyPending = false;
char replyid[2];
unsigned long replyTimer;

bool ledBlink = false;


void setup()
{
  while (!Serial); // wait for port to open before starting
  Serial.begin(115200); // primary serial port
  Serial1.begin(115200); // secondary serial port
  Serial.setTimeout(DATA_TIMEOUT);
  Serial1.setTimeout(DATA_TIMEOUT);
  debug(F("It's alive!"));
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

void debug(const String msg)
{
  // todo: properly formatted message sending
}

void reply(const char* id)
{
  Serial1.write('#');
  Serial1.write(id);
  Serial1.write(0);
  Serial1.write(0);
  Serial1.println();
}

void error(const char* id, const String code)
{
  Serial1.write('#');
  Serial1.write(id);
  Serial1.write(0);
  Serial1.write(2);
  Serial.write(code);
  Serial1.println();
}

void parseMessage()
{
  if(!Serial.readBytes(msgid,2))
    debug(F("msg timeout, no ID"));
  if(!Serial.readBytes(len,2))
    {
      debug(F("msg timeout, no DL"));
      if(is_critical(msgid))
        error(msgid,"TO");
    }
    msglength = 256*len[0] + len[1];
    if(!Serial.readBytes(msgdata,msglength))
    {
      debug(F("msg timeout, not enough data"));
      if(is_critical(msgid))
        error(msgid,"TO");
    }
    if(Serial.peek() != '\n')
      debug(F("warn: missing newline after command"));
    
    switch(msgid[0])
    {
      
      case 'P':  // power messages
      {
        if(msgid[1] == 'E') // emergency stop
        {
          // todo: stop all motors and such ASAP
          reply("PE");
          while(true); // make sure nothing else happens until reset
        }
        
        if(msgid[1] == 'O') // power off
        {
          // todo: clean up stuff before shutdown
          reply("PO")
          // go to sleep mode until reset
        }
        
        debug(F("unknown message, ID1 = 'P'")); // if we get here something is wrong
        error(msgid,"UM"); // power messages are all critical  
        return;
      }
      
      
      case 'D':  // data link messages
      {
        if(msgid[1] == 'C') // link check
        {
          reply("DC");
          return;
        }
        
        debug(F("unknown message, ID1 = 'D'")); // if we get here something is wrong
        error(msgid,"UM"); // data link messages are all critical
        return;
      }
      
      
      case 'L':  // LED messages
      {
        if(msgid[1] == 'S') // LED set
        {
          ledBlink = false;
          if(msgdata[0] == '1')
          {
            digitalWrite(13,HIGH);
          }
          if(msgdata[0] == '0')
          {
            digitalWrite(13,LOW);
          }
          if(msgdata[0] == 'b')
          {
            ledBlink = true;
          } 
          return;
        }
        
        debug(F("unknown message, ID1 = 'L'")); // if we get here something is wrong
        return;
      }
      
      
      case 'M':  // motor messages
      {
        if(msgid[1] == 'E') // motor enable
        {
          // todo: activate / deactivate motor controllers
          reply("ME");
          return;
        }
        
        if(msgid[1] == 'S') // motor set
        {
          // todo: set the speeds of all the motors
          reply("MS");
          return;
        }
        
        debug(F("unknown message, ID1 = 'M'")); // if we get here something is wrong
        error(msgid,"UM"); // motor messages are all critical
        return;
      }
      
      
      case 'S':  // motor messages
      {
        if(msgid[1] == 'E') // servo enable
        {
          // todo: activate / deactivate servo controllers (?)
          reply("SE");
          return;
        }
        
        if(msgid[1] == 'S') // servo set
        {
          // todo: set the positions of all the servos
          reply("SS");
          return;
        }
        
        debug(F("unknown message, ID1 = 'S'")); // if we get here something is wrong
        error(msgid,"UM"); // servo messages are all critical
        return;
      }
      
      
      case 'I':  // IMU messages
      {
        if(msgid[1] == 'E') // IMU enable
        {
          // todo: begin / end grabbing data from a specific IMU
          reply("IE");
          return;
        }
        
        if(msgid[1] == 'R') // IMU send rate
        {
          // todo: set timer for sending IMU data messages
          return;
        }
        
        debug(F("unknown message, ID1 = 'I'")); // if we get here something is wrong
        return;
      }
      
      
      case 'W':  // encoder messages
      {
        if(msgid[1] == 'E') // encoder enable
        {
          // todo: begin / end grabbing data from all encoders
          return;
        }
        
        if(msgid[1] == 'R') // encoder send rate
        {
          // todo: set timer for sending encoder data messages
          return;
        }
        
        debug(F("unknown message, ID1 = 'W'")); // if we get here something is wrong
        return;
      }


      case 'F':  // force sensor messages
      {
        if(msgid[1] == 'E') // force sensor enable
        {
          // todo: begin / end grabbing data from all force sensors
          return;
        }
        
        if(msgid[1] == 'R') // force sensor send rate
        {
          // todo: set timer for sending force sensor data messages
          return;
        }
        
        debug(F("unknown message, ID1 = 'F'")); // if we get here something is wrong
        return;
      }
    }
}
        

      
        
    
      
  
     
  
