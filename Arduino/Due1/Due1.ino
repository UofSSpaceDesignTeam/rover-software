
// Required external libraries

#include "Wire.h"
// #include "I2Cdev.h"
// #include "MPU6050_6Axis_MotionApps20.h"
#include "Encoder.h"
#include "Servo.h"

// Operating parameters

#define NUM_MOTORS 4
#define NUM_ENCODERS 4
#define NUM_IMU 1
#define NUM_FORCE 2

#define MSG_TIMEOUT 100
#define REPLY_TIMEOUT 500

#define LED_BLINKRATE 300

// Global variables
 
char msgid[2]; // messaging stuff
char len[2];
int msglength;
char msgdata[48];
bool replyPending = false;
char replyid[2];
unsigned long replyTimer;
String debugmsg = "";

boolean imu_enable = false;  // sensor reporting parameters
unsigned int imu_sendRate = 500;
unsigned long imu_sendTimer;

boolean encoder_enable = false;
unsigned int encoder_sendRate = 500;
unsigned long encoder_sendTimer;

boolean force_enable = false;
unsigned int force_sendRate = 500;
unsigned long force_sendTimer;


boolean ledBlink = false;  // internal state data
boolean ledState = false;
unsigned long ledTimer;


// todo: put stored sensor data variables here


void setup()
{
  while (!Serial); // wait for main serial port to open before starting
  Serial.begin(115200); // primary serial port
  Serial1.begin(115200); // secondary serial port
  Serial.setTimeout(MSG_TIMEOUT);
  Serial1.setTimeout(MSG_TIMEOUT);
  
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
  
  delay(100);
  debugmsg = "Init sensors...";
  debug();
  
  // todo: put sensor init stuff here
  // debug something on failure
  
  imu_sendTimer = millis();
  encoder_sendTimer = millis();
  force_sendTimer = millis();
  ledTimer = millis();
  
  debugmsg = "Init done";
  debug();
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
  
  if(ledBlink && millis() - ledTimer >= LED_BLINKRATE) // if it's time to blink the led
  {
    ledState = !ledState;  // switch the led on/off
    if(ledState)
      digitalWrite(13,HIGH);
    else
      digitalWrite(13,LOW);
    ledTimer = millis();
  }
  
  if(encoder_enable && millis() - encoder_sendTimer >= encoder_sendRate)
  {
    // todo: send the encoder data
    encoder_sendTimer = millis();
  }
  
  if(imu_enable && millis() - imu_sendTimer >= imu_sendRate)
  {
    // todo: send the imu data
    imu_sendTimer = millis();
  }
  
  if(force_enable && millis() - force_sendTimer >= force_sendRate)
  {
    // todo: send the force data
    force_sendTimer = millis();
  }
  
  // general monitoring stuff goes here
  // try not to take too long
  
}


void debug() // up to 255 chars
{
  int length = debugmsg.length();
  Serial.write('#');
  Serial.write("DB");
  Serial.write((byte)length/256);
  Serial.write((byte)length%256);
  Serial.println(debugmsg);
  debugmsg = "";
}


void reply(const char* id)
{
  Serial1.write('#');
  Serial1.write(id);
  Serial1.write((byte)0);
  Serial1.write((byte)0);
  Serial1.println();
}


void error(const char* id, const char* code)
{
  Serial1.write('#');
  Serial1.write(id);
  Serial1.write((byte)0);
  Serial1.write((byte)2);
  Serial1.write(code);
  Serial1.println();
}


boolean is_critical(char* id)
{
  if(id[0] == 'D' || id[0] == 'P' || id[0] == 'M' || id[0] == 'S')
    return true;
  return false;
}


void parseMessage()
{
  if(!Serial.readBytes(msgid,2))
    debugmsg = "msg timeout, no ID";
    debug();
  if(!Serial.readBytes(len,2))
    {
      debugmsg = "msg timeout, no DL";
      debug();
      if(is_critical(msgid))
        error(msgid,"TO");
    }
    msglength = 256*len[0] + len[1];
    if(!Serial.readBytes(msgdata,msglength))
    {
      debugmsg = "msg timeout, not enough data";
      debug();
      if(is_critical(msgid))
        error(msgid,"TO");
    }
    if(Serial.peek() != '\n')
      debugmsg = "warn: missing newline after command";
      debug();
    
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
          reply("PO");
          // go to sleep mode until reset
        }
        
        debugmsg = "unknown message, ID1 = 'P'"; // if we get here something is wrong
        debug();
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
        
        debugmsg = "unknown message, ID1 = 'D'"; // if we get here something is wrong
        debug();
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
        
        debugmsg = "unknown message, ID1 = 'L'"; // if we get here something is wrong
        debug();
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
        
        debugmsg = "unknown message, ID1 = 'M'"; // if we get here something is wrong
        debug();
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
        
        debugmsg = "unknown message, ID1 = 'S'"; // if we get here something is wrong
        debug();
        error(msgid,"UM"); // servo messages are all critical
        return;
      }
      
      
      case 'I':  // IMU messages
      {
        if(msgid[1] == 'E') // IMU enable
        {
          // todo: begin / end grabbing data from a specific IMU
          return;
        }
        
        if(msgid[1] == 'R') // IMU send rate
        {
          // todo: set timer for sending IMU data messages
          return;
        }
        
        debugmsg = "unknown message, ID1 = 'I'"; // if we get here something is wrong
        debug();
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
        
        debugmsg = "unknown message, ID1 = 'W'"; // if we get here something is wrong
        debug();
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
        
        debugmsg = "unknown message, ID1 = 'F'"; // if we get here something is wrong
        debug();
        return;
      }
    }
}


void parseReply()
{
  return;
}


      
        
    
      
  
     
  
