
// Main sketch for the Arduino DUE on the USST Lunabotics Rover. Written in Arduino 1.5.2


// Required external libraries

#include "Wire.h"  // i2c library for imu
#include "imuhack.h"  // imu functions (in development)
#include "Encoder.h"  //  used to read wheel encoders
#include "Servo.h"  //  used to control servomotors

// Hardware parameters, change as hardware evolves

#define NUM_MOTORS 2
#define NUM_SERVOS 0
#define NUM_ENCODERS 0
#define NUM_FORCE 0

// Pin connections, change as new parts are added

#define L_MOTOR_DIR 46
#define L_MOTOR_PWM 3
#define R_MOTOR_DIR 47
#define R_MOTOR_PWM 4
#define FIRST_SERVO_PIN 22 // servo signal output pins are from 22 to 21 + NUM_SERVOS

// Default parameters for operation

#define COMM_TIMEOUT 3000 // wait 3 seconds with no communication before stopping
#define MSG_TIMEOUT 100 // 100ms since last byte until we give up on a message
#define LED_BLINKRATE 300 // toggle LED every 300ms
#define SERVO_DEFAULT 90 // set all servos to center at start

// Global variables

boolean commTimeOut = false; // safety in case of fitpc issues
unsigned long commTimer; // timeout counter
char msgid[2]; // ID bytes of incoming messages
char len[2];  // length bytes of incoming messages
int msglength;  // length of incoming message
char msgdata[48];  // data bytes of incoming messages
String debugmsg = ""; // holds messages we want to send

boolean imuEnable = false;  // IMU state information
unsigned int imuSendRate = 500; // how often we send IMU data
unsigned long imuSendTimer; // keeps track of when we send data

boolean motorEnable = false; // motor controller state information
byte motorSpeeds[NUM_MOTORS]; // for storing values in case of timeout

boolean servoEnable = false; // servo controller state information
Servo servoArray[NUM_SERVOS]; // create array of servo objects
byte servoPositions[NUM_SERVOS]; // create array of stored servo positions
byte servoPins[NUM_SERVOS]; // creates array of pins that servos are attached to

boolean encoderEnable = false; // wheel encoder state information
unsigned int encoderSendRate = 500;
unsigned long encoderSendTimer;

boolean forceEnable = false; // force sensor state information
unsigned int forceSendRate = 500;
unsigned long forceSendTimer;

boolean ledBlink = false;  // led state information
boolean ledState = false;
unsigned long ledTimer;


void setup()  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
{
  while (!Serial); // wait for main serial port to open before starting
  Serial.begin(115200); // start the USB Serial port
  Serial.setTimeout(MSG_TIMEOUT);
  
  pinMode(13,OUTPUT); // set LED off to start
  digitalWrite(13,LOW);
  
  delay(100);
  debugmsg = "Init:"; // first message on boot
  debug();
  
  delay(10);
  debugmsg = "motors"; // set up the motor controller initial state (everything stopped)
  debug();
  analogWrite(L_MOTOR_PWM,0);
  analogWrite(R_MOTOR_PWM,0);
  pinMode(L_MOTOR_DIR,OUTPUT);
  digitalWrite(L_MOTOR_DIR,LOW);
  pinMode(R_MOTOR_DIR,OUTPUT);
  digitalWrite(R_MOTOR_DIR,LOW);
  for(int i=0; i<NUM_MOTORS; i++)
  {
    motorSpeeds[i] = 0;
  }
  
  delay(10);
  debugmsg = "servos"; // set up all the servomotors' initial state, but don't activate them yet
  debug();
  for(int i=0; i<NUM_SERVOS; i++)
  {
    servoPositions[i] = SERVO_DEFAULT;
    servoPins[i] = FIRST_SERVO_PIN + i;
  }
  
  delay(10);
  debugmsg = "encoders"; // set up the wheel encoders
  debug();
  //stuff
  
  delay(10);
  debugmsg = "force sensors"; // set up the force sensors
  debug();
  //stuff
  
  delay(10);
  debugmsg = "IMU"; // set up the imu
  debug();
  //stuff
  
  delay(10);
  debugmsg = "timers";
  debug();
  imuSendTimer = millis();
  forceSendTimer = millis();
  encoderSendTimer = millis();
  ledTimer = millis(); // for LED blinker
  commTimer = millis();  // for communication timeout
  
  delay(50);
  debugmsg = "done!";
  debug();
}


void loop()  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
{
  while(Serial.available()) // check for new messages until the buffer is empty
  {
    if(Serial.read() == '#') // if we find the message start character
    {
      if(parseMessage() != -2) // if we process the message with no receive errors
      {
        commTimer = millis(); // reset the timeout counter
        if(commTimeOut) // if we have timed out and are now running again
        {
          commTimeOut = false;
          if(motorEnable)
          {
            analogWrite(L_MOTOR_PWM,motorSpeeds[0]);
            analogWrite(R_MOTOR_PWM,motorSpeeds[1]);
          }
        }
      }
    } 
  }
  
  if(millis() - commTimer > COMM_TIMEOUT) // No sign of life from fitpc, stop and wait for a valid message
  {
    analogWrite(L_MOTOR_PWM,0);
    analogWrite(R_MOTOR_PWM,0);
    commTimeOut = true;
  }
  
  if(ledBlink && millis() - ledTimer >= LED_BLINKRATE) // if it's time to blink the led
  {
    ledState = !ledState;  // switch the led on/off
    if(ledState)
      digitalWrite(13,HIGH);
    else
      digitalWrite(13,LOW);
    ledTimer = millis(); // reset the timer
  }
  
  if(encoderEnable && millis() - encoderSendTimer >= encoderSendRate) // if it's time to send encoder data
  {
    // todo: send the encoder data
    encoderSendTimer = millis();
  }
  
  if(imuEnable && millis() - imuSendTimer >= imuSendRate) // if it's time to send imu data
  {
    // todo: send the imu data
    imuSendTimer = millis();
  }
  
  if(forceEnable && millis() - forceSendTimer >= forceSendRate) // if it's time to send force data
  {
    // todo: send the force data
    forceSendTimer = millis();
  }
  
  // other code that needs to be run periodically/constantly goes here, but make sure it's always fast!
  // consider using a timer as above
  
}


// don't leave things lying around down here
// make a new tab instead

      
        
    
      
  
     
  
