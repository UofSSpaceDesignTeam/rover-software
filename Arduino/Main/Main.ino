
// Main sketch for the Arduino DUE on the USST Lunabotics Rover. Written in Arduino 1.5.2


// Required external libraries

#include "Wire.h"  // these 3 are required for IMU
// #include "I2Cdev.h"  // but this one doesn't work on the DUE, we'll need a workaround eventually
#include "Encoder.h"  //  will be used to read wheel encoders
#include "Servo.h"  //  used to control e.g. camera pan

// Hardware parameters, change as hardware evolves

#define NUM_MOTORS 2
#define NUM_SERVOS 0
#define NUM_ENCODERS 0
#define NUM_IMU 1
#define NUM_FORCE 0

// Pin connections, change as new parts are added

#define L_MOTOR_DIR 46
#define L_MOTOR_PWM 3
#define R_MOTOR_DIR 47
#define R_MOTOR_PWM 4
#define FIRST_SERVO_PIN 22 // servo signal outputs are from 22 to 22 + NUM_SERVOS - 1

// Default parameters for operation

#define COMM_TIMEOUT 3000 // wait 3 seconds with no communication before going to error mode
#define MSG_TIMEOUT 100 // 100ms since last byte until we declare an error
#define REPLY_TIMEOUT 500 // wait 500ms for reply before going to error mode
#define LED_BLINKRATE 300 // toggle LED every 300ms
#define SERVO_DEFAULT 90 // set all servos to center at start

// Global variables

boolean errorMode = false; // whether we are in error mode
char msgid[2]; // ID bytes of incoming messages
char len[2];  // length bytes of incoming messages
int msglength;  // length of incoming message
char msgdata[48];  // data bytes of incoming messages
bool replyPending = false; // whether we are waiting for a reply
char replyid[2];  // ID bytes of reply message
unsigned long replyTimer; // How long we have waited for reply
String debugmsg = ""; // holds messages we want to send

boolean imuEnable = false;  // IMU state information
unsigned int imuSendRate = 500; // how often we send IMU data
unsigned long imuSendTimer; // keeps track of when we send data

boolean motorEnable = false; // motor controller state information

boolean servoEnable = false; // servo controller state information

boolean encoderEnable = false; // wheel encoder state information
unsigned int encoderSendRate = 500;
unsigned long encoderSendTimer;

boolean forceEnable = false; // force sensor state information
unsigned int forceSendRate = 500;
unsigned long forceSendTimer;


boolean ledBlink = false;  // led state information
boolean ledState = false;
unsigned long ledTimer;

// Creation of global objects

Servo servoArray[NUM_SERVOS]; // create array of servo objects
char servoPositions[NUM_SERVOS]; // create array of stored servo positions
char servoPins[NUM_SERVOS]; // creates array of pins that servos are attached to

// todo: encoders, IMU


void setup()  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
{
  while (!Serial); // wait for main serial port to open before starting
  Serial.begin(115200); // start the primary serial port
  Serial1.begin(115200); // start the secondary serial port (for replies)
  Serial.setTimeout(MSG_TIMEOUT);
  Serial1.setTimeout(MSG_TIMEOUT);
  
  pinMode(13,OUTPUT); // LED output
  digitalWrite(13,LOW);
  
  delay(100);
  debugmsg = "Init motors"; // set up the motor controller initial state (everything stopped)
  debug();
  pinMode(L_MOTOR_PWM,OUTPUT);
  digitalWrite(L_MOTOR_PWM,LOW);
  pinMode(R_MOTOR_PWM,OUTPUT);
  digitalWrite(R_MOTOR_PWM,LOW);
  pinMode(L_MOTOR_DIR,OUTPUT);
  digitalWrite(L_MOTOR_DIR,LOW);
  pinMode(R_MOTOR_DIR,OUTPUT);
  digitalWrite(R_MOTOR_DIR,LOW);
  
  delay(100);
  debugmsg = "Init servos"; // set up all the servomotors' initial state
  debug();
  
  for(int i=0; i<NUM_SERVOS; i++)
  {
    servoPositions[i] = SERVO_DEFAULT;
    servoPins[i] = FIRST_SERVO_PIN + i;
  }
  
  delay(100);
  debugmsg = "Init sensors"; // set up all the sensors' initial state and timers
  debug();
  
  imuSendTimer = millis();  // for IMU
  
  encoderSendTimer = millis(); // for wheel encoder
  
  forceSendTimer = millis(); // for force sensor
  
  ledTimer = millis(); // for LED blinker
  
  debugmsg = "Init done";
  debug();
}


void loop()  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
{
  while(Serial.available()) // check for new messages until the buffer is empty
  {
    if(Serial.read() == '#') // if we find the message start character
      parseMessage(); // read the message
  }
  
  if(replyPending) // if we are expecting a reply, check until the buffer is empty
  {
    while(Serial1.available()) // note Serial1 is the secondary serial port
    {
      if(Serial.read() == '#') // if we find the message start character
        parseReply(); // read the message
    }
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

      
        
    
      
  
     
  
