
// Main sketch for the Arduino DUE on the USST Lunabotics Rover. Written in Arduino 1.5.2

// Required libraries

#include "Wire.h"  // i2c library
#include "I2Cdev.h"  // advanced i2c functions
#include "MPU6050.h" // i2c interfacing to imu
#include "Encoder.h"  //  used to read wheel encoders
#include "Servo.h"  //  used to control servoMotors
#include "TestPlatformMotor.h" // used to control all Motors

// Hardware parameters for things stored in arrays

#define NUM_MOTORS 2
#define NUM_SERVOS 6
#define NUM_ENCODERS 0
#define NUM_FORCE 0

// Operational constants

#define LED_BLINKRATE 300 // how often the led blinks
#define COMM_TIMEOUT 3000 // wait 3 seconds with no communication before stopping
#define MSG_TIMEOUT 100 // 100ms since last byte until we give up on a message

// Global variables and objects

char msgid[2]; // ID bytes of incoming messages
char len[2];  // length bytes of incoming messages
int msglength;  // length of incoming message
char msgdata[48];  // data bytes of incoming messages
String debugmsg = ""; // holds debug strings we want to send

boolean timeOutEnable = false; // whether to watch for fitpc time out
boolean commTimeOut = false; // timeout flag
unsigned long commTimer; // timeout timer

MPU6050 imu(0x68); // imu object with i2c address
boolean imuExists = false; // whether it is working
boolean imuEnable = false;  // IMU state
unsigned int imuSendRate = 500; // how often we send IMU data
unsigned long imuSendTimer; // data reporting timer

boolean motorEnable = false; // Motor state
TestPlatformMotor leftMotor; // Motor 0
TestPlatformMotor rightMotor; // Motor 1
TestPlatformMotor *motorArray[NUM_MOTORS] = {&leftMotor, &rightMotor}; // array of Motor pointers
const byte motorPins[2][NUM_MOTORS] = {{46, 3}, {47, 4}};

boolean servoEnable = false; // Servo state
Servo frontLeft;  // Servo 0
Servo frontRight;  // Servo 1
Servo rightSide;  // Servo 2
Servo rearRight;  // Servo 3
Servo rearLeft;  // Servo 4
Servo leftSide;  // Servo 5
Servo *servoArray[NUM_SERVOS] = {&frontLeft, &frontRight, &rightSide, &rearRight, &rearLeft, &leftSide}; // array of servo pointers
const byte servoPins[NUM_SERVOS] = {20, 21, 22, 23, 24, 25}; // servo signal pins

boolean encoderEnable = false; // wheel encoder state information
unsigned int encoderSendRate = 500;
unsigned long encoderSendTimer;

boolean forceEnable = false; // force sensor state information
unsigned int forceSendRate = 500;
unsigned long forceSendTimer;

boolean ledState = false; // led state information
unsigned long ledTimer;
      
        
    
      
  
     
  
