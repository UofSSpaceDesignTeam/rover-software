
// Main sketch for the Arduino DUE on the USST Lunabotics Rover. Written in Arduino 1.5.2

// Required libraries

#include "Wire.h"  // i2c library
#include "I2Cdev.h"  // advanced i2c functions
#include "MPU6050.h" // i2c interfacing to imu
#include "Rangefinder.h" // library for IR sensors
#include "Encoder.h"  //  used to read wheel encoders
#include "Servo.h"  //  used to control servoMotors
#include "Motor.h" // used to control all Motors

// Operational constants

#define LED_BLINKRATE 300 // how often the led blinks
#define COMM_TIMEOUT 3000 // wait 3 seconds with no communication before stopping
#define MSG_TIMEOUT 100 // 100ms since last byte until we give up on a message

// Pin connections TODO: GET FINAL NUMBERS FOR MOTORS / ACTUATORS
// and also what really needs independent control

#define LEFTMOTORDIR 22
#define LEFTMOTORPWM 2
#define RIGHTMOTORDIR 23
#define RIGHTMOTORPWM 3

#define TESTACTUATORA 30
#define TESTACTUATORB 31
#define TESTACTUATORMOVE 32

#define FRONTLEFTSERVOPIN 47
#define FRONTRIGHTSERVOPIN 48
#define RIGHTSIDESERVOPIN 49
#define REARRIGHTSERVOPIN 50
#define REARLEFTSERVOPIN 51
#define LEFTSIDESERVOPIN 52

#define CAMERASERVOPIN 53

// Global variables and objects

char msgid[2]; // ID bytes of incoming messages
char len[2];  // length bytes of incoming messages
int msglength;  // length of incoming message
char msgdata[48];  // data bytes of incoming messages
String debugmsg = ""; // holds debug strings we want to send

boolean timeOutEnable = false; // whether to watch for fitpc time out
boolean commTimeOut = false; // timeout flag
unsigned long commTimer; // timeout timer

MPU6050 imu(0x68); // IMU object with i2c address
boolean imuExists = false; // whether it is working
boolean imuEnable = false;  // IMU state
unsigned int imuSendRate = 750; // how often we send IMU data
unsigned long imuSendTimer; // data reporting timer

boolean motorEnable = false; // Motor state
Motor leftMotor;
Motor rightMotor;

boolean actuatorEnable = false; // Actuator state
Actuator testActuator(140); // 140mm actuator

boolean rangefinderEnable = false; // Leveling & sensor state
Servo frontLeftServo;
Servo frontRightServo;
Servo rightSideServo;
Servo rearRightServo;
Servo rearLeftServo;
Servo leftSideServo; 
Servo cameraPan; // pans the xtion

Rangefinder frontLeftMidRange(2);
Rangefinder frontRightMidRange(2);
Rangefinder rightSideMidRange(2);
Rangefinder rearRightMidRange(2);
Rangefinder rearLeftMidRange(2);
Rangefinder leftSideMidRange(2);

boolean encoderEnable = false; // wheel encoder state information
unsigned int encoderSendRate = 500;
unsigned long encoderSendTimer;

boolean forceEnable = false; // force sensor state information
unsigned int forceSendRate = 500;
unsigned long forceSendTimer;

boolean ledState = false; // led state information
unsigned long ledTimer;
      
        
    
      
  
     
  
