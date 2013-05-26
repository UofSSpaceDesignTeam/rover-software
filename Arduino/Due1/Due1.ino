
// Main sketch for the Arduino DUE on the USST Lunabotics Rover. Written in Arduino 1.5.2


// Required libraries

#include "Wire.h"  // i2c library for imu
#include "imuhack.h"  // imu functions (in development)
#include "Encoder.h"  //  used to read wheel encoders
#include "Servo.h"  //  used to control servomotors
#include "Motor.h" // used to control all motors

// Hardware parameters

#define NUM_MOTORS 2
#define NUM_SERVOS 2
#define NUM_ENCODERS 0
#define NUM_FORCE 0

// Pin connections

#define LEFT_MOTOR_DIR 46
#define LEFT_MOTOR_PWM 3
#define RIGHT_MOTOR_DIR 47
#define RIGHT_MOTOR_PWM 4

#define PAN_SERVO_PIN 20
#define TILT_SERVO_PIN 21

// Operational constants

#define LED_BLINKRATE 300 // how often the led blinks
#define COMM_TIMEOUT 3000 // wait 3 seconds with no communication before stopping
#define MSG_TIMEOUT 100 // 100ms since last byte until we give up on a message

#define PAN_SERVO_DEFAULT 90 // initial postions of servos
#define TILT_SERVO_DEFAULT 90

// Global variables and objects

char msgid[2]; // ID bytes of incoming messages
char len[2];  // length bytes of incoming messages
int msglength;  // length of incoming message
char msgdata[48];  // data bytes of incoming messages
String debugmsg = ""; // holds messages we want to send

boolean timeOutEnable = false; // whether to watch for fitpc time out
boolean commTimeOut = false; // timeout flag
unsigned long commTimer; // timeout timer

boolean imuEnable = false;  // IMU state
unsigned int imuSendRate = 500; // how often we send IMU data
unsigned long imuSendTimer; // data reporting timer

boolean motorEnable = false; // Motor state
Motor leftMotor; // motor 0
Motor rightMotor; // motor 1
Motor *motorArray[NUM_MOTORS] = {&leftMotor, &rightMotor}; // array of motor pointers

Servo panServo; // servo 0
Servo tiltServo; // servo 1
Servo *servoArray[NUM_SERVOS] = {&panServo, &tiltServo}; // array of servo pointers

boolean encoderEnable = false; // wheel encoder state information
unsigned int encoderSendRate = 500;
unsigned long encoderSendTimer;

boolean forceEnable = false; // force sensor state information
unsigned int forceSendRate = 500;
unsigned long forceSendTimer;

boolean ledState = false; // led state information
unsigned long ledTimer;
      
        
    
      
  
     
  
