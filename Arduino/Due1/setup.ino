
// This function is run once on boot.


void setup()
{
  while (!Serial); // wait for main serial port to open before starting
  Serial.begin(115200); // start the USB Serial port
  Serial.setTimeout(MSG_TIMEOUT);
  
  pinMode(13,OUTPUT); // set LED off to start
  digitalWrite(13,LOW);
  
  delay(100);
  debugmsg = "Arduino DUE Initializing:"; // first message on boot
  debug();
  
  delay(10);
  debugmsg = "Motors"; // set up the motors
  debug();
  for(int i=0; i<NUM_MOTORS; i++)
  {
    motorArray[i]->attach(motorPins[i][1],motorPins[i][2]);
  }
  
  delay(10);
  debugmsg = "Servos"; // set up the servos
  debug();
  for(int i=0; i<NUM_SERVOS; i++)
  {
    servoArray[i]->attach(servoPins[i]);
  }
  
  delay(10);
  debugmsg = "Encoders"; // set up the wheel encoders
  debug();
  //stuff
  
  delay(10);
  debugmsg = "Force Sensors"; // set up the force sensors
  debug();
  //stuff
  
  delay(10);
  debugmsg = "IMU"; // set up the imu
  debug();
  Wire.begin();
  delay(5);
  imu.initialize();
  delay(5);
  if(imu.testConnection())
  {
    imuExists = true;
  }
  else
  {
    debugmsg = ("IMU unavailable");
    debug();
  }
  
  delay(10);
  debugmsg = "Timers";
  debug();
  imuSendTimer = millis();
  forceSendTimer = millis();
  encoderSendTimer = millis();
  ledTimer = millis(); // for LED blinker
  commTimer = millis();  // for communication timeout
  
  delay(10);
  debugmsg = "Done";
  debug();
}
