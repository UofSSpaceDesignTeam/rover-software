
// This function is run constantly.


void loop()
{
  while(Serial.available()) // check for new messages until the buffer is empty
  {
    if(Serial.read() == '#') // if we find the message start character
    {
      if(parseMessage() != -2) // if we process the message with no receive errors
      {
        commTimer = millis(); // reset the timeout counter
        if(commTimeOut) // if we have timed out, we can now continue
        {
          commTimeOut = false;
          if(motorEnable)
          {
            for(int i=0; i<NUM_MOTORS; i++) // set motors to pre-timeout setting
            {
              motorArray[i]->set(motorArray[i]->setting());
            }
          }
        }
      }
    } 
  }
  
  if(timeOutEnable && millis() - commTimer > COMM_TIMEOUT) // No sign of life from fitpc
  {
    analogWrite(LEFT_MOTOR_PWM,0); // stop and wait for a valid message without interrupting motor object state.
    analogWrite(RIGHT_MOTOR_PWM,0);
    debugmsg = "timeout";
    debug();
    commTimeOut = true;
    digitalWrite(13,HIGH); // leave led on while waiting
  }
  
  if(!commTimeOut && millis() - ledTimer >= LED_BLINKRATE) // if it's time to blink the led
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
