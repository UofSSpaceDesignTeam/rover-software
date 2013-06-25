
// The gigantic function for reading and processing incoming messages. See "Message format information" on Google Drive.
// returns -2 for invalid message format or type, -1 for general failure, 0 for success


int parseMessage()
{
  if(!Serial.readBytes(msgid,2)) // try to read the two ID bytes
  {
    debugmsg = "err 1"; // if we timeout
    debug();
    return -2;
  }
  if(!Serial.readBytes(len,2)) // try to read the two length bytes
  {
    debugmsg = "err 2"; // if we timeout
    debug();
    return -2;
  }
  msglength = 256*len[0] + len[1];
  if(!Serial.readBytes(msgdata,msglength)) // try to read the correct number of data bytes
  {
    debugmsg = "err 3"; // if we timeout
    debug();
    return -2;
  }
  
  switch(msgid[0]) // message category depends on the first ID byte
  {
    
    case 'P':  // power messages
    {
      if(msgid[1] == 'E') // emergency stop, stop all actuators quickly
      {
        leftMotor.set(127);  // stop moving
        rightMotor.set(127);
        digitalWrite(13,LOW); // led off
        while(true); // halt until reset
      }
      
      if(msgid[1] == 'O') // power off, not time critical
      {
        leftMotor.set(127);  // stop moving
        rightMotor.set(127);
        // todo: properly stop connected devices
        //todo: return all actuators to initial positions
        digitalWrite(13,LOW); // led off
        while(true); // halt until reset
      }
      
      debugmsg = "err 4"; // if we get here ID2 is bad
      debug();
      return -2;
    }
    
    
    case 'D':  // data link messages
    {
      if(msgid[1] == 'C') // just a meaningless message to prevent timeout
      {
        return 0;
      }
      
      debugmsg = "err 5"; // if we get here ID2 is bad
      debug();
      return -2;
    }
    
    
    case 'M':  // motor messages
    {
      if(msgid[1] == 'E') // motor enable commands
      {
        if(msgdata[0] == '0') // disable motors
        {
          leftMotor.set(127);
          rightMotor.set(127);
          motorEnable = false;
        }
        else if(msgdata[0] == '1') // enable motors
        {
          motorEnable = true;
        }
        else
        {
          debugmsg = "err 8"; // data byte is invalid
          debug();
          return -1;
        }
        return 0;
      }
      
      if(msgid[1] == 'S') // motor set
      {
        if(motorEnable)
        {
          leftMotor.set((byte)msgdata[0]);
          rightMotor.set((byte)msgdata[1]);
        }
        else
        {
          debugmsg = "err 9"; // trying to set motors while disabled
          debug();
          return -1;
        }
        return 0;
      }
      
      debugmsg = "err 10"; // if we get here ID2 is bad
      debug();
      return -2;
    }
    
    
    case 'R':  // rangefinder messages
    {
      if(msgid[1] == 'E')  // rangefinder system enable
      {
        if(msgdata[0] == '1')
        {
          rangefinderEnable = true;
          frontLeftServo.attach(FRONTLEFTSERVOPIN);
          frontRightServo.attach(FRONTRIGHTSERVOPIN);
          rightSideServo.attach(RIGHTSIDESERVOPIN);
          rearRightServo.attach(REARRIGHTSERVOPIN);
          rearLeftServo.attach(REARLEFTSERVOPIN);
          leftSideServo.attach(LEFTSIDESERVOPIN);
        }
        else if(msgdata[0] == '0')
        {
          rangefinderEnable = false;
          frontLeftServo.detach();
          frontRightServo.detach();
          rightSideServo.detach();
          rearRightServo.detach();
          rearLeftServo.detach();
          leftSideServo.detach();
        }
        else
        {
          debugmsg = "err 11";  // nope
          debug();
          return -1;
        }
        return 0;
      }
      
      //if(msgid[1] == 'Q') // rangefinder data request!
      //{
        //for(int i=0; i<20; i++)
      
      debugmsg = "err 14"; // if we get here ID2 is bad
      debug();
      return -2;
    }
    
    
    case 'I':  // IMU messages
    {
      if(msgid[1] == 'E') // IMU enable
      {
        if(msgdata[0] == '1')
        {
          if(imuExists)
          {
            imuEnable = true;
          }
          else
          {
            debugmsg = "err 6";
            debug();
          }
        }
        else if(msgdata[0] == '0')
        {
          imuEnable = false;
        }
        else
        {
          debugmsg = "err 15"; // data byte is invalid
          debug();
          return -1;
        }
        return 0;
      }
      
      if(msgid[1] == 'R') // IMU send rate
      {
        imuSendRate = parseBytes((byte)msgdata[0],(byte)msgdata[1]);
        return 0;
      }
      
      debugmsg = "err 16"; // if we get here ID2 is bad
      debug();
      return -2;
    }
    
    
    case 'W':  // encoder messages
    {
      if(msgid[1] == 'E') // encoder enable
      {
        if(msgdata[0] == '1')
        {
          encoderEnable = true;
          // todo: begin grabbing data from all encoders
        }
        else if(msgdata[0] == '0')
        {
          encoderEnable = false;
          // todo: stop grabbing data from all encoders
        }
        else
        {
          debugmsg = "err 17"; // data byte is invalid
          debug();
          return -1;
        }
        return 0;
      }
      
      if(msgid[1] == 'R') // encoder send rate
      {
        encoderSendTimer = parseBytes((byte)msgdata[0],(byte)msgdata[1]);
        return 0;
      }
      
      debugmsg = "err 18"; // if we get here ID2 is bad
      debug();
      return -2;
    }


    case 'F':  // force sensor messages
    {
      if(msgid[1] == 'E') // force sensor enable
      {
        if(msgdata[0] == '1')
        {
          forceEnable = true;
          // todo: begin grabbing data from all force sensors
        }
        else if(msgdata[0] == '0')
        {
          forceEnable = false;
          // todo: stop grabbing data from all force sensors
        }
        else
        {
          debugmsg = "err 19"; // data byte is invalid
          debug();
          return -1;
        }
        return 0;
      }
      
      if(msgid[1] == 'R') // force sensor send rate
      {
        forceSendTimer = parseBytes((byte)msgdata[0],(byte)msgdata[1]);
        return 0;
      }
      
      debugmsg = "err 20"; // if we get here ID2 is bad
      debug();
      return -2;
    }
  }
}
