
// The gigantic function for reading and processing incoming messages. See "Message format information" on Google Drive


void parseMessage()
{
  if(!Serial.readBytes(msgid,2)) // try to read the two ID bytes
  {
    debugmsg = "err: no ID"; // if we timeout
    debug();
  }
  if(!Serial.readBytes(len,2)) // try to read the two length bytes
  {
    debugmsg = "err: no DL"; // if we timeout
    debug();
    if(isCritical(msgid))
      error(msgid,"TO");
  }
  msglength = 256*len[0] + len[1];
  if(!Serial.readBytes(msgdata,msglength)) // try to read the correct number of data bytes
  {
    debugmsg = "err: not enough data"; // if we timeout
    debug();
    if(isCritical(msgid))
      error(msgid,"TO");
  }
  
  switch(msgid[0]) // message category depends on the first ID byte
  {
    
    case 'P':  // power messages
    {
      if(msgid[1] == 'E') // emergency stop, stop all actuators quickly
      {
        analogWrite(L_MOTOR_PWM,0);
        analogWrite(R_MOTOR_PWM,0);
        for(int i=0; i<NUM_SERVOS; i++)
        {
          servoArray[i].detach();
        }
        reply("PE");
        while(true); // halt until reset
      }
      
      if(msgid[1] == 'O') // power off, not time critical
      {
        // todo: properly stop connected devices
        //todo: return all actuators to initial state
        analogWrite(L_MOTOR_PWM,0);
        analogWrite(R_MOTOR_PWM,0);
        for(int i=0; i<NUM_SERVOS; i++)
        {
          servoArray[i].detach();
        }
        reply("PO");
        while(true); // halt until reset
      }
      
      debugmsg = "err: ID, ID1 = P"; // if we get here something is wrong
      debug();
      error(msgid,"UM"); // power messages are all critical  
      return;
    }
    
    
    case 'D':  // data link messages
    {
      if(msgid[1] == 'C') // just a meaningless message to prevent timeout
      {
        reply("DC");
        return;
      }
      
      debugmsg = "err: ID, ID1 = D"; // if we get here ID2 is bad
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
        else if(msgdata[0] == '0')
        {
          digitalWrite(13,LOW);
        }
        else if(msgdata[0] == 'b')
        {
          ledBlink = true;
        }
        else
        {
          debugmsg = "err: LS: bad arg";
          debug();
        }
        return;
      }
      
      debugmsg = "err: ID, ID1 = L"; // if we get here ID2 is bad
      debug();
      return;
    }
    
    
    case 'M':  // motor messages
    {
      if(msgid[1] == 'E') // motor enable
      {
        if(msgdata[0] == '0')
        {
          motorEnable = false;
          analogWrite(L_MOTOR_PWM,0);
          analogWrite(R_MOTOR_PWM,0);
        }
        else if(msgdata[0] == '1')
        {
          motorEnable = true;
          debug();
        }
        else
        {
          debugmsg = "err: ME: bad arg";
          debug();
        }
        reply("ME");
        return;
      }
      
      if(msgid[1] == 'S') // motor set
      {
        if(motorEnable == true)
        {
          byte m0_pwm = (byte)msgdata[0];
          byte m1_pwm = (byte)msgdata[1];
          Serial.println(m0_pwm);
          if(m0_pwm == 127) // stop
          {
            analogWrite(L_MOTOR_PWM,0);
          }
          else if(m0_pwm > 127) // forward
          {
            digitalWrite(L_MOTOR_DIR,LOW);
            analogWrite(L_MOTOR_PWM,2*(m0_pwm-127));
          }
          else // reverse
          {
            digitalWrite(L_MOTOR_DIR,HIGH);
            analogWrite(L_MOTOR_PWM,2*(127-m0_pwm));
            Serial.println(2*(127-m0_pwm));
          }
          
          if(m1_pwm == 127) // stop
          {
            analogWrite(R_MOTOR_PWM,0);
          }
          else if(m1_pwm > 127) // forward
          {
            digitalWrite(R_MOTOR_DIR,LOW);
            analogWrite(R_MOTOR_PWM,2*(m1_pwm-127));
          }
          else // reverse
          {
            digitalWrite(R_MOTOR_DIR,HIGH);
            analogWrite(R_MOTOR_PWM,2*(127-m1_pwm));
          }
        }
        else
        {
          debugmsg = "err: MS: motors off";
          debug();
        }
        reply("MS");
        return;
      }
      
      debugmsg = "err: ID, ID1 = M"; // if we get here something is wrong
      debug();
      error(msgid,"UM"); // motor messages are all critical
      return;
    }
    
    
    case 'S':  // servo messages
    {
      if(msgid[1] == 'E') // servo enable
      {
        if(msgdata[0] == '1')
        {
          servoEnable = true;
          for(int i=0; i<NUM_SERVOS; i++) // activate each servo
          {
            servoArray[i].attach(servoPins[i]);
            servoArray[i].write(servoPositions[i]);
          }
        }
        else if(msgdata[0] == '0')
        {
          servoEnable = false;
          for(int i=0; i<NUM_SERVOS; i++)
          {
            servoArray[i].detach();  //detach each servo
          }
        }
        else
        {
          debugmsg = "err: SE: bad arg";
          debug();
        }
        reply("SE");
        return;
      }
      
      
      if(msgid[1] == 'S') // servo set
      {
        if(servoEnable)
        {
          for(int i=0; i<NUM_SERVOS; i++)
          {
            servoPositions[i] = msgdata[i];
            if(servoArray[i].attached()) // if servos are enabled
              servoArray[i].write(servoPositions[i]);
          }
        }
        else
        {
          debugmsg = "err: SS: servos off";
          debug();
        }
        reply("SS");
        return;
      }
      
      debugmsg = "err: ID, ID1 = S"; // if we get here something is wrong
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
      
      debugmsg = "err: ID, ID1 = I"; // if we get here something is wrong
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
      
      debugmsg = "err: ID, ID1 = W"; // if we get here something is wrong
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
      
      debugmsg = "err: ID, ID1 = F"; // if we get here something is wrong
      debug();
      return;
    }
  }
}
