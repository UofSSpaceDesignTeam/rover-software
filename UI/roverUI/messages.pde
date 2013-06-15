Message outMessage = new Message();
Message inMessage = new Message();
byte[] speed = new byte[2];
boolean move_enable_confirmed = false;
boolean move_disable_confirmed = true;
boolean move_pending = false; //becomes true when a movement command needs to be sent.
boolean emergency_sent = false;
int message_delay = 20;//a time delay (in milliseconds) between messages being sent to the rover
void sendMessages(){
  
  if ((move_upButton.isActive() || move_downButton.isActive() || move_leftButton.isActive() || move_rightButton.isActive()) && move_pending == true&&(millis()-last_message)>message_delay){
   
    if(connected){// only attempts to send the message if the user is connected to the rover
                  // (prevents null pointer errors which crash the program)
      speed[0] = byte(motor1Speed);//converts the motor speeds to bytes for transmission
      speed[1] = byte(motor2Speed);
   
      outMessage.Message(MessageProtocol.ID1_MOTORS,MessageProtocol.ID2_ROTATION,speed);
      outMessage.sendMessage(); 
      last_message=millis();
      bytes_sent += outMessage.getLength();
      
    } else {
      new_debug_message("Not connected");
    }
    move_pending = false;
    // this used to be where movement commands were sent,but they have been moved
    // to the mousePressed/keyPressed functions for better control
  }
  
  if (stop_moveButton.isActive()&&(millis()-last_message)>message_delay){
    
    //println(int(byte(200)));
    
    speed[0] = byte(127);
    speed[1] = byte(127);
    

    if(connected){
      outMessage.Message(MessageProtocol.ID1_MOTORS,MessageProtocol.ID2_ROTATION,speed);
      //println(char(outMessage.getMessage()));//for debugging purposes
      outMessage.sendMessage();  
      last_message = millis(); 
      bytes_sent += outMessage.getLength();

    }else{
      new_debug_message("Not connected");
    } 
    stop_moveButton.deactivate();//temporary loacation, should be deactivated upon confirmation from rover
    //outMessage.Message(MessageProtocol.ID1_MOTORS,MessageProtocol.ID2_ENABLE_DISABLE,null);  
  }
  if (enable_moveButton.isActive()&&(millis()-last_message)>message_delay){
    if(move_enable_confirmed == false){
    new_debug_message("Enable move");
    //println(int(byte(200)));
    byte[] data = new byte[1];
    data[0] = '1';
   
    //speed[0] = byte(127);
    //speed[1] = byte(127);
    //outMessage.Message(MessageProtocol.ID1_MOTORS,MessageProtocol.ID2_ROTATION,speed);
    //outMessage.sendMessage();    
    if(connected){
      outMessage.Message(MessageProtocol.ID1_MOTORS,MessageProtocol.ID2_ENABLE_DISABLE,data);
      outMessage.sendMessage(); 
      bytes_sent += outMessage.getLength();
      move_disable_confirmed = false;
      move_enable_confirmed = true;
      last_message = millis();
    }else{
      new_debug_message("Not connected");
      enable_moveButton.deactivate();
    } 
    }    
  } else if(!enable_moveButton.isActive()&&(millis()-last_message)>message_delay){
    if(move_disable_confirmed == false){
      new_debug_message("Disable move");
      //println(int(byte(200)));
      byte[] data = new byte[0];
      //data[0] = byte(0);
      
      //outMessage.Message(MessageProtocol.ID1_MOTORS,MessageProtocol.ID2_ROTATION,speed);
      //outMessage.sendMessage();  
      if(connected){  
        outMessage.Message(MessageProtocol.ID1_MOTORS,MessageProtocol.ID2_ENABLE_DISABLE,data);
        outMessage.sendMessage(); 
        bytes_sent += outMessage.getLength();
        move_disable_confirmed = true;
        move_enable_confirmed = false;
        last_message = millis();
      }else{
        new_debug_message("Not connected");
        enable_moveButton.activate();
      } 
    }
  }
  if(stopButton.isActive()&&(millis()-last_message)>message_delay&&emergency_sent==false){
    if(connected){  
        byte[] data = new byte[0];
        //data[0] = byte(0);
        outMessage.Message(MessageProtocol.ID1_POWER,MessageProtocol.ID2_EMERGENCY_OFF,data);
        outMessage.sendMessage(); 
        bytes_sent += outMessage.getLength();
        //stopButton.deactivate();
        last_message = millis();
        emergency_sent = true;
      }else{
        new_debug_message("Not connected");
      } 
    
  }
  

      
}

void getMessages(){
  if (client != null){
    if(client.available()>0){
      inMessage.recieveMessage();
      
      if (inMessage.getId1()=='D'){ 
        
        if(inMessage.getId2()=='B'){//Debug messages
          byte[] debug_data = inMessage.getData();
          println((char)debug_data[0]);
           
          String rover_debug_message = "Rover: ";
          //rover_debug_message += (char)debug_data[0];
          for(int i = 0;i!=debug_data.length;i++){
            rover_debug_message +=(char)debug_data[i];
          }
           
           new_debug_message(rover_debug_message);            
        }
      }
    } 
  }
}

