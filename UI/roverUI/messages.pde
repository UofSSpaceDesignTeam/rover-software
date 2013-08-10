Message outMessage = new Message();
Message inMessage = new Message();
byte[] speed = new byte[2];
boolean move_enable_confirmed = false;
boolean move_disable_confirmed = true;
boolean move_pending = false; //becomes true when a movement command needs to be sent.
boolean emergency_sent = false;
int message_delay = 30;//a time delay (in milliseconds) between messages being sent to the rover



void sendMessages(){
  
  if ((move_upButton.isActive() || move_downButton.isActive() || move_leftButton.isActive() || move_rightButton.isActive()) && move_pending == true&&(millis()-last_message)>message_delay){
   
    if(connected){// only attempts to send the message if the user is connected to the rover
                  // (prevents null pointer errors which crash the program)
      speed[0] = byte(motor1Speed);//converts the motor speeds to bytes for transmission
      speed[1] = byte(motor2Speed);
      if(enable_moveButton.isActive() && move_enable_confirmed == true){
        outMessage.Message(MessageProtocol.ID1_MOTORS,MessageProtocol.ID2_ROTATION,speed);
        outMessage.sendMessage(); 
        last_message=millis();
      } else {
        new_debug_message("Movement Not Enabled");
      }
      
    } else {
      //notifies the user if there is no connection
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
      if(enable_moveButton.isActive() && move_enable_confirmed == true){
        outMessage.Message(MessageProtocol.ID1_MOTORS,MessageProtocol.ID2_ROTATION,speed);
        //println(char(outMessage.getMessage()));//for debugging purposes
        outMessage.sendMessage();  
        last_message = millis();  
      } else {
        new_debug_message("Movement Not Enabled");
      }
    }else{
      new_debug_message("Not connected");
    } 
    
    /*temporary loacation, should be deactivated upon confirmation from rover*/
    stop_moveButton.deactivate();
  }
  if (enable_moveButton.isActive()&&(millis()-last_message)>message_delay){
    if(move_enable_confirmed == false){
    
   
    //speed[0] = byte(127);
    //speed[1] = byte(127);
    //outMessage.Message(MessageProtocol.ID1_MOTORS,MessageProtocol.ID2_ROTATION,speed);
    //outMessage.sendMessage();    
    if(connected){
      new_debug_message("Enable move");
      //println(int(byte(200)));
      byte[] data = new byte[1];
      data[0] = '1';
      outMessage.Message(MessageProtocol.ID1_MOTORS,MessageProtocol.ID2_ENABLE_DISABLE,data);
      outMessage.sendMessage(); 
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
      
      
      //outMessage.Message(MessageProtocol.ID1_MOTORS,MessageProtocol.ID2_ROTATION,speed);
      //outMessage.sendMessage();  
      if(connected){  
        new_debug_message("Disable move");
        //println(int(byte(200)));
        byte[] data = new byte[1];
        data[0] = byte(0);
        outMessage.Message(MessageProtocol.ID1_MOTORS,MessageProtocol.ID2_ENABLE_DISABLE,data);
        outMessage.sendMessage(); 
        move_disable_confirmed = true;
        move_enable_confirmed = false;
        last_message = millis();
      }else{
        new_debug_message("Not connected");
        enable_moveButton.activate();
      } 
    }
  }
//Emergency Stop Message
  //only sends a message once and only if the emergency stop button has been activated  
  if(stopButton.isActive()&&(millis()-last_message)>message_delay&&emergency_sent==false){
    //only attempts to send the message while connected to the rover
    if(connected){
        //message class needs a byte variable input for data, but the emergency stop message needs no data other than the ID1 and ID2
        // so it sends a byte array of zero length  
        byte[] data = new byte[1];
        data[0] = 0;
        //sets up the message to be sent
        outMessage.Message(MessageProtocol.ID1_POWER,MessageProtocol.ID2_EMERGENCY_OFF,data);
        //sends message
        outMessage.sendMessage();
        
       
        //adds message length to total bytes sent (used for measuring bandwidth use) 
        
        
        last_message = millis();
        //message sent successfully, so don't need to attempt to sent again
        emergency_sent = true;
      }else{
        new_debug_message("Not connected");
        stopButton.deactivate();
      }  
  }
  
  if(stopButton.isActive()&&(millis()-last_message)>message_delay&&emergency_sent==false){
    //only attempts to send the message while connected to the rover
    if(connected){
        /*message class needs a byte variable input for data, but the emergency stop message needs no data other than the ID1 and ID2
          so it sends a byte array of zero length  */ 
        byte[] data = new byte[1];
        data[0]=0;
        /*sets up the message to be sent*/
        outMessage.Message(MessageProtocol.ID1_POWER,MessageProtocol.ID2_EMERGENCY_OFF,data);
        /*sends message*/
        outMessage.sendMessage();
        
       
        /*adds message length to total bytes sent (used for measuring bandwidth use)*/ 
        
        
       
        /*message sent successfully, so don't need to attempt to sent again*/
        emergency_sent = true;
      }else{
        new_debug_message("Not connected");
        stopButton.deactivate();
      } 
  }


   if(d_upButton.isActive()&&(millis()-last_message)>message_delay && d_upButton.isPending()){
      /*only attempts to send the message while connected to the rover*/
      if(connected){
          
          byte[] data = new byte[1];
          data[0]=0;
          /*sets up the message to be sent*/
          outMessage.Message(MessageProtocol.ID1_DIGGER,MessageProtocol.ID2_RAISE,data);
          /*sends message*/
          outMessage.sendMessage();
          d_upButton.sent();
        }else{
          new_debug_message("Not connected");
          d_upButton.deactivate();
        }
          
    }
    if(d_downButton.isActive()&&(millis()-last_message)>message_delay && d_downButton.isPending()){
      /*only attempts to send the message while connected to the rover*/
      if(connected){
          
          byte[] data = new byte[1];
          data[0]=0;
          /*sets up the message to be sent*/
          outMessage.Message(MessageProtocol.ID1_DIGGER,MessageProtocol.ID2_LOWER,data);
          /*sends message*/
          outMessage.sendMessage();
          d_downButton.sent();
        }else{
          new_debug_message("Not connected");
          d_downButton.deactivate();
        } 
    }
    if(d_outButton.isActive()&&(millis()-last_message)>message_delay && d_outButton.isPending()){
      /*only attempts to send the message while connected to the rover*/
      if(connected){
          
          byte[] data = new byte[1];
          data[0]=0;
          /*sets up the message to be sent*/
          outMessage.Message(MessageProtocol.ID1_DIGGER,MessageProtocol.ID2_OUT,data);
          /*sends message*/
          outMessage.sendMessage();
          d_outButton.sent();
        }else{
          new_debug_message("Not connected");
          d_outButton.deactivate();
        } 
    }
    if(d_inButton.isActive()&&(millis()-last_message)>message_delay && d_inButton.isPending()){
      /*only attempts to send the message while connected to the rover*/
      if(connected){
          
          byte[] data = new byte[1];
          data[0]=0;
          /*sets up the message to be sent*/
          outMessage.Message(MessageProtocol.ID1_DIGGER,MessageProtocol.ID2_IN,data);
          /*sends message*/
          outMessage.sendMessage();
          d_inButton.sent();
        }else{
          new_debug_message("Not connected");
          d_inButton.deactivate();
        } 
       
    }
    if(d_digButton.isActive()&&(millis()-last_message)>message_delay && d_digButton.isPending()){
      /*only attempts to send the message while connected to the rover*/
      if(connected){
          
          byte[] data = new byte[1];
          data[0]=0;
          /*sets up the message to be sent*/
          outMessage.Message(MessageProtocol.ID1_DIGGER,MessageProtocol.ID2_DIG,data);
          /*sends message*/
          outMessage.sendMessage();
          d_digButton.sent();                    

        }else{
          new_debug_message("Not connected");
          //d_digButton.deactivate();
        } 
    }
    
    if(d_stopButton.isActive()&&(millis()-last_message)>message_delay && d_stopButton.isPending() ){
      /*only attempts to send the message while connected to the rover*/
      if(connected){
          
          byte[] data = new byte[1];
          data[0]=0;
          /*sets up the message to be sent*/
          outMessage.Message(MessageProtocol.ID1_DIGGER,MessageProtocol.ID2_STOP_DIG,data);
          /*sends message*/
          outMessage.sendMessage();
          d_stopButton.sent();                    

        }else{
          new_debug_message("Not connected");
          d_stopButton.deactivate();
        } 
    }
    if(d_autoButton.isActive()&&(millis()-last_message)>message_delay && d_autoButton.isPending()){
      /*only attempts to send the message while connected to the rover*/
      if(connected){
          
          byte[] data = new byte[1];
          data[0]=0;
          /*sets up the message to be sent*/
          outMessage.Message(MessageProtocol.ID1_DIGGER,MessageProtocol.ID2_AUTO,data);
          /*sends message*/
          outMessage.sendMessage();
          d_autoButton.sent();                    

        }else{
          new_debug_message("Not connected");
          d_autoButton.deactivate();
        } 
       
    }
    if(b_upButton.isActive()&&(millis()-last_message)>message_delay && b_upButton.isPending()){
      /*only attempts to send the message while connected to the rover*/
      if(connected){
          
          byte[] data = new byte[1];
          data[0]=0;
          /*sets up the message to be sent*/
          outMessage.Message(MessageProtocol.ID1_BUCKET,MessageProtocol.ID2_RAISE,data);
          /*sends message*/
          outMessage.sendMessage();
          b_upButton.sent();
        }else{
          new_debug_message("Not connected");
          b_upButton.deactivate();
        }
          
    }
    if(b_downButton.isActive()&&(millis()-last_message)>message_delay && b_downButton.isPending()){
      /*only attempts to send the message while connected to the rover*/
      if(connected){
          
          byte[] data = new byte[1];
          data[0]=0;
          /*sets up the message to be sent*/
          outMessage.Message(MessageProtocol.ID1_BUCKET,MessageProtocol.ID2_LOWER,data);
          /*sends message*/
          outMessage.sendMessage();
          b_downButton.sent();
        }else{
          new_debug_message("Not connected");
          b_downButton.deactivate();
        } 
    }
    if(b_levelButton.isActive()&&(millis()-last_message)>message_delay && b_levelButton.isPending()){
      /*only attempts to send the message while connected to the rover*/
      if(connected){
          
          byte[] data = new byte[1];
          data[0]=0;
          /*sets up the message to be sent*/
          outMessage.Message(MessageProtocol.ID1_BUCKET,MessageProtocol.ID2_LEVEL,data);
          /*sends message*/
          outMessage.sendMessage();
          b_levelButton.sent();
        }else{
          new_debug_message("Not connected");
          b_levelButton.deactivate();
        } 
    }
    if(b_dumpButton.isActive()&&(millis()-last_message)>message_delay && b_dumpButton.isPending()){
      /*only attempts to send the message while connected to the rover*/
      if(connected){
          
          byte[] data = new byte[1];
          data[0]=0;
          /*sets up the message to be sent*/
          outMessage.Message(MessageProtocol.ID1_BUCKET,MessageProtocol.ID2_DUMP,data);
          /*sends message*/
          outMessage.sendMessage();
          b_dumpButton.sent();
        }else{
          new_debug_message("Not connected");
          b_dumpButton.deactivate();
        } 
       
    }
    
    
    if(b_autoButton.isActive()&&(millis()-last_message)>message_delay && b_autoButton.isPending()){
      /*only attempts to send the message while connected to the rover*/
      if(connected){
          
          byte[] data = new byte[1];
          data[0]=0;
          /*sets up the message to be sent*/
          outMessage.Message(MessageProtocol.ID1_BUCKET,MessageProtocol.ID2_AUTO,data);
          /*sends message*/
          outMessage.sendMessage();
          b_autoButton.sent();                    

        }else{
          new_debug_message("Not connected");
          b_autoButton.deactivate();
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

