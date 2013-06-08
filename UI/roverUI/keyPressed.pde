
// activates every time a keyboard button is pressed down
// does action corresponding to which button was pressed

int scrollAdjust = 5; //increment at which scrollbar moves with each button press of PAGE UP or PAGE DOWN 

void keyPressed() {
  
  if (key == CODED) {// special case for specail keys**: UP, DOWN, LEFT, RIGHT, arrow keys, ALT, CONTROL, SHIFT
                     // need to use keyCode == __ instead of key = __ (ie: keyCode = DOWN for down arrow key)
                     // some keys are coded using numbers, ie: keyCode = 35 for the "End" button
                      // **does not include BACKSPACE, TAB, ENTER, RETURN, ESC, and DELETE [ie: can use if(key == TAB)]
                      // key == ' ' for spacebar
    if (keyCode == UP) {
      // pressing the up arrow key tells the rover to move forward
      motor1Speed = speedbar.getPos()+127; //sets both motors to a positive speed
      motor2Speed = speedbar.getPos()+127; // based on position of speed slider
      move_pending = true;
      //the following code was moved from the "messages" tab to allow press-and-hold movement
//      speed[0] = byte(motor1Speed);//converts the motor speeds to bytes for transmission
//      speed[1] = byte(motor2Speed);
//      
//      if(connected){// only attempts to send the message if the user is connected to the rover
//                    // (prevents null pointer errors which crash the program)
//        outMessage.Message(MessageProtocol.ID1_MOTORS,MessageProtocol.ID2_ROTATION,speed);
//        outMessage.sendMessage(); 
//      } else {
//        println("Not connected");
//      }
      move_upButton.activate();
      move_downButton.deactivate();
      move_leftButton.deactivate();
      move_rightButton.deactivate();

    } else if (keyCode == DOWN) { 
      // pressing the down arrow key tells the rover to move backward      
       motor1Speed = motor1Speed =127 - speedbar.getPos();
       motor2Speed = motor2Speed =127 - speedbar.getPos();
       move_pending = true;
//      speed[0] = byte(motor1Speed);
//      speed[1] = byte(motor2Speed);
//    
//      if(connected){
//        outMessage.Message(MessageProtocol.ID1_MOTORS,MessageProtocol.ID2_ROTATION,speed);
//        outMessage.sendMessage(); 
//      }else {
//        println("Not connected");
//      }
      
       move_upButton.deactivate();
       move_downButton.activate();
       move_leftButton.deactivate();
       move_rightButton.deactivate();

    } else if (keyCode == LEFT) { 
      // pressing the left arrow key tells the rover to turn left
       motor1Speed = motor1Speed =speedbar.getPos()+127;
       motor2Speed = motor2Speed =127 - speedbar.getPos();
       move_pending = true;
//      speed[0] = byte(motor1Speed);
//      speed[1] = byte(motor2Speed);
//      println(int(speed[0]));
//      println(int(speed[1]));
//
//      if(connected){
//        outMessage.Message(MessageProtocol.ID1_MOTORS,MessageProtocol.ID2_ROTATION,speed);
//        outMessage.sendMessage(); 
//      } else {
//        println("Not connected");
//      }
 
      move_upButton.deactivate();
      move_downButton.deactivate();
      move_leftButton.activate();
      move_rightButton.deactivate();
       
    }else if (keyCode == RIGHT) { 
      // pressing the right arrow key tells the rover to turn right   
       motor1Speed = motor1Speed =127 - speedbar.getPos();
       motor2Speed = motor2Speed =speedbar.getPos()+127;
       move_pending = true;
//      speed[0] = byte(motor1Speed);
//      speed[1] = byte(motor2Speed);
//      println(int(speed[0]));
//      println(int(speed[1]));
//      if(connected){
//        outMessage.Message(MessageProtocol.ID1_MOTORS,MessageProtocol.ID2_ROTATION,speed);
//        outMessage.sendMessage(); 
//      } else {
//        println("Not connected");
//      }
      move_upButton.deactivate();
      move_downButton.deactivate();
      move_leftButton.deactivate();
      move_rightButton.activate();
       
    }else if (keyCode == 33) { //keyCode 33 corresponds to the Page Up button
    // pressing the Page Up button moves the speed slider to the right (increase speed)
      if (speedbar.newspos + scrollAdjust/speedbar.ratio < speedbar.sposMax){ // checks to see if speed slider is at maximum
        speedbar.newspos = speedbar.newspos + scrollAdjust/speedbar.ratio;    // only increases speed up to maximum value (currently 127)
      } else {
        speedbar.newspos = speedbar.sposMax;
      }  
    }else if (keyCode == 34) { //keyCode 34 corresponds to the Page Down button
     // pressing the Page Down button moves the speed slider to the left (decrease speed)
      if (speedbar.newspos - scrollAdjust/speedbar.ratio > speedbar.sposMin){ // checks to see if speed slider is at minimum
        speedbar.newspos = speedbar.newspos - scrollAdjust/speedbar.ratio;    // only decreases speed down to minimum value (currently 0)
      } else {
        speedbar.newspos = speedbar.sposMin;
      }
    } else if (keyCode == 35) { //keyCode 35 corresponds to the End button
     // pressing the End button activates the "stop moving" button and tells the rover to stop (this is NOT the emergency stop)
      stop_moveButton.activate();
    } else if (keyCode == 36) { //keyCode 36 corresponds to the Home button
     // pressing the Home button enables/disables the motors
      if (enable_moveButton.isActive()){
        enable_moveButton.deactivate();
      }else{
        enable_moveButton.activate();
      }
    }
    
  } else { //case for alphanumeric keys and symbols
    if (key == 'A'||key =='a') { 
      d_downButton.activate();
    } else if (key == 'Q'||key =='q') { 
      d_upButton.activate();
    }else if (key == 'W'||key =='w') { 
      d_outButton.activate();
    }else if (key == 'E'||key =='e') { 
      d_inButton.activate();
    }else if (key == 'D'||key =='d') { 
      if(!d_digButton.isActive()){
        d_digButton.activate();
      } else{
        d_stopButton.activate();
      }
    } else if (key == 'S'||key =='s') { 
      if(wireless_serverButton.isActive()){
        wireless_serverButton.deactivate();
        tethered_serverButton.activate();
       } else if (tethered_serverButton.isActive()){
        test_serverButton.activate();
        tethered_serverButton.deactivate();
      } else if (test_serverButton.isActive()){
        test_serverButton.deactivate();
        wireless_serverButton.activate();
      }else {
        test_serverButton.activate(); 
      }
    } else if (key == ' ') { 
      stopButton.activate();
    }
  }
   
}


//keyReleased activates when a key is released
//similar logic as keyPressed in termes of key and keyCode functionality
void keyReleased(){
  
  if (key == CODED) {
    if (keyCode == UP && move_upButton.isActive()) {
//       speed[0] = byte(127);
//       speed[1] = byte(127);
//       
//       outMessage.Message(MessageProtocol.ID1_MOTORS,MessageProtocol.ID2_ROTATION,speed);
//       if(connected){
//         outMessage.sendMessage();    
//       } else {
//        println("Not connected");
//      }    
       
       if(connected){
          stop_moveButton.activate();
       }
       move_upButton.deactivate();
       move_leftButton.deactivate();
       move_rightButton.deactivate();
     
    } else if (keyCode == DOWN && move_downButton.isActive()) { 
          if(connected){
           stop_moveButton.activate();
          }
          move_downButton.deactivate();
//       if(connected){
//         move_downButton.deactivate();
//         speed[0] = byte(127);
//         speed[1] = byte(127);
//       
//         outMessage.Message(MessageProtocol.ID1_MOTORS,MessageProtocol.ID2_ROTATION,speed);
//         outMessage.sendMessage();    
//       }  
    } else if (keyCode == LEFT && move_leftButton.isActive()) { 
         if(connected){
          stop_moveButton.activate();
         }     
         move_leftButton.deactivate();
//       if(connected){
//         move_leftButton.deactivate();
//         speed[0] = byte(127);
//         speed[1] = byte(127);
//       
//         outMessage.Message(MessageProtocol.ID1_MOTORS,MessageProtocol.ID2_ROTATION,speed);
//         outMessage.sendMessage();    
//       }  
    } else if (keyCode == RIGHT && move_rightButton.isActive()) { 
        if(connected){
          stop_moveButton.activate();
        }
        move_rightButton.deactivate();
//       if(connected){
//         move_rightButton.deactivate();
//         speed[0] = byte(127);
//         speed[1] = byte(127);
//       
//         outMessage.Message(MessageProtocol.ID1_MOTORS,MessageProtocol.ID2_ROTATION,speed);
//         outMessage.sendMessage();    
//       }  
    } else if (keyCode == 35) { //keyCode 35 corresponds to the End button
      stop_moveButton.deactivate();
    } else if (keyCode == TAB) { 
    
    }
    
  } else {
    if (key == 'A') {
   
    } else { 

    }
  }  
  
  
  
}
