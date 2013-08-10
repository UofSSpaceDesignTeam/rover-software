
// activates every time a keyboard button is pressed down
// does action corresponding to which button was pressed
int scrollAdjust = 5; //increment at which scrollbar moves with each button press of PAGE UP or PAGE DOWN 

void keyPressed() {
 
 if (key == CODED) {// special case for specail keys**: UP, DOWN, LEFT, RIGHT, arrow keys, ALT, CONTROL, SHIFT
                       // need to use keyCode == __ instead of key = __ (ie: keyCode = DOWN for down arrow key)
                       // some keys are coded using numbers, ie: keyCode = 35 for the "End" button
                        // **does not include BACKSPACE, TAB, ENTER, RETURN, ESC, and DELETE [ie: can use if(key == TAB)]
                        // key == ' ' for spacebar
    if (keydown == false){
      keydown = true;
      if (keyCode == UP) {
        // pressing the up arrow key tells the rover to move forward
        motor1Speed = speedbar.getPos()+127; //sets both motors to a positive speed
        motor2Speed = speedbar.getPos()+127; // based on position of speed slider
        move_pending = true;
       
        move_upButton.activate();
        move_downButton.deactivate();
        move_leftButton.deactivate();
        move_rightButton.deactivate();
  
      } else if (keyCode == DOWN) { 
        // pressing the down arrow key tells the rover to move backward      
         motor1Speed = motor1Speed =127 - speedbar.getPos();
         motor2Speed = motor2Speed =127 - speedbar.getPos();
         move_pending = true;
         
         move_upButton.deactivate();
         move_downButton.activate();
         move_leftButton.deactivate();
         move_rightButton.deactivate();
  
      } else if (keyCode == LEFT) { 
        // pressing the left arrow key tells the rover to turn left
        motor1Speed = motor1Speed =speedbar.getPos()+127;
        motor2Speed = motor2Speed =127 - speedbar.getPos();
        move_pending = true;
   
        move_upButton.deactivate();
        move_downButton.deactivate();
        move_leftButton.activate();
        move_rightButton.deactivate();
         
      }else if (keyCode == RIGHT) { 
        // pressing the right arrow key tells the rover to turn right   
        motor1Speed = motor1Speed =127 - speedbar.getPos();
        motor2Speed = motor2Speed =speedbar.getPos()+127;
        move_pending = true;
 
        move_upButton.deactivate();
        move_downButton.deactivate();
        move_leftButton.deactivate();
        move_rightButton.activate();
      }
    }
      if (keyCode == 33) { //keyCode 33 corresponds to the Page Up button
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
      if(keydown == false){
        keydown = true;
        if (key == '1') {
          
        } else if (key == '2') { 
          d_upButton.activate();
        } else if (key == '3') { 
          d_outButton.activate();
        } else if (key == '4') {
          b_upButton.activate();
        } else if (key == '5') {
          b_dumpButton.activate();
        } else if (key == '6') {
          
        } else if (key == '7') {
          
        } else if (key == '8') {
          
        } else if (key == '9') {
          
        } else if (key == '0') {
          
        } else if (key == 'A'||key =='a') { 
          d_autoButton.activate();
        } else if (key == 'D'||key =='d') { 
          if(!d_digButton.isActive()){
            d_digButton.activate();
          } else{
            d_stopButton.activate();
          }
        } else if (key == 'E'||key =='e') { 
          d_inButton.activate();
        }else if (key == 'F'||key =='f') { 
            b_autoButton.activate();
        } else if (key == 'Q'||key =='q') { 
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
        } else if (key == 'R'||key =='r') { 
            b_downButton.activate();
        }else if (key == 'S'||key =='s') { 
            d_stopButton.activate();
        }else if (key == 'T'||key =='t') { 
            b_levelButton.activate();
        } else if (key == 'W'||key =='w') { 
          d_downButton.activate();
        } else if (key == ' ') { 
          stopButton.activate();
          
        }
      }
    }
}


//keyReleased activates when a key is released
//similar logic as keyPressed in termes of key and keyCode functionality
void keyReleased(){
  keydown = false;
  if (key == CODED) {
    if (keyCode == UP && move_upButton.isActive()) { 
       
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

    } else if (keyCode == LEFT && move_leftButton.isActive()) { 
         if(connected){
          stop_moveButton.activate();
         }     
         move_leftButton.deactivate();
 
    } else if (keyCode == RIGHT && move_rightButton.isActive()) { 
        if(connected){
          stop_moveButton.activate();
        }
        move_rightButton.deactivate();

    } else if (keyCode == 35) { //keyCode 35 corresponds to the End button
      stop_moveButton.deactivate();
    } else if (keyCode == TAB) { 
    
    }
    
  }  else { //case for alphanumeric keys and symbols
       if (key == '1') {
        
      } else if (key == '2') { 
        d_upButton.deactivate();
      } else if (key == '3') { 
        d_outButton.deactivate();
      } else if (key == '4') {
        b_upButton.deactivate();
      } else if (key == '5') {
        b_dumpButton.deactivate();
      } else if (key == '6') {
        
      } else if (key == '7') {
        
      } else if (key == '8') {
        
      } else if (key == '9') {
        
      } else if (key == '0') {
        
      } else if (key == 'A'||key =='a') { 
        //d_autoButton.deactivate();
      }else if (key == 'D'||key =='d') { 
          if(d_digButton.isActive()){
            //d_digButton.activate();
          } 
          if (d_stopButton.isActive()){
            d_stopButton.deactivate();
          }
        } else if (key == 'E'||key =='e') { 
        d_inButton.deactivate();
      }else if (key == 'F'||key =='f') { 
        //b_autoButton.deactivate();
      } else if (key == 'R'||key =='r') { 
          b_downButton.deactivate();
          if(connected){
            new_debug_message("STOP MOVING hopper");
            byte[] data = new byte[1];
            data[0]=0;
            /*sets up the message to be sent*/
            outMessage.Message(MessageProtocol.ID1_BUCKET,MessageProtocol.ID2_STOP_MOVE,data);
            /*sends message*/
            outMessage.sendMessage();
          }
      }else if (key == 'S'||key =='s') { 
          d_stopButton.deactivate();
      }else if (key == 'T'||key =='t') { 
          b_levelButton.deactivate();
      } else if (key == 'W'||key =='w') { 
        d_downButton.deactivate();
      } else if (key == ' ') { 
        //stopButton.deactivate();
        
      }
    }  
 
  
}
