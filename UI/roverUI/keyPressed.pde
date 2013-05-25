
// activates every time a keyboard button is pressed down
// does action corresponding to which button was pressed
void keyPressed() {
  int scrollAdjust = 5; //increment at which scrollbar moves with each button press of PAGE UP or PAGE DOWN 
  if (key == CODED) {// special case for specail keys: UP, DOWN, LEFT, RIGHT, arrow keys, ALT, CONTROL, SHIFT
                      // does not include BACKSPACE, TAB, ENTER, RETURN, ESC, and DELETE [ie: can use if(key == TAB)]
    if (keyCode == UP) {
      // pressing the up arrow key tells the rover to move forward
      motor1Speed = speedbar.getPos()+127;
      motor2Speed = speedbar.getPos()+127; 
      move_upButton.activate();
          
    } else if (keyCode == DOWN) { 
      // pressing the down arrow key tells the rover to move backward      
       motor1Speed = motor1Speed =127 - speedbar.getPos();
       motor2Speed = motor2Speed =127 - speedbar.getPos();
       move_downButton.activate();

    } else if (keyCode == LEFT) { 
      // pressing the left arrow key tells the rover to turn left
       motor1Speed = motor1Speed =speedbar.getPos()+127;
       motor2Speed = motor2Speed =127 - speedbar.getPos();
       move_leftButton.activate();
       
    }else if (keyCode == RIGHT) { 
      // pressing the right arrow key tells the rover to turn right   
       motor1Speed = motor1Speed =127 - speedbar.getPos();
       motor2Speed = motor2Speed =speedbar.getPos()+127;
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
    }
    
  } else {
    if (key == 'A') { //case for alphanumeric keys and symbols
   
    } else { 

    }
  }
  
}




void keyReleased(){
  //println("key: "+key);
  //println("keyCode: "+keyCode);
  
  if (key == CODED) {
    if (keyCode == UP) {
       move_upButton.deactivate();
    } else if (keyCode == DOWN) { 
       move_downButton.deactivate();
    } else if (keyCode == LEFT) { 
       move_leftButton.deactivate();
    }else if (keyCode == RIGHT) { 
       move_rightButton.deactivate();
    }
    
  } else {
    if (key == 'A') {
   
    } else { 

    }
  }  
  
  
  
}
