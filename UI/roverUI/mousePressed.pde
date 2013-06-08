void mousePressed() {
//Movement Buttons//  
//Move Up Button (up arrow) 
  if (overButton(move_upButton.myX,move_upButton.myY,move_upButton.myWidth,move_upButton.myHeight)) {
    //move_upButton.myColor = default_color;
    move_upButton.activate();
    move_pending = true;
    motor1Speed = speedbar.getPos()+127;
    motor2Speed = speedbar.getPos()+127;
    
       
  }
  
//Move Down Button (down arrow)  
  if (overButton(move_downButton.myX,move_downButton.myY,move_downButton.myWidth,move_downButton.myHeight)) {
    //move_downButton.myColor = default_color;
    move_downButton.activate();
    motor1Speed = motor1Speed =127 - speedbar.getPos();
    motor2Speed = motor2Speed =127 - speedbar.getPos();
    move_pending = true;

  }
  
//Move/turn Left Button  (left arrow) 
  if (overButton(move_leftButton.myX,move_leftButton.myY,move_leftButton.myWidth,move_leftButton.myHeight)) {
    //move_leftButton.myColor = default_color;
    move_leftButton.activate();
    motor1Speed = motor1Speed =speedbar.getPos()+127;
    motor2Speed = motor2Speed =127 - speedbar.getPos();
    
    move_pending = true;
    
  }
//Move/turn Right Button (right arrow)  
  if (overButton(move_rightButton.myX,move_rightButton.myY,move_rightButton.myWidth,move_rightButton.myHeight)) {
    //move_rightButton.myColor = default_color;
    move_rightButton.activate();
    motor1Speed = motor1Speed =127 - speedbar.getPos();
    motor2Speed = motor2Speed =speedbar.getPos()+127;
    
    move_pending = true;
  }
  
//Stop Movement Button
  if (overButton(stop_moveButton.myX,stop_moveButton.myY,stop_moveButton.myWidth,stop_moveButton.myHeight)) {
    //move_rightButton.myColor = default_color;
    stop_moveButton.activate();
  }
//Motor Enable/Disable Button
  if (overButton(enable_moveButton.myX,enable_moveButton.myY,enable_moveButton.myWidth,enable_moveButton.myHeight)) {
    //move_rightButton.myColor = default_color;
    if (enable_moveButton.isActive()){
      enable_moveButton.deactivate();
    }else{
      enable_moveButton.activate();
    }
  }
  
//Reset button
  if (overButton(resetButton.myX,resetButton.myY,resetButton.myWidth,resetButton.myHeight)) {
    resetButton.activate();
    fill(active_color);
    rect(resetButton.myX,resetButton.myY,resetButton.myWidth,resetButton.myHeight);

    new_debug_message("Reset");
   
    ai_offButton.activate();
    video_offButton.activate();
    video_depthButton.activate();
    enable_moveButton.deactivate();
    move_disable_confirmed = true;
    move_enable_confirmed = false; 
  }
//Connect Button
  if (overButton(connectButton.myX,connectButton.myY,connectButton.myWidth,connectButton.myHeight)) {
    if (wireless_serverButton.isActive()||tethered_serverButton.isActive()||test_serverButton.isActive()){
      if (!connectButton.isActive()){
        if (connected == false){ // sends a connecting message to user on first connection
          new_debug_message("Attempting to Connect to Server...");
        }
        connectButton.activate();
      } else {
        connectButton.deactivate();
      }
    } else {
      new_debug_message("Please select a server");
    }

  }
//Test Server Button
  if (overButton(test_serverButton.myX-7,test_serverButton.myY-8,test_serverButton.myWidth,test_serverButton.myHeight+10)) {
    if(!connected){
      test_serverButton.activate();
      wireless_serverButton.deactivate();
      tethered_serverButton.deactivate();    
    } else {
      new_debug_message("Please disconnect before changing servers");
    }
  }
//Wireless Server Button
  if (overButton(wireless_serverButton.myX-7,wireless_serverButton.myY-8,wireless_serverButton.myWidth,wireless_serverButton.myHeight)) {
    if(!connected){
      wireless_serverButton.activate();
      test_serverButton.deactivate();
      tethered_serverButton.deactivate();    
    } else {
      new_debug_message("Please disconnect before changing servers");
    }
  }
//Tethered Server Button  
  if (overButton(tethered_serverButton.myX-7,tethered_serverButton.myY-8,tethered_serverButton.myWidth,tethered_serverButton.myHeight)) {
    if(!connected){
      tethered_serverButton.activate();
      wireless_serverButton.deactivate();
      test_serverButton.deactivate();   
    } else {
      new_debug_message("Please disconnect before changing servers");
    }
  }
//Digger Up Button  
  if (overButton(d_upButton.myX,d_upButton.myY,d_upButton.myWidth,d_upButton.myHeight)) {
    d_upButton.activate();
  }
//Digger Down Button
  if (overButton(d_downButton.myX,d_downButton.myY,d_downButton.myWidth,d_downButton.myHeight)) {
    //d_downButton.myColor = default_color   ;
    d_downButton.activate();
  }
//Digger Stop Button  
  if (overButton(d_stopButton.myX,d_stopButton.myY,d_stopButton.myWidth,d_stopButton.myHeight)) {
    //d_stopButton.myColor = default_color;
    d_stopButton.activate();
  }
//Digger Dig Button  
  if (overButton(d_digButton.myX,d_digButton.myY,d_digButton.myWidth,d_stopButton.myHeight)) {
    //d_digButton.myColor = default_color;
    d_digButton.activate();
  }
//Digger Out Button  
  if (overButton(d_outButton.myX,d_outButton.myY,d_outButton.myWidth,d_outButton.myHeight)) {
    //d_outButton.myColor = default_color;
    d_outButton.activate();
  }
//Digger In Button  
  if (overButton(d_inButton.myX,d_inButton.myY,d_inButton.myWidth,d_inButton.myHeight)) {
    //d_inButton.myColor = default_color;
    d_inButton.activate();
  }
//Bucket Up Button
  if (overButton(b_upButton.myX,b_upButton.myY,b_upButton.myWidth,b_upButton.myHeight)) {
    //b_upButton.myColor = default_color;
    b_upButton.activate();
  }
//Bucket Down Button  
  if (overButton(b_downButton.myX,b_downButton.myY,b_downButton.myWidth,b_downButton.myHeight)) {
    //b_downButton.myColor = default_color;
    b_downButton.activate();
  }
//Bucket Dump Button  
  if (overButton(b_dumpButton.myX,b_dumpButton.myY,b_dumpButton.myWidth,b_dumpButton.myHeight)) {
    //b_dumpButton.myColor = default_color;
    b_dumpButton.activate();
  }
//Bucket Level Button  
  if (overButton(b_levelButton.myX,b_levelButton.myY,b_levelButton.myWidth,b_levelButton.myHeight)) {
    //b_levelButton.myColor = default_color;
    b_levelButton.activate();
  }

//Video On Button
  if (overButton(video_onButton.myX,video_onButton.myY,video_onButton.myWidth,video_onButton.myHeight)) {
    //video_onButton.myColor = default_color;
    video_onButton.activate();
  }
//Video Off Button  
  if (overButton(video_offButton.myX,video_offButton.myY,video_offButton.myWidth,video_offButton.myHeight)) {
    //video_offButton.myColor = default_color;\
    video_offButton.activate();
    
  }
//Video Depthmap Button  
  if (overButton(video_depthButton.myX,video_depthButton.myY,video_depthButton.myWidth,video_depthButton.myHeight)) {
    //video_depthButton.myColor = default_color;
    video_depthButton.activate();
  }
//Video Color Button  
  if (overButton(video_colorButton.myX,video_colorButton.myY,video_colorButton.myWidth,video_colorButton.myHeight)) {
    //video_colorButton.myColor = default_color;
    video_colorButton.activate();
  }

//Camera Up Button
  if (overButton(cam_upButton.myX,cam_upButton.myY,cam_upButton.myWidth,cam_upButton.myHeight)) {
    //cam_upButton.myColor = default_color;
    cam_upButton.activate();
  }
//Camera Down Button  
  if (overButton(cam_downButton.myX,cam_downButton.myY,cam_downButton.myWidth,cam_downButton.myHeight)) {
    //cam_downButton.myColor = default_color;
    cam_downButton.activate();
  }
//Camera Left Button  
  if (overButton(cam_leftButton.myX,cam_leftButton.myY,cam_leftButton.myWidth,cam_leftButton.myHeight)) {
    //cam_leftButton.myColor = default_color;
    cam_leftButton.activate();
  }
//Camera Right Button  
  if (overButton(cam_rightButton.myX,cam_rightButton.myY,cam_rightButton.myWidth,cam_rightButton.myHeight)) {
    //cam_rightButton.myColor = default_color;
    cam_rightButton.activate();
  }
 
//AI On Button
  if (overButton(ai_onButton.myX,ai_onButton.myY,ai_onButton.myWidth,ai_onButton.myHeight)) {
    //ai_onButton.myColor = default_color;
    ai_onButton.activate();
    //ai_onButton.complement.deactivate();
  }
//AI Off Button  
  if (overButton(ai_offButton.myX,ai_offButton.myY,ai_offButton.myWidth,ai_offButton.myHeight)) {
    //ai_offButton.myColor = default_color;
    ai_offButton.activate();
    //ai_offButton.complement.deactivate();
  }
 
//Emergency Stop Button  
  if (overButton(stopButton.myX,stopButton.myY,stopButton.myWidth,stopButton.myHeight)) {
    //stopButton.myColor = color(255,30,20);
    stopButton.activate();
    
  }

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////function separator (because i'm blind and always miss the end  of the mousePressed function)////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Activates when the mouse button is released
void mouseReleased() {
  if (move_upButton.isActive() || move_downButton.isActive() || move_leftButton.isActive() || move_rightButton.isActive()){
    if(connected){
      new_debug_message("STOP");
      move_upButton.deactivate();
      move_downButton.deactivate();
      move_leftButton.deactivate();
      move_rightButton.deactivate();
//      speed[0] = byte(127);
//      speed[1] = byte(127);
//      outMessage.Message(MessageProtocol.ID1_MOTORS,MessageProtocol.ID2_ROTATION,speed);
//      outMessage.sendMessage();
      if(connected){
        stop_moveButton.activate();
      }
    }
  }
  if (overButton(d_upButton.myX,d_upButton.myY,d_upButton.myWidth,d_upButton.myHeight)) {
    //d_upButton.myColor = default_color;
    //d_upButton.deactivate();
  }
  if (overButton(d_downButton.myX,d_downButton.myY,d_downButton.myWidth,d_downButton.myHeight)) {
    //d_downButton.myColor = default_color   ;
    //d_downButton.deactivate();
  }
  if (overButton(d_stopButton.myX,d_stopButton.myY,d_stopButton.myWidth,d_stopButton.myHeight)) {
    //d_stopButton.myColor = default_color;
    d_stopButton.deactivate();
  }
  if (overButton(d_digButton.myX,d_digButton.myY,d_digButton.myWidth,d_stopButton.myHeight)) {
    //d_digButton.myColor = default_color;
    d_digButton.deactivate();
  }
  
  if (overButton(d_outButton.myX,d_outButton.myY,d_outButton.myWidth,d_outButton.myHeight)) {
    //d_outButton.myColor = default_color;
    d_outButton.deactivate();
  }
  if (overButton(d_inButton.myX,d_inButton.myY,d_inButton.myWidth,d_inButton.myHeight)) {
    //d_inButton.myColor = default_color;
    d_inButton.deactivate();
  }

  if (overButton(b_upButton.myX,b_upButton.myY,b_upButton.myWidth,b_upButton.myHeight)) {
    //b_upButton.myColor = default_color;
    b_upButton.deactivate();
  }
  if (overButton(b_downButton.myX,b_downButton.myY,b_downButton.myWidth,b_downButton.myHeight)) {
    //b_downButton.myColor = default_color;
    b_downButton.deactivate();
  }
  if (overButton(b_dumpButton.myX,b_dumpButton.myY,b_dumpButton.myWidth,b_dumpButton.myHeight)) {
    //b_dumpButton.myColor = default_color;
    b_dumpButton.deactivate();
  }
  if (overButton(b_levelButton.myX,b_levelButton.myY,b_levelButton.myWidth,b_levelButton.myHeight)) {
    //b_levelButton.myColor = default_color;
    b_levelButton.deactivate();
  }


  if (overButton(video_onButton.myX,video_onButton.myY,video_onButton.myWidth,video_onButton.myHeight)) {
    //video_onButton.myColor = default_color;
    //video_onButton.deactivate();
  }
  if (overButton(video_offButton.myX,video_offButton.myY,video_offButton.myWidth,video_offButton.myHeight)) {
    //video_offButton.myColor = default_color;\
    //video_offButton.deactivate();
  }
  if (overButton(video_depthButton.myX,video_depthButton.myY,video_depthButton.myWidth,video_depthButton.myHeight)) {
    //video_depthButton.myColor = default_color;
    //video_depthButton.deactivate();
  }
  if (overButton(video_colorButton.myX,video_colorButton.myY,video_colorButton.myWidth,video_colorButton.myHeight)) {
    //video_colorButton.myColor = default_color;
    //video_colorButton.deactivate();
  }


  if (overButton(cam_upButton.myX,cam_upButton.myY,cam_upButton.myWidth,cam_upButton.myHeight)) {
    //cam_upButton.myColor = default_color;
    cam_upButton.deactivate();
  }
  if (overButton(cam_downButton.myX,cam_downButton.myY,cam_downButton.myWidth,cam_downButton.myHeight)) {
    //cam_downButton.myColor = default_color;
    cam_downButton.deactivate();
  }
  if (overButton(cam_leftButton.myX,cam_leftButton.myY,cam_leftButton.myWidth,cam_leftButton.myHeight)) {
    //cam_leftButton.myColor = default_color;
    cam_leftButton.deactivate();
  }
  if (overButton(cam_rightButton.myX,cam_rightButton.myY,cam_rightButton.myWidth,cam_rightButton.myHeight)) {
    //cam_rightButton.myColor = default_color;
    cam_rightButton.deactivate();
  }
 
  if (overButton(move_upButton.myX,move_upButton.myY,move_upButton.myWidth,move_upButton.myHeight)) {
    //move_upButton.myColor = default_color;
    move_upButton.deactivate();

  }
  if (overButton(move_downButton.myX,move_downButton.myY,move_downButton.myWidth,move_downButton.myHeight)) {
    //move_downButton.myColor = default_color;
    move_downButton.deactivate();
  }
  if (overButton(move_leftButton.myX,move_leftButton.myY,move_leftButton.myWidth,move_leftButton.myHeight)) {
    //move_leftButton.myColor = default_color;
    move_leftButton.deactivate();
  }
  if (overButton(move_rightButton.myX,move_rightButton.myY,move_rightButton.myWidth,move_rightButton.myHeight)) {
    //move_rightButton.myColor = default_color;
    move_rightButton.deactivate();
  }
 
 if (overButton(stop_moveButton.myX,stop_moveButton.myY,stop_moveButton.myWidth,stop_moveButton.myHeight)) {
    //move_rightButton.myColor = default_color;
    stop_moveButton.deactivate();
  }
 
  if (overButton(ai_onButton.myX,ai_onButton.myY,ai_onButton.myWidth,ai_onButton.myHeight)) {
    //ai_onButton.myColor = default_color;
    //ai_onButton.deactivate();
  }
  if (overButton(ai_offButton.myX,ai_offButton.myY,ai_offButton.myWidth,ai_offButton.myHeight)) {
    //ai_offButton.deactivate();
  }
  if (overButton(stopButton.myX,stopButton.myY,stopButton.myWidth,stopButton.myHeight)) {
    
    stopButton.deactivate();
  }
  if (overButton(resetButton.myX,resetButton.myY,resetButton.myWidth,resetButton.myHeight)) {
    
    resetButton.deactivate();
   
  }
  if (overButton(connectButton.myX,connectButton.myY,connectButton.myWidth,connectButton.myHeight)) {
    
    //connectButton.deactivate();
   
  }
}

boolean overButton(int x, int y, int width, int height)  {
  if (mouseX >= x && mouseX <= x+width && 
      mouseY >= y && mouseY <= y+height) {
    return true;
  } else {
    return false;
  }
}
