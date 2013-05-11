void mousePressed() {

  if (overButton(resetButton.myX,resetButton.myY,resetButton.myWidth,resetButton.myHeight)) {
    //ai_offButton.myColor = default_color;
    resetButton.activate();
    client = new Client(this, roverIp, comm_port);
    ai_offButton.activate();
    video_offButton.activate();
    video_depthButton.activate();
    enable_moveButton.deactivate();
    move_disable_confirmed = true;
    move_enable_confirmed = false;
  }
  
    if (overButton(d_upButton.myX,d_upButton.myY,d_upButton.myWidth,d_upButton.myHeight)) {
    //d_upButton.myColor = default_color;
    d_upButton.activate();
  }
  if (overButton(d_downButton.myX,d_downButton.myY,d_downButton.myWidth,d_downButton.myHeight)) {
    //d_downButton.myColor = default_color   ;
    d_downButton.activate();
  }
  if (overButton(d_stopButton.myX,d_stopButton.myY,d_stopButton.myWidth,d_stopButton.myHeight)) {
    //d_stopButton.myColor = default_color;
    d_stopButton.activate();
  }
  if (overButton(d_digButton.myX,d_digButton.myY,d_digButton.myWidth,d_stopButton.myHeight)) {
    //d_digButton.myColor = default_color;
    d_digButton.activate();
  }
  
  if (overButton(d_outButton.myX,d_outButton.myY,d_outButton.myWidth,d_outButton.myHeight)) {
    //d_outButton.myColor = default_color;
    d_outButton.activate();
  }
  if (overButton(d_inButton.myX,d_inButton.myY,d_inButton.myWidth,d_inButton.myHeight)) {
    //d_inButton.myColor = default_color;
    d_inButton.activate();
  }

  if (overButton(b_upButton.myX,b_upButton.myY,b_upButton.myWidth,b_upButton.myHeight)) {
    //b_upButton.myColor = default_color;
    b_upButton.activate();
  }
  if (overButton(b_downButton.myX,b_downButton.myY,b_downButton.myWidth,b_downButton.myHeight)) {
    //b_downButton.myColor = default_color;
    b_downButton.activate();
  }
  if (overButton(b_dumpButton.myX,b_dumpButton.myY,b_dumpButton.myWidth,b_dumpButton.myHeight)) {
    //b_dumpButton.myColor = default_color;
    b_dumpButton.activate();
  }
  if (overButton(b_levelButton.myX,b_levelButton.myY,b_levelButton.myWidth,b_levelButton.myHeight)) {
    //b_levelButton.myColor = default_color;
    b_levelButton.activate();
  }


  if (overButton(video_onButton.myX,video_onButton.myY,video_onButton.myWidth,video_onButton.myHeight)) {
    //video_onButton.myColor = default_color;
    video_onButton.activate();
    video_depthButton.activate();
  }
  if (overButton(video_offButton.myX,video_offButton.myY,video_offButton.myWidth,video_offButton.myHeight)) {
    //video_offButton.myColor = default_color;\
    video_offButton.activate();
    
  }
  if (overButton(video_depthButton.myX,video_depthButton.myY,video_depthButton.myWidth,video_depthButton.myHeight)) {
    //video_depthButton.myColor = default_color;
    video_depthButton.activate();
  }
  if (overButton(video_colorButton.myX,video_colorButton.myY,video_colorButton.myWidth,video_colorButton.myHeight)) {
    //video_colorButton.myColor = default_color;
    video_colorButton.activate();
  }


  if (overButton(cam_upButton.myX,cam_upButton.myY,cam_upButton.myWidth,cam_upButton.myHeight)) {
    //cam_upButton.myColor = default_color;
    cam_upButton.activate();
  }
  if (overButton(cam_downButton.myX,cam_downButton.myY,cam_downButton.myWidth,cam_downButton.myHeight)) {
    //cam_downButton.myColor = default_color;
    cam_downButton.activate();
  }
  if (overButton(cam_leftButton.myX,cam_leftButton.myY,cam_leftButton.myWidth,cam_leftButton.myHeight)) {
    //cam_leftButton.myColor = default_color;
    cam_leftButton.activate();
  }
  if (overButton(cam_rightButton.myX,cam_rightButton.myY,cam_rightButton.myWidth,cam_rightButton.myHeight)) {
    //cam_rightButton.myColor = default_color;
    cam_rightButton.activate();
  }
 
  if (overButton(move_upButton.myX,move_upButton.myY,move_upButton.myWidth,move_upButton.myHeight)) {
    //move_upButton.myColor = default_color;
    move_upButton.activate();
    motor1Speed = 200;
    motor2Speed = 200;
    
  }
  if (overButton(move_downButton.myX,move_downButton.myY,move_downButton.myWidth,move_downButton.myHeight)) {
    //move_downButton.myColor = default_color;
    move_downButton.activate();
    motor1Speed = motor1Speed =50;
    motor2Speed = motor2Speed =50;
  }
  if (overButton(move_leftButton.myX,move_leftButton.myY,move_leftButton.myWidth,move_leftButton.myHeight)) {
    //move_leftButton.myColor = default_color;
    move_leftButton.activate();
    motor1Speed = motor1Speed =200;
    motor2Speed = motor2Speed =50;
  }
  if (overButton(move_rightButton.myX,move_rightButton.myY,move_rightButton.myWidth,move_rightButton.myHeight)) {
    //move_rightButton.myColor = default_color;
    move_rightButton.activate();
    motor1Speed = motor1Speed =50;
    motor2Speed = motor2Speed =20;
  }
  if (overButton(stop_moveButton.myX,stop_moveButton.myY,stop_moveButton.myWidth,stop_moveButton.myHeight)) {
    //move_rightButton.myColor = default_color;
    stop_moveButton.activate();
  }
  if (overButton(enable_moveButton.myX,enable_moveButton.myY,enable_moveButton.myWidth,enable_moveButton.myHeight)) {
    //move_rightButton.myColor = default_color;
    if (enable_moveButton.isActive()){
      enable_moveButton.deactivate();
    }else{
      enable_moveButton.activate();
    }
  }
 
 
  if (overButton(ai_onButton.myX,ai_onButton.myY,ai_onButton.myWidth,ai_onButton.myHeight)) {
    //ai_onButton.myColor = default_color;
    ai_onButton.activate();
    //ai_onButton.complement.deactivate();
  }
  if (overButton(ai_offButton.myX,ai_offButton.myY,ai_offButton.myWidth,ai_offButton.myHeight)) {
    //ai_offButton.myColor = default_color;
    ai_offButton.activate();
    //ai_offButton.complement.deactivate();
  }
 
  
  
  if (overButton(stopButton.myX,stopButton.myY,stopButton.myWidth,stopButton.myHeight)) {
    //stopButton.myColor = color(255,30,20);
    stopButton.activate();
    
  }
//  if (overButton(d_upButton.myX,d_upButton.myY,d_upButton.myWidth,d_upButton.myHeight)) {
//    //d_upButton.myColor = color(255,0,0);
//    d_upButton.activate();
//  }
//  if (overButton(d_downButton.myX,d_downButton.myY,d_downButton.myWidth,d_downButton.myHeight)) {
//    d_downButton.myColor = color(255,0,0);
//  }
//  if (overButton(d_stopButton.myX,d_stopButton.myY,d_stopButton.myWidth,d_stopButton.myHeight)) {
//    d_stopButton.myColor = color(255,0,0);
//  }
//  if (overButton(d_digButton.myX,d_digButton.myY,d_digButton.myWidth,d_stopButton.myHeight)) {
//    d_digButton.myColor = color(255,0,0);
//  }
//  
//  if (overButton(d_inButton.myX,d_inButton.myY,d_inButton.myWidth,d_inButton.myHeight)) {
//    d_inButton.myColor = color(255,0,0);
//  }
//  if (overButton(d_outButton.myX,d_outButton.myY,d_outButton.myWidth,d_outButton.myHeight)) {
//    d_outButton.myColor = color(255,0,0);
//  }  
//
//  if (overButton(b_upButton.myX,b_upButton.myY,b_upButton.myWidth,b_upButton.myHeight)) {
//    b_upButton.myColor = color(255,0,0);
//  }
//  if (overButton(b_downButton.myX,b_downButton.myY,b_downButton.myWidth,b_downButton.myHeight)) {
//    b_downButton.myColor = color(255,0,0);
//  }
//  if (overButton(b_dumpButton.myX,b_dumpButton.myY,b_dumpButton.myWidth,b_dumpButton.myHeight)) {
//    b_dumpButton.myColor = color(255,0,0);
//  }
//  if (overButton(b_levelButton.myX,b_levelButton.myY,b_levelButton.myWidth,b_levelButton.myHeight)) {
//    b_levelButton.myColor = color(255,0,0);
//  }
//
//
//  if (overButton(video_onButton.myX,video_onButton.myY,video_onButton.myWidth,video_onButton.myHeight)) {
//    video_onButton.myColor = color(255,0,0);
//  }
//  if (overButton(video_offButton.myX,video_offButton.myY,video_offButton.myWidth,video_offButton.myHeight)) {
//    video_offButton.myColor = color(255,0,0);
//  }
//  if (overButton(video_depthButton.myX,video_depthButton.myY,video_depthButton.myWidth,video_depthButton.myHeight)) {
//    video_depthButton.myColor = color(255,0,0);
//  }
//  if (overButton(video_colorButton.myX,video_colorButton.myY,video_colorButton.myWidth,video_colorButton.myHeight)) {
//    video_colorButton.myColor = color(255,0,0);
//  }
//
//
//  if (overButton(cam_upButton.myX,cam_upButton.myY,cam_upButton.myWidth,cam_upButton.myHeight)) {
//    cam_upButton.myColor = color(255,0,0);
//  }
//  if (overButton(cam_downButton.myX,cam_downButton.myY,cam_downButton.myWidth,cam_downButton.myHeight)) {
//    cam_downButton.myColor = color(255,0,0);
//  }
//  if (overButton(cam_leftButton.myX,cam_leftButton.myY,cam_leftButton.myWidth,cam_leftButton.myHeight)) {
//    cam_leftButton.myColor = color(255,0,0);
//  }
//  if (overButton(cam_rightButton.myX,cam_rightButton.myY,cam_rightButton.myWidth,cam_rightButton.myHeight)) {
//    cam_rightButton.myColor = color(255,0,0);
//  }
//  
//  if (overButton(move_upButton.myX,move_upButton.myY,move_upButton.myWidth,move_upButton.myHeight)) {
//    move_upButton.myColor = color(255,0,0);
//  }
//  if (overButton(move_downButton.myX,move_downButton.myY,move_downButton.myWidth,move_downButton.myHeight)) {
//    move_downButton.myColor = color(255,0,0);
//  }
//  if (overButton(move_leftButton.myX,move_leftButton.myY,move_leftButton.myWidth,move_leftButton.myHeight)) {
//    move_leftButton.myColor = color(255,0,0);
//  }
//  if (overButton(move_rightButton.myX,move_rightButton.myY,move_rightButton.myWidth,move_rightButton.myHeight)) {
//    move_rightButton.myColor = color(255,0,0);
//  }
//  
//  if (overButton(ai_onButton.myX,ai_onButton.myY,ai_onButton.myWidth,ai_onButton.myHeight)) {
//    ai_onButton.myColor = color(255,0,0);
//  }
//  if (overButton(ai_offButton.myX,ai_offButton.myY,ai_offButton.myWidth,ai_offButton.myHeight)) {
//    ai_offButton.myColor = color(255,0,0);
//  }
//  if (overButton(stopButton.myX,stopButton.myY,stopButton.myWidth,stopButton.myHeight)) {
//    stopButton.myColor = color(0,0,255);
//  }
// 
// 
}

void mouseReleased() {
  
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
    //ai_offButton.myColor = default_color;
    //ai_offButton.deactivate();
  }
  if (overButton(stopButton.myX,stopButton.myY,stopButton.myWidth,stopButton.myHeight)) {
    //stopButton.myColor = color(255,30,20);
    stopButton.deactivate();
  }
  if (overButton(resetButton.myX,resetButton.myY,resetButton.myWidth,resetButton.myHeight)) {
    //ai_offButton.myColor = default_color;
    resetButton.deactivate();
    //ai_offButton.complement.deactivate();
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
