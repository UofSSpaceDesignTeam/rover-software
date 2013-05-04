void drawButtons(){

//creates the boxes that represent each button
  fill(0,150,25);
  //digger
  rect(d_upButton.myX,d_upButton.myY,d_upButton.myWidth,d_upButton.myHeight);
  rect(d_downButton.myX,d_downButton.myY,d_downButton.myWidth,d_downButton.myHeight);
  rect(d_stopButton.myX,d_stopButton.myY,d_stopButton.myWidth,d_stopButton.myHeight);
  rect(d_digButton.myX,d_digButton.myY,d_digButton.myWidth,d_digButton.myHeight);
  //bucket
  rect(b_upButton.myX,b_upButton.myY,b_upButton.myWidth,b_upButton.myHeight);
  rect(b_downButton.myX,b_downButton.myY,b_downButton.myWidth,b_downButton.myHeight);
  rect(b_dumpButton.myX,b_dumpButton.myY,b_dumpButton.myWidth,b_dumpButton.myHeight);
  rect(b_levelButton.myX,b_levelButton.myY,b_levelButton.myWidth,b_levelButton.myHeight);
  //video
  rect(video_onButton.myX,video_onButton.myY,video_onButton.myWidth,video_onButton.myHeight);
  rect(video_offButton.myX,video_offButton.myY,video_offButton.myWidth,video_offButton.myHeight);
  rect(video_depthButton.myX,video_depthButton.myY,video_depthButton.myWidth,video_depthButton.myHeight);
  rect(video_colorButton.myX,video_colorButton.myY,video_colorButton.myWidth,video_colorButton.myHeight);
  //camera
  rect(cam_upButton.myX,cam_upButton.myY,cam_upButton.myWidth,cam_upButton.myHeight);
  rect(cam_downButton.myX,cam_downButton.myY,cam_downButton.myWidth,cam_downButton.myHeight);
  rect(cam_leftButton.myX,cam_leftButton.myY,cam_leftButton.myWidth,cam_leftButton.myHeight);
  rect(cam_rightButton.myX,cam_rightButton.myY,cam_rightButton.myWidth,cam_rightButton.myHeight);
  //movement
  rect(move_upButton.myX,move_upButton.myY,move_upButton.myWidth,move_upButton.myHeight);
  rect(move_downButton.myX,move_downButton.myY,move_downButton.myWidth,move_downButton.myHeight);
  rect(move_leftButton.myX,move_leftButton.myY,move_leftButton.myWidth,move_leftButton.myHeight);
  rect(move_rightButton.myX,move_rightButton.myY,move_rightButton.myWidth,move_rightButton.myHeight);
  //AI on/off
  rect(ai_onButton.myX,ai_onButton.myY,ai_onButton.myWidth,ai_onButton.myHeight);
  rect(ai_offButton.myX,ai_offButton.myY,ai_offButton.myWidth,ai_offButton.myHeight);
 
  //emergency stop button
  fill(255,30,10);
  rect(stopButton.myX,stopButton.myY,stopButton.myWidth,stopButton.myHeight);
  
//text labels for the buttons
  textSize(18);
  fill(255);
  //digger
  text(d_upButton.myLabel,d_upButton.myX+d_upButton.myWidth/2,d_upButton.myY+d_upButton.myHeight/2-3);
  text(d_downButton.myLabel,d_downButton.myX+d_downButton.myWidth/2,d_downButton.myY+d_downButton.myHeight/2-3);
  text(d_stopButton.myLabel,d_stopButton.myX+d_stopButton.myWidth/2,d_stopButton.myY+d_stopButton.myHeight/2-3);
  text(d_digButton.myLabel,d_digButton.myX+d_digButton.myWidth/2,d_digButton.myY+d_digButton.myHeight/2-3);
  //bucket
  text(b_upButton.myLabel,b_upButton.myX+b_upButton.myWidth/2,b_upButton.myY+b_upButton.myHeight/2-3);
  text(b_downButton.myLabel,b_downButton.myX+b_downButton.myWidth/2,b_downButton.myY+b_downButton.myHeight/2-3);
  text(b_dumpButton.myLabel,b_dumpButton.myX+b_dumpButton.myWidth/2,b_dumpButton.myY+b_dumpButton.myHeight/2-3);
  text(b_levelButton.myLabel,b_levelButton.myX+b_levelButton.myWidth/2,b_levelButton.myY+b_levelButton.myHeight/2-3);
  //video buttons
  text(video_onButton.myLabel,video_onButton.myX+video_onButton.myWidth/2,video_onButton.myY+video_onButton.myHeight/2-3);
  text(video_offButton.myLabel,video_offButton.myX+video_offButton.myWidth/2,video_offButton.myY+video_offButton.myHeight/2-3);
  text(video_depthButton.myLabel,video_depthButton.myX+video_depthButton.myWidth/2,video_depthButton.myY+video_depthButton.myHeight/2-3);
  text(video_colorButton.myLabel,video_colorButton.myX+video_colorButton.myWidth/2,video_colorButton.myY+video_colorButton.myHeight/2-3);
  //camera controls
  text(cam_upButton.myLabel,cam_upButton.myX+cam_upButton.myWidth/2,cam_upButton.myY+cam_upButton.myHeight/2-3);
  text(cam_downButton.myLabel,cam_downButton.myX+cam_downButton.myWidth/2,cam_downButton.myY+cam_downButton.myHeight/2-3);
  text(cam_leftButton.myLabel,cam_leftButton.myX+cam_leftButton.myWidth/2,cam_leftButton.myY+cam_leftButton.myHeight/2-3);
  text(cam_rightButton.myLabel,cam_rightButton.myX+cam_rightButton.myWidth/2,cam_rightButton.myY+cam_rightButton.myHeight/2-3);
  //AI on/off
  text(ai_onButton.myLabel,ai_onButton.myX+ai_onButton.myWidth/2,ai_onButton.myY+ai_onButton.myHeight/2-3);
  text(ai_offButton.myLabel,ai_offButton.myX+ai_offButton.myWidth/2,ai_offButton.myY+ai_offButton.myHeight/2-3);
  //emergency stop
  textSize(24);
  text(stopButton.myLabel,stopButton.myX+stopButton.myWidth/2,stopButton.myY+stopButton.myHeight/2-3);
  
//the following code adds arrows to the movement control buttons
  stroke(255);
  arrow(move_upButton.myX + move_upButton.myWidth/2, move_upButton.myY+move_upButton.myHeight-10,move_upButton.myX + move_upButton.myWidth/2,  move_upButton.myY+10);
  arrow(move_downButton.myX + move_downButton.myWidth/2, move_downButton.myY+10,move_downButton.myX + move_downButton.myWidth/2,  move_downButton.myY+move_downButton.myHeight-10);
  arrow(move_leftButton.myX+move_leftButton.myWidth -10, move_leftButton.myY +move_leftButton.myHeight/2,move_leftButton.myX+10, move_leftButton.myY +move_leftButton.myHeight/2);
  arrow(move_rightButton.myX + 10, move_rightButton.myY+move_rightButton.myHeight/2, move_rightButton.myX + move_rightButton.myWidth -10 ,move_rightButton.myY+move_rightButton.myHeight/2);
  
  
}
