
// when called, this function draws all of the UI buttons and their labels
// each button can be drawn with the active or default colour, so the function
// checks if each button is active and then draws that button with the necessary color

void drawButtons(){

//creates the boxes that represent each button
  
  //reset
  if(resetButton.isActive()){fill(active_color);}else{fill(default_color);}
  rect(resetButton.myX,resetButton.myY,resetButton.myWidth,resetButton.myHeight);
  
  //server connect
  if(connectButton.isActive()){fill(active_color);}else{fill(default_color);}
  rect(connectButton.myX,connectButton.myY,connectButton.myWidth,connectButton.myHeight);
  
  //server selection buttons
  if(test_serverButton.isActive()){fill(active_color);}else{fill(default_color);}
  ellipse(test_serverButton.myX,test_serverButton.myY,test_serverButton.myWidth,test_serverButton.myHeight);//Note:button is ellipse, not rectangle
  
  if(wireless_serverButton.isActive()){fill(active_color);}else{fill(default_color);}
  ellipse(wireless_serverButton.myX,wireless_serverButton.myY,wireless_serverButton.myWidth,wireless_serverButton.myHeight);//Note:button is ellipse, not rectangle
  
  if(tethered_serverButton.isActive()){fill(active_color);}else{fill(default_color);}
  ellipse(tethered_serverButton.myX,tethered_serverButton.myY,tethered_serverButton.myWidth,tethered_serverButton.myHeight);//Note:button is ellipse, not rectangle
  
  // digging controls
  if(d_upButton.isActive()){fill(active_color);}else{fill(default_color);}
  rect(d_upButton.myX,d_upButton.myY,d_upButton.myWidth,d_upButton.myHeight);
  if(d_downButton.isActive()){fill(active_color);}else{fill(default_color);}
  rect(d_downButton.myX,d_downButton.myY,d_downButton.myWidth,d_downButton.myHeight);
  if(d_stopButton.isActive()){fill(active_color);}else{fill(default_color);}
  rect(d_stopButton.myX,d_stopButton.myY,d_stopButton.myWidth,d_stopButton.myHeight);
  if(d_digButton.isActive()){fill(active_color);}else{fill(default_color);}
  rect(d_digButton.myX,d_digButton.myY,d_digButton.myWidth,d_digButton.myHeight);
  
  if(d_outButton.isActive()){fill(active_color);}else{fill(default_color);}
  rect(d_outButton.myX,d_outButton.myY,d_outButton.myWidth,d_outButton.myHeight);
  if(d_inButton.isActive()){fill(active_color);}else{fill(default_color);}
  rect(d_inButton.myX,d_inButton.myY,d_inButton.myWidth,d_inButton.myHeight);

  //bucket
  if(b_upButton.isActive()){fill(active_color);}else{fill(default_color);}
  rect(b_upButton.myX,b_upButton.myY,b_upButton.myWidth,b_upButton.myHeight);
  if(b_downButton.isActive()){fill(active_color);}else{fill(default_color);}
  rect(b_downButton.myX,b_downButton.myY,b_downButton.myWidth,b_downButton.myHeight);
  if(b_dumpButton.isActive()){fill(active_color);}else{fill(default_color);}
  rect(b_dumpButton.myX,b_dumpButton.myY,b_dumpButton.myWidth,b_dumpButton.myHeight);
  if(b_levelButton.isActive()){fill(active_color);}else{fill(default_color);}
  rect(b_levelButton.myX,b_levelButton.myY,b_levelButton.myWidth,b_levelButton.myHeight);
  
  //video controls
  if(video_onButton.isActive()){fill(active_color);}else{fill(default_color);}
  rect(video_onButton.myX,video_onButton.myY,video_onButton.myWidth,video_onButton.myHeight);
  if(video_offButton.isActive()){fill(active_color);}else{fill(default_color);}
  rect(video_offButton.myX,video_offButton.myY,video_offButton.myWidth,video_offButton.myHeight);
  if(video_depthButton.isActive()){fill(active_color);}else{fill(default_color);}
  rect(video_depthButton.myX,video_depthButton.myY,video_depthButton.myWidth,video_depthButton.myHeight);
  if(video_colorButton.isActive()){fill(active_color);}else{fill(default_color);}
  rect(video_colorButton.myX,video_colorButton.myY,video_colorButton.myWidth,video_colorButton.myHeight);
  
  //camera movement
  if(cam_upButton.isActive()){fill(active_color);}else{fill(default_color);}
  rect(cam_upButton.myX,cam_upButton.myY,cam_upButton.myWidth,cam_upButton.myHeight);
  if(cam_downButton.isActive()){fill(active_color);}else{fill(default_color);}
  rect(cam_downButton.myX,cam_downButton.myY,cam_downButton.myWidth,cam_downButton.myHeight);
  if(cam_leftButton.isActive()){fill(active_color);}else{fill(default_color);}
  rect(cam_leftButton.myX,cam_leftButton.myY,cam_leftButton.myWidth,cam_leftButton.myHeight);
  if(cam_rightButton.isActive()){fill(active_color);}else{fill(default_color);}
  rect(cam_rightButton.myX,cam_rightButton.myY,cam_rightButton.myWidth,cam_rightButton.myHeight);
  
  //rover movement
  if(move_upButton.isActive()){fill(active_color);}else{fill(default_color);}
  rect(move_upButton.myX,move_upButton.myY,move_upButton.myWidth,move_upButton.myHeight);
  if(move_downButton.isActive()){fill(active_color);}else{fill(default_color);}
  rect(move_downButton.myX,move_downButton.myY,move_downButton.myWidth,move_downButton.myHeight);
  if(move_leftButton.isActive()){fill(active_color);}else{fill(default_color);}
  rect(move_leftButton.myX,move_leftButton.myY,move_leftButton.myWidth,move_leftButton.myHeight);
  if(move_rightButton.isActive()){fill(active_color);}else{fill(default_color);}
  rect(move_rightButton.myX,move_rightButton.myY,move_rightButton.myWidth,move_rightButton.myHeight);
  
  if(stop_moveButton.isActive()){fill(active_color);}else{fill(default_color);}
  rect(stop_moveButton.myX,stop_moveButton.myY,stop_moveButton.myWidth,stop_moveButton.myHeight);
  if(enable_moveButton.isActive()){fill(active_color);}else{fill(default_color);}
  rect(enable_moveButton.myX,enable_moveButton.myY,enable_moveButton.myWidth,enable_moveButton.myHeight);
  
  //AI on/off
  if(ai_onButton.isActive()){fill(active_color);}else{fill(default_color);}
  rect(ai_onButton.myX,ai_onButton.myY,ai_onButton.myWidth,ai_onButton.myHeight);
  if(ai_offButton.isActive()){fill(active_color);}else{fill(default_color);}
  rect(ai_offButton.myX,ai_offButton.myY,ai_offButton.myWidth,ai_offButton.myHeight);
 
  //emergency stop button
  if(stopButton.isActive()){fill(0,0,255);}else{fill(stopButton.myColor);}
  rect(stopButton.myX,stopButton.myY,stopButton.myWidth,stopButton.myHeight);
  
//text labels for the buttons
 
  textSize(18);

  fill(0);
  //server selection buttons
  text(test_serverButton.myLabel,test_serverButton.myX+test_serverButton.myWidth+test_serverButton.myLabel.length()*4,test_serverButton.myY-3);
  text(wireless_serverButton.myLabel,wireless_serverButton.myX+wireless_serverButton.myWidth+wireless_serverButton.myLabel.length()*4,wireless_serverButton.myY-3);
  text(tethered_serverButton.myLabel,tethered_serverButton.myX+tethered_serverButton.myWidth+tethered_serverButton.myLabel.length()*4,tethered_serverButton.myY-3);

  fill(255);
  //reset button
  text(resetButton.myLabel,resetButton.myX+resetButton.myWidth/2,resetButton.myY+resetButton.myHeight/2-3);
  
  //server connect/reconnect
  text(connectButton.myLabel,connectButton.myX+connectButton.myWidth/2,connectButton.myY+connectButton.myHeight/2-3);

  //digger
  text(d_upButton.myLabel,d_upButton.myX+d_upButton.myWidth/2,d_upButton.myY+d_upButton.myHeight/2-3);
  text(d_downButton.myLabel,d_downButton.myX+d_downButton.myWidth/2,d_downButton.myY+d_downButton.myHeight/2-3);
  text(d_stopButton.myLabel,d_stopButton.myX+d_stopButton.myWidth/2,d_stopButton.myY+d_stopButton.myHeight/2-3);
  text(d_digButton.myLabel,d_digButton.myX+d_digButton.myWidth/2,d_digButton.myY+d_digButton.myHeight/2-3);
  
  text(d_inButton.myLabel,d_inButton.myX+d_inButton.myWidth/2,d_inButton.myY+d_inButton.myHeight/2-3);
  text(d_outButton.myLabel,d_outButton.myX+d_outButton.myWidth/2,d_outButton.myY+d_outButton.myHeight/2-3);
  
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
  
  //rover movement controls
  text(stop_moveButton.myLabel,stop_moveButton.myX+stop_moveButton.myWidth/2,stop_moveButton.myY+stop_moveButton.myHeight/2-3);
  text(enable_moveButton.myLabel,enable_moveButton.myX+enable_moveButton.myWidth/2,enable_moveButton.myY+enable_moveButton.myHeight/2-3);
  
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


///function for drawing arrows on buttons
//  (or just arrows in general)
//
//code taken from reply #2 of http://processing.org/discourse/beta/num_1219607845.html 
void arrow(int x1, int y1, int x2, int y2) {
  line(x1, y1, x2, y2);
  pushMatrix();
  translate(x2, y2);
  float a = atan2(x1-x2, y2-y1);
  rotate(a);
  line(0, 0, -10, -10);
  line(0, 0, 10, -10);
  popMatrix();
} 


