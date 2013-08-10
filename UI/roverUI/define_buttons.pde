// definition of the button class and all button variables used
// moved to separate tab for ease of changing/checking values and adding buttons

// reset button
Button resetButton = new Button("Reset",10,10,65,40,false,default_color); 

// server connect button
Button connectButton = new Button("Connect",85,10,85,40,false,default_color); 

// server selection buttons
Button test_serverButton = new Button("Test Server",200,16,15,15,false,default_color); 
Button wireless_serverButton = new Button("Wireless Server",200,41,15,15,false,default_color); 
Button tethered_serverButton = new Button("Tethered Server",200,66,15,15,false,default_color); 

// digger control buttons
Button d_upButton = new Button("Up",20,130,65,40,false,default_color); 
Button d_downButton = new Button("Down",20,190,65,40,false,default_color); 
Button d_stopButton = new Button("Stop",105,250,65,40,false,default_color); 
Button d_digButton = new Button("Dig",20,250,65,40,false,default_color); 
Button d_outButton = new Button("Out",105,130,65,40,false,default_color); 
Button d_inButton = new Button("In",105,190,65,40,false,default_color); 
Button d_autoButton = new Button("Auto-Dig",20,310,150,40,false,default_color); 

// bucket control buttons
Button b_upButton = new Button("Up",200,130,65,40,false,default_color); 
Button b_downButton = new Button("Down",200,190,65,40,false,default_color); 
Button b_dumpButton = new Button("Dump",285,130,65,40,false,default_color); 
Button b_levelButton = new Button("Level",285,190,65,40,false,default_color); 
Button b_autoButton = new Button("Auto-Dump",200,310,150,40,false,default_color); 
Button b_stopButton = new Button("Stop",200,250,65,40,false,default_color); 

// video option buttons
Button video_onButton = new Button("On",400,270,65,40,false,default_color); 
Button video_offButton = new Button("Off",485,270,65,40,false,default_color); 
Button video_depthButton = new Button("Depth",570,270,65,40,false,default_color); 
Button video_colorButton = new Button("Color",655,270,65,40,false,default_color); 

// camera movement buttons
Button cam_upButton = new Button("Up",855,95,65,40,false,default_color); 
Button cam_downButton = new Button("Down",855,145,65,40,false,default_color); 
Button cam_leftButton = new Button("Left",780,120,65,40,false,default_color); 
Button cam_rightButton = new Button("Right",930,120,65,40,false,default_color); 

// rover movement buttons
Button move_upButton = new Button("Move Up",855,250,65,40,false,default_color); 
Button move_downButton = new Button("Move Down",855,350,65,40,false,default_color); 
Button move_leftButton = new Button("Move Left",780,300,65,40,false,default_color); 
Button move_rightButton = new Button("Move Right",930,300,65,40,false,default_color); 
Button stop_moveButton = new Button("STOP",855,300,65,40,false,default_color); 
Button enable_moveButton = new Button("Enabled",780,350,65,40,false,default_color); 

// emergency stop button
Button stopButton = new Button("Emergency Stop",395,320,330,50,false,color(255,30,10)); 

// AI enable/disable buttons
Button ai_onButton = new Button("ON",725,460,65,40,false,default_color);
Button ai_offButton = new Button("OFF",810,460,65,40,false,default_color); 

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Definition of button class
// moved to bottom for faster access to defined buttons
class Button
{  
  Button(String label, int x, int y, int w, int h, boolean toggle, color initial_color) // constructor
  {
    myLabel = label;
    myX = x;
    myY = y;
    myWidth = w;
    myHeight = h;
    toggleSwitch = toggle;
    complement = null;
    active = false;
    myColor = initial_color; 
  }
  
  public String getLabel() {return myLabel;} // accessors
  public int getX() {return myX;}
  public int getY() {return myY;}
  public int getWidth() {return myWidth;}
  public int getHeight() {return myHeight;}
  public boolean isToggle() {return toggleSwitch;}
  public Button getOther() {return complement;}
  public boolean isActive() {return active;}
  public boolean isPending() {return pending;}

  public void activate() {active = true; pending = true; if(complement!=null){complement.deactivate();}} // mutators
  public void deactivate() {active = false; pending = false;}
  public void setOther(Button other) {complement = other;}
  public void sent() {pending = false;}
  public void toggle() {active = !active;pending = false;}

  final String myLabel;
  final int myX;
  final int myY;
  final int myWidth;
  final int myHeight;
  final boolean toggleSwitch;
  public color myColor;
  public Button complement;
  boolean active;
  boolean pending;
};
