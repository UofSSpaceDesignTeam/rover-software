// definition of the button class and all button variables used
// moved to separate tab for ease of changing/checking values and adding buttons

// reset button
Button resetButton = new Button("Reset",10,10,65,40,false,default_color); 

// server connect button
Button connectButton = new Button("Connect",85,10,85,40,false,default_color); 

// server selection buttons
Button test_serverButton = new Button("Test Server",200,18,15,15,false,default_color); 
Button wireless_serverButton = new Button("Wireless Server",200,43,15,15,false,default_color); 
Button tethered_serverButton = new Button("Tethered Server",200,68,15,15,false,default_color); 

// digger control buttons
Button d_upButton = new Button("Up",45,130,65,40,false,default_color); 
Button d_downButton = new Button("Down",45,190,65,40,false,default_color); 
Button d_stopButton = new Button("Stop",45,250,65,40,false,default_color); 
Button d_digButton = new Button("Dig",45,310,65,40,false,default_color); 
Button d_outButton = new Button("Out",130,130,65,40,false,default_color); 
Button d_inButton = new Button("In",130,190,65,40,false,default_color); 

// bucket control buttons
Button b_upButton = new Button("Up",230,130,65,40,false,default_color); 
Button b_downButton = new Button("Down",230,190,65,40,false,default_color); 
Button b_dumpButton = new Button("Dump",230,250,65,40,false,default_color); 
Button b_levelButton = new Button("Level",230,310,65,40,false,default_color); 

// video option buttons
Button video_onButton = new Button("On",340,310,65,40,false,default_color); 
Button video_offButton = new Button("Off",425,310,65,40,false,default_color); 
Button video_depthButton = new Button("Depth",510,310,65,40,false,default_color); 
Button video_colorButton = new Button("Color",595,310,65,40,false,default_color); 

// camera movement buttons
Button cam_upButton = new Button("Up",810,90,65,40,false,default_color); 
Button cam_downButton = new Button("Down",810,150,65,40,false,default_color); 
Button cam_leftButton = new Button("Left",730,120,65,40,false,default_color); 
Button cam_rightButton = new Button("Right",890,120,65,40,false,default_color); 

// rover movement buttons
Button move_upButton = new Button("Move Up",810,245,65,40,false,default_color); 
Button move_downButton = new Button("Move Down",810,300,65,40,false,default_color); 
Button move_leftButton = new Button("Move Left",730,300,65,40,false,default_color); 
Button move_rightButton = new Button("Move Right",890,300,65,40,false,default_color); 
Button stop_moveButton = new Button("STOP",810,355,65,40,false,default_color); 
Button enable_moveButton = new Button("Enable",730,355,65,40,false,default_color); 

// emergency stop button
Button stopButton = new Button("Emergency Stop",335,360,330,50,false,color(255,30,10)); 

// AI enable/disable buttons
Button ai_onButton = new Button("ON",725,460,65,40,false,default_color);
Button ai_offButton = new Button("OFF",810,460,65,40,false,default_color); 


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
  
  public void activate() {active = true; if(complement!=null){complement.deactivate();}} // mutators
  public void deactivate() {active = false;}
  public void setOther(Button other) {complement = other;}
  
  final String myLabel;
  final int myX;
  final int myY;
  final int myWidth;
  final int myHeight;
  final boolean toggleSwitch;
  public color myColor;
  public Button complement;
  boolean active;
};
