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
  
  public void activate() {active = true;} // mutators
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
