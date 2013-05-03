class Button
{  
  Button(String label, int x, int y, int w, int h, boolean toggle) // constructor
  {
    myLabel = label;
    myX = x;
    myY = y;
    myWidth = w;
    myHeight = h;
    toggleSwitch = toggle;
    complement = null;
    active = false;
  }
  
  public String getLabel() {return myLabel;} // accessors
  public int getX() {return myX;}
  public int getY() {return myY;}
  public int getWidth() {return myWidth;}
  public int getHeight() {return myHeight;}
  public boolean isToggle() {return toggleSwitch;}
  public button getOther() {return complement;}
  public boolean isActive() {return active;}
  
  public void activate() {active = true;} // mutators
  public void deactivate() {active = false;}
  public void setOther(button other) {complement = other;}
  
  final String myLabel;
  final int myX;
  final int myY;
  final int myWidth;
  final int myHeight;
  final boolean toggleSwitch;
  button complement;
  boolean active;
};