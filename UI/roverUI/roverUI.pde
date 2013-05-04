import processing.net.*;

Client client;
String roverIp = "192.168.1.10";

int eStopX = 300; // information for emergency stop button
int eStopy = 600;
int eStopWidth = 200;
int eStopHeight = 50;

int videoX = 352; // location of video monitor (320x240)
int videoY = 200;

Button upButton = new Button("Up",45,125,65,40,false); 
Button downButton = new Button("Down",45,185,65,40,false); 
Button stopButton = new Button("Stop",45,250,65,40,false); 
Button digButton = new Button("Dig",45,310,65,40,false); 
Button b_upButton = new Button("Up",190,125,65,40,false); 
Button b_downButton = new Button("Down",190,185,65,40,false); 
Button b_dumpButton = new Button("Dump",190,250,65,40,false); 
Button b_levelButton = new Button("Level",190,310,65,40,false); 

Button video_onButton = new Button("On",340,310,65,40,false); 


void setup()
{
  size(1024,600);
  //client = new Client(this,roverIp,81);
}

void draw()
{
  background(230);
  drawButtons();
}


void drawButtons(){
  fill(0,150,25);
  rect(upButton.myX,upButton.myY,upButton.myWidth,upButton.myHeight);
  rect(downButton.myX,downButton.myY,downButton.myWidth,downButton.myHeight);
  rect(stopButton.myX,stopButton.myY,stopButton.myWidth,stopButton.myHeight);
  rect(digButton.myX,digButton.myY,digButton.myWidth,digButton.myHeight);
  rect(b_upButton.myX,b_upButton.myY,b_upButton.myWidth,b_upButton.myHeight);
  rect(b_downButton.myX,b_downButton.myY,b_downButton.myWidth,b_downButton.myHeight);
  rect(b_dumpButton.myX,b_dumpButton.myY,b_dumpButton.myWidth,b_dumpButton.myHeight);
  rect(b_levelButton.myX,b_levelButton.myY,b_levelButton.myWidth,b_levelButton.myHeight);
  rect(video_onButton.myX,video_onButton.myY,video_onButton.myWidth,video_onButton.myHeight);
  
  fill(255);
  text(upButton.myLabel,upButton.myX+25,upButton.myY+20);
  text(downButton.myLabel,downButton.myX+25,downButton.myY+20);

}
