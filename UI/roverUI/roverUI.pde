import processing.net.*;

Client client;
String roverIp = "192.168.1.10";

int eStopX = 300; // information for emergency stop button
int eStopy = 600;
int eStopWidth = 200;
int eStopHeight = 50;

int videoX = 352; // location of video monitor (320x240)
int videoY = 200;
color default_color = color(0,150,25);
//b = bucket, d = digger
Button d_upButton = new Button("Up",45,130,65,40,false,default_color); 
Button d_downButton = new Button("Down",45,190,65,40,false,default_color); 
Button d_stopButton = new Button("Stop",45,250,65,40,false,default_color); 
Button d_digButton = new Button("Dig",45,310,65,40,false,default_color); 

Button b_upButton = new Button("Up",190,130,65,40,false,default_color); 
Button b_downButton = new Button("Down",190,190,65,40,false,default_color); 
Button b_dumpButton = new Button("Dump",190,250,65,40,false,default_color); 
Button b_levelButton = new Button("Level",190,310,65,40,false,default_color); 

Button video_onButton = new Button("On",340,310,65,40,false,default_color); 
Button video_offButton = new Button("Off",425,310,65,40,false,default_color); 
Button video_depthButton = new Button("Depth",510,310,65,40,false,default_color); 
Button video_colorButton = new Button("Color",595,310,65,40,false,default_color); 

Button cam_upButton = new Button("Up",810,90,65,40,false,default_color); 
Button cam_downButton = new Button("Down",810,150,65,40,false,default_color); 
Button cam_leftButton = new Button("Left",730,120,65,40,false,default_color); 
Button cam_rightButton = new Button("Right",890,120,65,40,false,default_color); 

Button move_upButton = new Button("Move Up",810,245,65,40,false,default_color); 
Button move_downButton = new Button("Move Down",810,305,65,40,false,default_color); 
Button move_leftButton = new Button("Move Left",730,275,65,40,false,default_color); 
Button move_rightButton = new Button("Move Right",890,275,65,40,false,default_color); 

Button stopButton = new Button("Emergency Stop",335,360,330,50,false,color(255,30,10)); 

Button ai_onButton = new Button("ON",725,460,65,40,false,default_color);
Button ai_offButton = new Button("OFF",810,460,65,40,false,default_color); 
//ai_onButton.activate();
//ai_offButton.setOther(ai_onButton);


void setup()
{
  size(1024,600);
  ai_offButton.setOther(ai_onButton);
  ai_onButton.setOther(ai_offButton);
  video_onButton.setOther(video_offButton);
  video_offButton.setOther(video_onButton);
 video_depthButton.setOther(video_colorButton);
 video_colorButton.setOther(video_depthButton); 
  //client = new Client(this,roverIp,81);
}

void draw()
{
  textAlign(CENTER,CENTER);

  background(230);
  fill(210);
  stroke(0);
  strokeWeight(2);
  rect(35,120,85,240);  //box around digger controls
  rect(180,120,85,240);  //box arount bucket controls
  rect(715,80,255,120); //box around camera controls
  rect(715,235,255,120); //box around movement controls
  rect(715,450,300,140);//box around AI Status
  rect(335,55,330,250);//Video box
  rect(10,420,405,170);//message box
  
  fill(0);
  textSize(24);
  text("Excavation",160,70);
  text("Camera",845,65);
  text("Movement",845,220);
  text("Video", 500,30);
  text("Messages",200,400);
  
  textSize(16);
  text("Digger",80,100);
  text("Bucket",220,100);
  text("AI Status",870,440);
  textSize(15);
  text("Arrow Keys",925,330);
  drawButtons();
}



