import processing.net.*;

Client client;
//String roverIp = "192.168.1.10";
String roverIp = "10.227.5.214";
int comm_port = 7050;
int eStopX = 300; // information for emergency stop button
int eStopy = 600;
int eStopWidth = 200;
int eStopHeight = 50;

int videoX = 352; // location of video monitor (320x240)
int videoY = 200;
color default_color = color(0,100,15);
color active_color = color(30,220,40);
int motor1Speed = 0;
int motor2Speed = 0;
//Button[] allButtons = new Button[20]; //just an idea i had, please ignore for now


//ai_onButton.activate();
//ai_offButton.setOther(ai_onButton);


void setup()
{
  size(1024,600);
  //try{
    client = new Client(this, roverIp, comm_port);
  //}catch(IOException e){e.printStackTrace();}
  ai_offButton.activate();
  video_offButton.activate();
  video_depthButton.activate();
  d_upButton.setOther(d_downButton);
  d_downButton.setOther(d_upButton);
  d_outButton.setOther(d_inButton);
  d_inButton.setOther(d_outButton);
  d_digButton.setOther(d_stopButton);
  d_stopButton.setOther(d_digButton);
  
  b_upButton.setOther(b_downButton);
  b_downButton.setOther(b_upButton);
  b_dumpButton.setOther(b_levelButton);
  b_levelButton.setOther(b_dumpButton);
  
  ai_offButton.setOther(ai_onButton);
  ai_onButton.setOther(ai_offButton);
  
  video_onButton.setOther(video_offButton);
  video_offButton.setOther(video_onButton);
  video_depthButton.setOther(video_colorButton);
  video_colorButton.setOther(video_depthButton); 
}

void draw()
{
  textAlign(CENTER,CENTER);
  
  background(230);
  fill(210);
  stroke(0);
  strokeWeight(2);
  //rect(35,120,85,240);  //}box around digger controls
 // rect(35,120,170,120); //}
  
  //rect(220,120,85,240);  //box arount bucket controls
  //rect(715,80,255,120); //box around camera controls
  //rect(715,235,255,120); //box around movement controls
  //rect(715,450,300,140);//box around AI Status
  rect(330,75,videoX,videoY);//Video box
  rect(10,420,405,170);//message box
  
  fill(0);
  textSize(24);
  text("Excavation",160,70);
  text("Camera",845,65);
  text("Movement",845,220);
  text("Video", 500,30);
  text("Messages",200,400);
  
  textSize(16);
  text("Digger",120,100);
  text("Bucket",265,100);
  text("AI Status",870,440);
  textSize(15);
  text("Arrow Keys",925,330);
  drawButtons();
  getMessages();
  sendMessages();
}



