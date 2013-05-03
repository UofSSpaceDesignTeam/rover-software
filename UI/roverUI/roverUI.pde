import processing.net.*;
import Button;

Client client;
String roverIp = "192.168.1.10";

int eStopX = 300; // information for emergency stop button
int eStopy = 600;
int eStopWidth = 200;
int eStopHeight = 50;

int videoX = 352; // location of video monitor (320x240)
int videoY = 200;


void setup()
{
  size(1024,600);
  client = new Client(this,roverIp,81);
}

void draw()
{
  background(230);
  drawButtons();
}

