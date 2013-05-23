import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import processing.net.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class rover_test_server extends PApplet {


Server myServer;
byte[] inmessage;
byte[] outmessage;
int noteX = 5;
int noteY = 15;
int count = 0;
byte[] lastin;

public void setup(){
 size(400,400);
 background(00,195,50);
 myServer = new Server(this, 7050);
}

//void draw() {
//  Client getclient = myServer.available();
//  if(getclient != null){
//    inmessage = getclient.readStringUntil('\n');
//    if(inmessage!= null){
//     count++;
//     text("Client says: " + inmessage,noteX,noteY);
//     noteY = noteY +15;
//    }
//    myServer.write("gotcha "+count+"\n");
//  }
//}

// The serverEvent function is called whenever a new client connects.
public void serverEvent(Server server, Client client) {
  String message = "A new client has connected:" + client.ip();
  println(message);
}
public void draw() {
 background(00,195,50);
 noteY=15;
 Client getclient = myServer.available();

  if(inmessage != null){
    fill(0);
     text("Client says: "+ PApplet.parseChar(inmessage[0]),noteX,noteY,150,15);
    noteY= noteY+15; 
    text("Client says: "+ PApplet.parseChar(inmessage[1]),noteX,noteY,150,15);
    noteY= noteY+15; 
    text("Client says: "+ PApplet.parseChar(inmessage[2]),noteX,noteY,150,15);
    noteY= noteY+15; 
    text("Client says: "+ PApplet.parseInt(inmessage[3]),noteX,noteY,150,15);
    noteY= noteY+15; 
    text("Client says: "+ PApplet.parseInt(inmessage[4]),noteX,noteY,150,15);
     noteY= noteY+15; 

    for(int i = 5; i < inmessage.length-1; i++){
     text("Client says: "+ PApplet.parseInt(inmessage[i]),noteX,noteY,150,15);
     noteY= noteY+15; 
    }
     text("Client says: "+ PApplet.parseChar(inmessage[inmessage.length-1]),noteX,noteY,150,15);

  }
  if(getclient != null){

  if(getclient.available()>0){
    if(inmessage!=null){
     lastin = inmessage;
    }
   inmessage = getclient.readBytes();
  }
 }
}
public void keyPressed(){
//  if (key != ENTER && key != CODED && key != BACKSPACE){
//   //outmessage = outmessage + key;
//   //text(outmessage,10,height*9/10); 
//  }else if(key == BACKSPACE && outmessage.length()>0){
//   outmessage = outmessage.substring(0,outmessage.length()-1);
//   background(00,195,50);
//   redraw();
//  }else if(key == ENTER){
//   outmessage = outmessage + '\n';
//   myServer.write(outmessage);
//   outmessage = "";
//  }
}
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "rover_test_server" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
