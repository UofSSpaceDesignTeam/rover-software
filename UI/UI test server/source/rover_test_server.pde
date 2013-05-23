import processing.net.*;
Server myServer;
byte[] inmessage;
byte[] outmessage;
int noteX = 5;
int noteY = 15;
int count = 0;
byte[] lastin;

void setup(){
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
void serverEvent(Server server, Client client) {
  String message = "A new client has connected:" + client.ip();
  println(message);
}
