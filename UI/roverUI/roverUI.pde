import processing.net.*;
boolean keydown; /* keeps track of when a button is pressed released, (becomes true when a key is pressed and false when it is released
                 to counteract the "hold key to repeat" function that all computer keyboards use
                  eg: holding down the "k" key will type "kkkkkkkkkkkk" which we don't want for press-and-hold keyboard controls*/
Client client; /* client used to connect to rover server*/ 
String testIp = "127.0.0.1";/*for testing that does not need connection to rover, requires running test server program*/
//String testIp = "10.227.0.179"; /*local IP of computer running test server; (this one varies between computers and networks so don't be afraid to change it as needed)*/
String roverIp_wireless = "192.168.1.10"; /* Current IP of rover for wireless connection */
String roverIp_tethered = "192.168.1.11"; /* Current IP of rover for tethered ethernet connection */

//String roverIp = "10.227.5.214"; /* Current IP of rover, may change as network changes */

/* port used for connection to the rover server*/
int comm_port = 7050; 
/*emergency stop button dimensions are defined separately due to the button having a unique size */
int eStopX = 300;      
int eStopy = 600;      
int eStopWidth = 200; 
int eStopHeight = 50;
/* x-coordinate of video monitor location (camera video feed)*/
int videoX = 352; 
/* y-coordinate of video monitor location*/
int videoY = 200; 
/* default colour of buttons when not activated(excludes emergency stop button)*/
color default_color = color(0,100,15); 
/*color of buttons when active/pressed (excludes emergency stop button)*/
color active_color = color(30,220,40); 

/* variable holding speed of right motor (will need to be changed when number of motors is finalized)*/
int motor1Speed = 0; 
/* variable holding speed of left motor (see above comment)*/
int motor2Speed = 0; 

 /*variable used for timing messages sent to rover to prevent timeout disconnects (may not be necessary)*/ 
int timer = millis();
/*keeps track of time that last message was sent at (relative to starting UI program) */
int last_message = 0; 
/* tracks amount of data sent over network (in bytes) */
int bytes_sent = 0;  

/*setup is run once at startup before starting the main "draw" loop 
 it is mainly used for giving variables their starting values and defining the size of the window the program will run in*/
void setup() 
{
  keydown = false; 
  size(1024,600); // size of UI window
  
  /*sets buttons which are on/active by default */
  ai_offButton.activate();
  video_offButton.activate();
  video_depthButton.activate();
  //wireless_serverButton.activate();
  
  /*sets buttons which compliment each other turning
  one on will turn its complement off and vice-versa*/
  //wireless_serverButton.setOther(test_serverButton);
  //test_serverButton.setOther(rover_serverButton);
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
  
  for (int i = 0; i < window_messages.length; i = i+1) {
   window_messages[i] = " ";
  }
 
//  try{ client = new Client(this, roverIp_wireless, comm_port); //
//    println("wireless server " + roverIp_wireless + " is available");
//    client.stop();
//  } catch (NullPointerException a){ 
//     try{ client = new Client(this, roverIp_tethered, comm_port); //
//     println("tethered server " + roverIp_tethered + " is available");
//     client.stop();
//     } catch (NullPointerException b){ 
//       try{ client = new Client(this, testIp, comm_port); //
//       println("test server " + testIp + " is available");
//       client.stop();
//       } catch (NullPointerException c){ 
//         println("No server available");
//       }
//     }   
//  }
  
}

 /* main loop of program*/
void draw()
{
  /*the first part of the loop just redraws all objects seen on the UI window*/
  
  textAlign(CENTER,CENTER); // all text drawn using the text function will appear 
                            // with the x- and y-coordinates at the centre of the text
  
  background(230); // changes the colour of the window's background
  fill(210); //sets colour of drawn objects 
  stroke(0); // sets colour of lines and borders to 0 (black)
  strokeWeight(2); // sets thickness of lines and borders
  //rect(35,120,85,240);  //}box around digger controls
  // rect(35,120,170,120); //}
  
  //rect(220,120,85,240);    //box arount bucket controls
  //rect(715,80,255,120);   //box around camera controls
  //rect(715,235,255,120);   //box around movement controls
  //rect(715,450,300,140);   //box around AI Status
  rect(330,75,videoX,videoY);//Video box

  //  additional text labels for understanding the UI
  fill(0);
  textSize(24);
  //text("Excavation",160,70);
  text("Camera",845,65);
  text("Movement",845,220);
  text("Video", 500,30);
  text("Messages",200,400);
  textSize(16);
  text("Digger",120,100);
  text("Bucket",265,100);
  text("AI Status",870,440);
  textSize(15);
  text("Arrow Keys",925,355);
  text("MIN",713,418);
  text("MAX",990,418);
  text(bytes_sent,30,60); 
  // This second part of the loop calls upon the functions 
  // that do all of the thinking
  
  drawButtons();  // draws all of the UI buttons (moved to a separate function to keep the main loop clean(er)  -- see drawButtons_function tab for more detail
  debug_messages();
  speedbar.update(); //updates the position of the speed slider if it has been moved -- see speed_slider tab for more detail
  speedbar.display();  //draws the speed slider in its new location                  -- ditto
  text(speedbar.getPos(),940,400); // text to show what the speed slider's current speed value is
  connection(); // manages server connection -- see below
  //if(connected){
    getMessages();  // checks for messages from server -- see messages tab for more detail
    sendMessages();  // sends messages to server       -- ditto
  //}
}


byte[] debug = new byte[0];

boolean connected = false; // variable is set to true when client connects to a server
// connection() - checks client connection status, then attempts to connect,disconnect, or reconnect
//                 depending on what commands were given
void connection() {
 
  if (connectButton.isActive()){// if the connect button was clicked, attempt to connect to a server
     
      try{client.ip(); // checks to see if client is already connected to the server
        if (millis() - timer > 1000) { // if connection is going to time out
          //debug[0] = 0;
          if(millis()-last_message>message_delay){
            outMessage.Message(MessageProtocol.ID1_DEBUG,MessageProtocol.ID2_CHECK,debug);
            outMessage.sendMessage();
          //message to maintain connection (to do later);  // send message to maintain connection
            last_message = millis();
            timer = millis();  //reset timeout timer
          }
        }
      } catch (NullPointerException e){  // if not connected, does the following
        if (connected == false){ // sends a connecting message to user on first connection
          ///new_debug_message("Conncecting to Server..."); //moved for faster feedback
        } else { // if there previously was a connection, notify user of disconnection
          new_debug_message("Lost Connection");
          connectButton.deactivate();
          connected = false;
        }
        
        if(wireless_serverButton.isActive()){ // attempts to connect to an IP based on server button selected
          client = new Client(this, roverIp_wireless, comm_port);
        } else if (tethered_serverButton.isActive()) {
          client = new Client(this, roverIp_tethered, comm_port);
        } else {
          client = new Client(this, testIp, comm_port);      
        }
        try{client.ip(); // verifies that a connection was made
          new_debug_message("Conncected to "+client.ip());
          connected = true;
          connectButton.activate();

        } catch (NullPointerException f){// if a connection was not made, notify user
          new_debug_message("Unable to Connect");  
          connectButton.deactivate();  // deactivates connection button so that program does not get stuck
                                       // in an infinite loop of connection attempts
        }
    }  
  } else { // if the connection button is disabled, notify the user of the disconnection
    if (connected == true){ // but only if previously connected
      client.stop();
      new_debug_message("Connection Terminated");
      connected = false;
    }
    
  }
}

