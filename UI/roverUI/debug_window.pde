
//variable for storing the messages displayed in the message box
String[] window_messages = new String[10];

//draws the debug message window and its messages when called
//it's the "messages" window in the bottom left of the UI
void debug_messages()
{
  //the following code is for drawing the message "box" itself
  fill(210); //sets fill colour to  light grey 
  stroke(0); // sets colour of the box borders to 0 (black)
  strokeWeight(2); // sets thickness of the borders
  rect(10,420,405,170);  // draws the message box at the given coordinates
  
  //the following is for drawing the messages themselves
  textAlign(LEFT);  //left aligns the text drawn with the "text" function
  fill(0);          // colour of  message text set to black
  textSize(14);     //size of message text
  int gap = 16;     // spacing between messages
  for (int i = 0; i < window_messages.length; i = i+1) {
    text(window_messages[i],15,438+i*gap);
  }  
}

//function for adding a new message to the debug message window
// "deb_message" being the new message to be displayed
void new_debug_message(String deb_message)
{
  //first all old messages are shifted "up" one space in the message array
  //effectively throwing away the oldest message
  for (int i = 0; i < window_messages.length-1; i = i+1) {
   window_messages[i] = window_messages[i+1];
  }
  //then the new message is added in the "lowest" space
  //so that it appears at the bottom of the message window
  window_messages[window_messages.length-1] = deb_message;
  
  //this function runs the message drawing function again so that the new message is displayed right away
  //instead of waiting for the main draw function to loop again
  //**added to make messages scroll more smoothly, especially when many new messages are added
  //within a small amount of time (~1 second)**  
  debug_messages();
}
