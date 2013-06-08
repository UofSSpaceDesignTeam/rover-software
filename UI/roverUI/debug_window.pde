String[] window_messages = new String[10];

void debug_messages()
{
  fill(210); //sets colour of drawn objects 
  stroke(0); // sets colour of lines and borders to 0 (black)
  strokeWeight(2); // sets thickness of lines and borders
  rect(10,420,405,170);      // draws the message box
  
  textAlign(LEFT);
  fill(0);
  textSize(14);
  int gap = 16;
  for (int i = 0; i < window_messages.length; i = i+1) {
    text(window_messages[i],15,438+i*gap);
  }  
}

void new_debug_message(String deb_message)
{
  for (int i = 0; i < window_messages.length-1; i = i+1) {
   window_messages[i] = window_messages[i+1];
  }
  window_messages[window_messages.length-1] = deb_message;  
  debug_messages();
}
