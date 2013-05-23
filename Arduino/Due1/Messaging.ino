
// Low level functions for communication. See "Message format information" on Google Drive


void debug() // sends the human readable info in debugmsg to the fitPC
{
  int length = debugmsg.length();
  Serial.write('#');
  Serial.write("DB"); // debug message ID
  Serial.write((byte)length/256); // high byte of 2-byte length
  Serial.write((byte)length%256); // low byte of 2-byte length
  Serial.print(debugmsg); // send the string
  Serial.write(0x0A);
  debugmsg = ""; // reset the string to empty
}


void sendMessage(char* id, const char* data) // send a message of a specified type
{
  int length = 0;
  
  if(id == "ID") // message is imu data
    length = 12;
    
  else if(id == "WD") // message is encoder data
    length = 2*NUM_ENCODERS;
    
  else if(id == "FD") // message is force sensor data
    length =  2*NUM_FORCE;
    
  Serial.write('#');
  Serial.write(id); // message ID
  Serial.write(length/256); // high byte of 2-byte length
  Serial.write(length%256); // low byte of 2-byte length
  Serial.write(data); // byte stream of actual data
  Serial.write(0x0A);
}

int parseBytes(byte high, byte low) // turns 2 bytes into an int
{
  return (int)high/256 + (int)low%256;
}

