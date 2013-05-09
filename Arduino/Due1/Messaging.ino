
// Various low level functions for communication. See "Message format information" on Google Drive


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


void reply(const char* id) // send a reply when required (on secondary serial port)
{
  Serial1.write('#');
  Serial1.write(id); // reply ID varies
  Serial1.write((byte)0); // not an error message, so zero data bytes are specified here
  Serial1.write((byte)0);
  Serial1.write(0x0A);
}


void error(const char* id, const char* code) // an error message is a special case of a reply
{
  Serial1.write('#');
  Serial1.write(id); // reply ID varies
  Serial1.write((byte)0);
  Serial1.write((byte)2); // two data bytes to communicate our error code
  Serial1.write(code); // the 2 byte error code
  Serial1.write(0x0A);
}


boolean isCritical(char* id) // Messages of certain types will require replies
{
  if(id[0] == 'D' || id[0] == 'P' || id[0] == 'M' || id[0] == 'S')
    return true;
  return false;
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

