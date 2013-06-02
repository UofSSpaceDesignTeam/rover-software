
// Other functions for communication. See "Message format information" on Google Drive


void debug() // sends the human readable info in debugmsg to the fitPC
{
  int length = debugmsg.length();
  Serial.write("#DB"); // debug message ID
  Serial.write((byte)length/256); // high byte of 2-byte length
  Serial.write((byte)length%256); // low byte of 2-byte length
  Serial.print(debugmsg); // send the string
  Serial.write(0x0A); // bare newline
  debugmsg = ""; // reset the string to empty
}

int parseBytes(byte high, byte low) // turns 2 bytes into an int
{
  return (int)high/256 + (int)low%256;
}

