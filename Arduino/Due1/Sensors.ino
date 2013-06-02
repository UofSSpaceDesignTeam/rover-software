
// functions pertaining to individual sensors  and their messages

void sendIMUs() // reads the IMUs and sends the data
{
  int16_t ax, ay, az, pitch, roll;
  imu.getAcceleration(&ax, &ay, &az);
  pitch = (int)(100*atan2(ay, az));
  pitch = constrain(pitch,-9000,9000);
  roll = (int)(100*atan2(ax, az));
  roll = constrain(roll,-9000,9000);
    
  Serial.write("#ID");
  Serial.write((byte)0); // length byte 1
  Serial.write((byte)4); // length byte 2
  Serial.write((byte)pitch/256); // high byte
  Serial.write((byte)pitch%256); // low byte
  Serial.write((byte)roll/256); // high byte
  Serial.write((byte)roll%256); // low byte
  Serial.write(0x0A); // bare newline
}
  
