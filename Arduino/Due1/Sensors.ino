
// functions pertaining to sensor interfacing and their messages

void sendIMUs() // reads the IMUs and sends the data
{
  int16_t ax, ay, az, pitch, roll; // need some 16 bit signed ints
  imu.getAcceleration(&ax, &ay, &az); // read raw accel values
  pitch = (int)degrees((atan2(ax, az))); // calculate pitch in degrees
  pitch = constrain(pitch,-90,90); // ensure it's in acceptable range
  roll = (int)degrees((atan2(ay, az))); // calculate roll in degrees
  roll = constrain(roll,-90,90); // ensure it's in acceptable range
    
  Serial.write("#ID"); // message header
  Serial.write((byte)0); // length byte 1
  Serial.write((byte)2); // length byte 2
  Serial.write((char)pitch); // high byte
  Serial.write((char)roll); // high byte
  Serial.write(0x0A); // bare newline
}

void levelRangefinders()  // uses IMU readings to level the sensor pods
{
  int16_t ax, ay, az, pitch, roll; // need some 16 bit signed ints
  imu.getAcceleration(&ax, &ay, &az); // read raw accel values
  pitch = (int)degrees((atan2(ax, az))); // calculate pitch in degrees
  pitch = constrain(pitch,-90,90); // ensure it's in acceptable range
  roll = (int)degrees((atan2(ay, az))); // calculate roll in degrees
  roll = constrain(roll,-90,90); // ensure it's in acceptable range
  
  // todo, attach stuff here
  
  // the below values may be + or - depending on mounting, and need calibration.
  frontLeftServo.write(90 + pitch);
  frontRightServo.write(90 - pitch);
  leftSideServo.write(90 + roll);
  rightSideServo.write(90 - roll);
  rearLeftServo.write(90 + pitch);
  rearRightServo.write(90 - pitch);
}
  
