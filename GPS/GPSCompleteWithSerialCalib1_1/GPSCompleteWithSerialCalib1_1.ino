#include <TinyGPS.h>
#include <Servo.h>

/*
Calibration:  The program will display the currently read Base GPS coordinates every 5 seconds.  The user enters angles
between 20 and 180 degrees to turn the antenna relative to the base station.  Using trial and error the user locates the
angle with the highest signal strength (reading the signal strength using the programs associated with the antenna). 
Once the best servo angle is found and the program is reporting satisfactory, and consistent, GPS coordinates for the
base station the user enters 'y' or 'Y'.  The calibration angle is then calibrated and the program follows the rover
automatically.

Currently, the connected gps plays the part of the base.  The rover is set arbitrarily using coordinates, for testing.
Servo is attached on pin 9, GPS is on Serial2.  There is lots of print statements for troubleshooting that can be 
removed.  There is another gps lib (TinyGPSPlus) that can be used to incorporate doubles because the Arduino Due can
handle doubles.  That only needs to be done if we want higher precision of the values, but the angle has to be rounded
to an integer value anyways for the servo so it is probably unneccesary.

Work that needs to be done:
  - Integrate the 2nd gps (Existing function for this called getRoverGPS just needs to be adapted)
    * receive gps data from a second unit
    * use the received data instead of the arbitrarily assigned points
  - Debug statements need to be deleted or commented out (any Serial.print or Serial.println statement)
*/
//gpsRover connects to Serial2

//***    what I used to distinguish my comments from yours. Nicely done :)

TinyGPS gpsBase;
TinyGPS gpsRover;  //Probably not necessary if data is transmitted as coordinates
Servo antennaServo;
bool calibrated1 = false;// Used for 2 different phases of the initial calibration
int calibrationAngle;  // This is the angle that must be added to the angle from north to get the servo angle
int servoLimitLow = 20;
int servoLimitHigh = 160;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial2.begin(9600);
  antennaServo.attach(9);
  calibrationAngle = calibrate();
}

void loop() {
  // put your main code here, to run repeatedly: 
  int servoAngle;
  bool newData = getBaseGPS;
  if (newData){
    float roverLon, roverLat;
    float baseLon, baseLat;
    unsigned long age;
    float roverAngle;
    
    getRoverGPS(&roverLat, &roverLon);
    
    gpsBase.f_get_position(&baseLat, &baseLon, &age);
    //Add code here for retrieving the roverLat and roverLon
    //Include a test for proper data such as below if the data is not verified before retrieval
    
    if (baseLat == TinyGPS::GPS_INVALID_F_ANGLE || baseLon == TinyGPS::GPS_INVALID_F_ANGLE){
      Serial.println("GPS error");
    }
    else {
        roverAngle = calculateAngle(roverLat, roverLon, baseLat, baseLon);
        Serial.print("The angle(from north) is:  ");//all Serial.print statements are for troubleshooting only
        Serial.println(roverAngle);
        Serial.println(calibrationAngle);
        
        float angle = roverAngle - calibrationAngle;
        Serial.print("Servo angle:  ");
        Serial.println(angle);
        servoAngle = angle + 0.5;
        
        //if the rover goes out of the limits of the servos movement it stops trying to turn that far
        //Perhaps add a message to the user indicating that the rover is beyond the turning range
        servoAngle = constrain(servoAngle, servoLimitLow, servoLimitHigh);
        Serial.print("The angle(of the servo) is:  ");
        Serial.println(servoAngle);
        antennaServo.write(servoAngle);  //moves the servo to the determined angle
    }
  }
}

float calculateAngle(double destinationLat, double destinationLon, double homeLat, double homeLon){    //*** why use doubles when you only call this f'n using floats as params?
  destinationLon = (destinationLon * 71)/4068;//conversion to radians
  destinationLat = (destinationLat * 71)/4068;
  homeLon = (homeLon * 71)/4068;
  homeLat = (homeLat * 71)/4068;
  double dLon = destinationLon - homeLon;
  double y = sin(dLon) * cos(destinationLat);
  double x = cos(homeLat) * sin(destinationLat) - sin(homeLat) * cos(destinationLat) * cos(dLon);      //*** ah, I see
  float angle = atan2(y, x);
  angle = (angle * 4068)/71;
  return angle;
}

//Calibration Function
//Input servo angles to the program explicitly using serial monitor
//when the servo is pointed towards the roverGPS satisfactorily enter 'y' (without quotation marks)

int calibrate(){
  Serial.println("beginning calibration");
  char incomingByte[3];
  int servoAngle = 0;
  int calibrationAngle = 0;
  float roverLon, roverLat;
  float baseLon, baseLat;
  unsigned long age;
  bool newData = false;
  calibrated1 = false;
  int baseDisplayDelay = 0;
  while (calibrated1 == false){
    //Serial.println("calibrated1 = false");
    if (millis()/1000 > baseDisplayDelay){  //Display currently read base GPS every 5 seconds
      baseDisplayDelay += 5;
      DisplayBaseGPS(&baseLat, &baseLon, &age);
    }     
    int len = 0;
    while ((Serial.available() > 0) && (len < 3)) {            //*** Possible overflow of incomingByte? (i.e. if loop has more than 3 iterations)
      incomingByte[len] = Serial.read();                          // ^Fixed: Added a check for length
      len++;
      delay(100);
    }
    if (incomingByte[0] == 'Y' || incomingByte[0] == 'y'){    //*** nice catch for both upper and lower cases
      calibrated1 = true;
    }
    else if (incomingByte[0] >= '0' && incomingByte[0] <= '9'){    //*** what if input chars aren't y or 0->9?
      servoAngle = convertToInt(incomingByte, len);
      servoAngle = constrain(servoAngle, servoLimitLow, servoLimitHigh);
    }                                                           //*** everything above dealing with serial comms, could be a (or a few) seperate f'n's to allow more focus on the calibration itself
  }                                                              // ^ I made some into f'ns and deleted some that was unneccesary (constrained values instead of telling the user to stay within them)
  getRoverGPS(&roverLat, &roverLon);
  float roverAngle;
  roverAngle = calculateAngle(roverLat, roverLon, baseLat, baseLon);
  float angleTemp = roverAngle - servoAngle;
  float roundingValue = 0.5;
  if (angleTemp < 0){
    roundingValue = -0.5;
  }
  calibrationAngle = angleTemp + roundingValue;  
  return calibrationAngle;
}
    
//Reads GPS data from Serial2 and encodes it in gpsBase
//Add the same for the 2nd GPS if it is being read in the same way (probably isn't)
bool getBaseGPS(){
  bool newData = false;
  while (Serial2.available()){ 
    char c = Serial2.read();
    if (gpsBase.encode(c)){
      newData = true;
    }
  }
  return newData;
}

//This currently feeds arbitrary poins.  Needs to be updated to receive data.  It needs to assign
//the correct values to *roverLat and *roverLon then return true or return false if it fails to get data.
bool getRoverGPS(float *roverLat, float *roverLon){
  //Try to get data, return true if successful.  Put values in variables roverLat and roverLon by address
  bool newData = false;
  *roverLat = 52.12;
  *roverLon = -106.669;  
  *roverLat = *roverLat + int(millis()/1000)/100.0;  //should add .01 to roverLat every second
  return newData;
}

//converts an array of integers of length len into an int type.  Doesn't verify the incoming array, just tries to convert it.
int convertToInt(char incomingByte[], int len){
  int value = 0;
  for (int i = 0; i < len; i++){ 
    int increment = (int(incomingByte[i]) - 48) * pow(10, len-(1+i)); //converts number entered as characters into usable int value
    value = value + increment;
  }
  return value;
}

//Prints the base GPS coordinates to serial.  Used during calibration.
void DisplayBaseGPS(float *baseLat, float *baseLon, unsigned long *age){
  bool newData = getBaseGPS();
  if (newData){
    gpsBase.f_get_position(baseLat, baseLon, age);
    if (*baseLat == TinyGPS::GPS_INVALID_F_ANGLE || *baseLon == TinyGPS::GPS_INVALID_F_ANGLE){
      Serial.println("Base GPS invalid.");
    }
    else{
      Serial.print("Current base decimal degrees latitude:  ");
      Serial.println(*baseLat);
      Serial.print("Current base decimal degrees longitude:  ");
      Serial.println(*baseLon);
    }
  }
  else{
    Serial.println("No Base GPS data ready.");
  }
}
