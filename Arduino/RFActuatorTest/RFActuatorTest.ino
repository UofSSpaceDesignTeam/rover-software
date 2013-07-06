// Currently set up for test platform motor controller.
// Wiring connections: blk -> A, red -> B, white -> 5v, yellow -> gnd
// DIR -> pin 4, PWM -> pin 5, GND -> gnd, blue -> analog in 0

#define FULL_IN 963 // raw adc values at limits
#define FULL_OUT 57

char msg[4]; // command buffer
int pos; // actuator position
int lastpos;
int sum; 
int distance; //distance as detected by RF
char RF;

void setup()
{
  Serial.begin(115200);
  pinMode(6,OUTPUT); // pwm
  pinMode(7,OUTPUT); // a
  pinMode(8,OUTPUT); // b
  Serial.println("Enter a position (0-140mm). Or enter 'R' to use RF.");
  Serial.println("Current position:");
  pos = getPosition();
  lastpos = pos;
  Serial.println(pos);
  Serial.println("Current distance:");
  distance=getDistance();
  Serial.println(distance);
  Serial.println("Postion:  Distance Read:");
}


void loop()
{
  pos = getPosition();
  if(abs(pos - lastpos) > 1) // watch for external movement
  {
    Serial.print(pos);
    Serial.print("   ");
    Serial.println(distance);
    lastpos = pos;
    delay(25);
  }
  if(Serial.available()) // got a command
  {
    delay(50); // wait for all of it to come in
    msg[0] = Serial.read();
    msg[1] = Serial.read();
    msg[2] = Serial.read();
    msg[3] = '\0';
    if(msg[0]=='r')
    {
      distance=getRange();
      movePosition(map(distance,0,1023,0,140));
    }
    if(msg[0]!=='r')
    {
      movePosition(constrain(atoi(msg),0,140)); // move to given position
    }
    Serial.flush();
  }
}

int getPosition() // get current position from adc
{
  return constrain(map(analogRead(0),FULL_IN,FULL_OUT,0,140),0,140);
}

void movePosition(int p) // move near a specified position
{
  pos = getPosition();
  if(p > pos + 1)
  {
    moveOut();
    while(p > pos + 1)
    {
      pos = getPosition();
      Serial.print(pos); // report position while moving 
      Serial.print("   ");
      Serial.println(distance);
      delay(50);
    }
    halt();
    return;
  }
  if(p < pos - 1)
  {
    moveIn();
    while(p < pos - 1)
    {
      pos = getPosition();
      Serial.print(pos);
      Serial.print("   ");
      Serial.println(distance);
      delay(50);
    }
    halt();
    return;
  }
}


void moveOut()
{
   digitalWrite(7,LOW);
   digitalWrite(8,HIGH);
   analogWrite(6,255);
}


void moveIn()
{
   digitalWrite(7,HIGH);
   digitalWrite(8,LOW);
   analogWrite(6,255);
}


void halt()
{
   analogWrite(6,0);
}


int getDistance()
{
    sum=0;
  for(int i=0; i<40; i++)
  {
    sum+= analogRead(1);
    sum=sum/40;  //takes 40 readings and averages them
    return sum;
  }
}

