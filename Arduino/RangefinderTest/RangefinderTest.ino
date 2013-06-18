
int sum;

void setup()
{
  Serial.begin(115200);
  Serial.println("IR Sensor Testing Program");
  Serial.println("Send any character to average 40 readings.");
}

void loop()
{
  while(!Serial.available());
  while(Serial.available())
    Serial.read();
  sum = 0;
  for(int i=0; i<40; i++)
  {
    sum += analogRead(0);
    delay(50);
  }
  Serial.println(sum/40);
}
