
float result;

void setup()
{
  Serial.begin(115200);
  Serial.println("Medium range IR Sensor Demo / Tuning Program");
  Serial.println("Send a character to begin.");
  while(!Serial.available());
}

void loop()
{
  result = 0;
  for(int i = 0; i < 20; i++)
  {
    result += analogRead(0);
  }
  result /= 20.0;
  result = 9600.0/(abs(result - 20.0)) + 2.5;
  
  if(result > 22) result += 2;
  if(result > 32) result += 2;
  if(result > 50) result += 2;
  if(result > 70) result += 2;
  if(result > 110) result -= 3;

  result = constrain((int)result,20,120);
  Serial.print("Range = ");
  Serial.print((int)result);
  Serial.println(" cm");
  delay(100);
}
