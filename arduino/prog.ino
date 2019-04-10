void setup()
{
  pinMode(2, OUTPUT);
  Serial.begin(9600);
}

void loop()
{
  Serial.flush();
  
  if( Serial.read() >= 0 )
  {
    digitalWrite(2, HIGH);
    delay(1);
  }
  
  digitalWrite(2, LOW);
  delay(1);
}
