
const int ms = 50;

void loop()
{
  digitalWrite(2, HIGH);
  delay(ms);
  digitalWrite(2, LOW);
  delay(ms);
}

void setup()
{
  pinMode(2, OUTPUT);
}
