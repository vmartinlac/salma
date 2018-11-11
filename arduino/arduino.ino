
const int ms = 500;

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
