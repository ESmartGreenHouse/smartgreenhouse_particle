void setup() {
    pinMode(A2, INPUT);
}
void light_sensor(int pin) {
   int val = analogRead(pin);
   Serial.print(100*val/2510); //lighting value in percents
   Serial.print("%\n\r");
   delay(500);
}

void loop()
{
    light_sensor(A2);
}

