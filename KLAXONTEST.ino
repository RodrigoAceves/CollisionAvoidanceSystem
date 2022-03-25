#define KLAXON 6
void setup() {
  // put your setup code here, to run once:
 pinMode(KLAXON, OUTPUT);
 Serial.begin(9600);
}

void loop() {
  digitalWrite(KLAXON,HIGH);
  delay(500);
  digitalWrite(KLAXON,LOW);
  delay(500);
}
