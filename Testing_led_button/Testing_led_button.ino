
#define LED_PIN_BUTTON 48

void setup() {
  // put your setup code here, to run once:
  SerialUSB.begin(115200);
  pinMode(LED_PIN_BUTTON, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_PIN_BUTTON, HIGH);
  SerialUSB.println("HIGH");
  delay(2000);
  digitalWrite(LED_PIN_BUTTON, LOW);
  SerialUSB.println("LOW");
  delay(2000);
}
