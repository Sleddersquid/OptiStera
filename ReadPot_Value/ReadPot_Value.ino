
int pot_value_read;

#define POT_PIN_V A6


void setup() {
  // put your setup code here, to run once:
  SerialUSB.begin(115200);

  pinMode(POT_PIN_V, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  pot_value_read = analogRead(POT_PIN_V);

  SerialUSB.println(pot_value_read);
}
