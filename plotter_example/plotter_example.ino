int random_variable;
int static_variable = 500;

void setup() {
  SerialUSB.begin(115200);
}

void loop() {
  random_variable = random(0, 1000);

  SerialUSB.print("Variable_1:");
  SerialUSB.print(random_variable);
  SerialUSB.print(",");
  SerialUSB.print("Variable_2:");
  SerialUSB.println(static_variable);
}

