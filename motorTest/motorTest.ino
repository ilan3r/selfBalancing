// simple test for motor control 

const int IN1 = 18;     // Motor A input 1
const int IN2 = 19;     // Motor A input 2
const int ENA = 23;     // Motor A enable (PWM capable)

void setup() {
  // Set pin modes
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Start Serial for debugging
  Serial.begin(115200);
  Serial.println("Motor test starting...");
}

void loop() {
  // Forward
  Serial.println("Motor Forward");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 200);  // PWM value (0-255)

  // delay(2000);

  // // Stop
  // Serial.println("Motor Stop");
  // analogWrite(ENA, 0);
  // delay(1000);

  // // Reverse
  // Serial.println("Motor Reverse");
  // digitalWrite(IN1, LOW);
  // digitalWrite(IN2, HIGH);
  // analogWrite(ENA, 200);

  // delay(2000);

  // // Stop again
  // Serial.println("Motor Stop");
  // analogWrite(ENA, 0);
  // delay(2000);
}

