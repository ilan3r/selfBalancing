// // Use ESP32 LEDC for PWM (because analogWrite is not standard on ESP32)
// const int pwmChannelA = 0;
// const int pwmChannelB = 1;
// const int pwmFreq = 1000;
// const int pwmResolution = 8; // 8-bit = 0-255

// Motor A pins
const int IN1 = 15;
const int IN2 = 2;
const int ENA = 16;  // PWM capable

// Motor B pins
const int IN3 = 4;
const int IN4 = 0;
const int ENB = 12;  // PWM capable

const int motSpeed = 100;   // test the minimum motor speed

void setup() {
  Serial.begin(115200);
  Serial.println("Starting motor test...");

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
}

void loop() {
  // FORWARD
  Serial.println("Motors forward");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, motSpeed);  // 0–255
  analogWrite(ENB, motSpeed);
  delay(2000);

  // STOP
  Serial.println("Motors stop");
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  delay(1000);

  // REVERSE
  Serial.println("Motors reverse");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, motSpeed);
  analogWrite(ENB, motSpeed);
  delay(2000);

  // STOP
  Serial.println("Motors stop");
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  delay(1000);
}

