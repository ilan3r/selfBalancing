#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <BluetoothSerial.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);


// ===================================== variables
BluetoothSerial SerialBT;
bool oneTime = false; 
String inputBuffer = " "; 

const int PID_INTERVAL_MS = 10; 


//  PID loop variables 
float kp = 1.23;  // var1 
float kd = 4.56;   // var2 
float ki = 7.89;   // var3

float error, previousError = 0, integral = 0, derivative = 0; 
unsigned long lastTime = 0, startTime = 0, lastPIDtime = 0; 
bool calibrated = false; 
float targetAngle = 0, currentAngle = 0, output = 0; 


// ========================================== pin declarations 
const int LEDpin = 2; 

//Motor A pins
const int IN1 = 15;
const int IN2 = 2;
const int ENA = 16;  // PWM capable

// Motor B pins
const int IN3 = 4;
const int IN4 = 0;
const int ENB = 12;  // PWM capable




// ============= function prototypes 
void handleCommand(String cmd);
void writeMotors(int motorSpeed);
void runPIDloop(); 



// ======================= setup loop 
void setup() {

  Serial.begin(115200);
  pinMode(LEDpin, OUTPUT); 

  SerialBT.begin("ESP32-BT-Ilan");  // Bluetooth device name
  Serial.println("Bluetooth ready. Pair and open a serial terminal.");

  Wire.begin(21, 22);  // set SDA and SCL for ESP32
  bno.begin();

  if (!bno.begin()) {
    Serial.println("Couldn't find BNO055. Check wiring!");
    while (1);
  }

  bno.setExtCrystalUse(true); // Optional but improves accuracy

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  startTime = millis(); 

  digitalWrite(LEDpin, HIGH); 

}


// =============================== main loop 
void loop() {

  // calibrate if just started 
  if (!calibrated && (millis() - startTime > 5000)){

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    targetAngle = euler.y();  // Set roll as upright reference
    calibrated = true;
    lastTime = millis();
    Serial.println("Calibrated! Target angle set.");
    SerialBT.print("main angle has been set! \r\n");
    digitalWrite(LEDpin, LOW);

  }

  // don't do anything until calibrated 
  else if (!calibrated){
    return; 
  }


  // work out the timing
  unsigned long now = millis(); 
 
  
  delay(10); 


  if (SerialBT.available()) {
    if (!oneTime){
      SerialBT.print("Serial bluetooth conneccted! \r\n");
      oneTime = true; 
    }

    char c = SerialBT.read();
    if (c == '\r' || c == '\n'){
      SerialBT.print("newline triggured! \r\n");
      handleCommand(inputBuffer);
      inputBuffer = "";  // Clear after command processed
    } else {
      inputBuffer += c;
    }
  }

  // understand waht output does and cap it at 255 or -255 
  
  if (millis() - lastPIDtime >= PID_INTERVAL_MS){
    lastPIDtime = millis(); 
    runPIDloop();
  }

}


void runPIDloop(){
  // time calculations 
  float dt = (millis() - lastTime) / 1000.0; 
  lastTime = millis(); 

  // get angle from the BNO055 
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float currentAngle = euler.y(); 


  // PID calculations 
  error = targetAngle - currentAngle; 
  integral += error * dt; 
  derivative = (error - previousError) / dt; 
  output = kp * error + kd * derivative + ki * integral; 
  previousError = error; 

  output = constrain(output, -255, 255);
  writeMotors(output);

}




void writeMotors(int motorSpeed){

  // move backwards if negative number 
  if (motorSpeed < 0){
    // make the speed positive 
    motorSpeed *= -1; 
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, motorSpeed);
    analogWrite(ENB, motorSpeed);


  }

  else if (motorSpeed >= 0){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, motorSpeed);  
    analogWrite(ENB, motorSpeed);

  }
}




void handleCommand(String cmd) {
  cmd.trim();  // Remove leading/trailing spaces/newlines

  SerialBT.print("Received command: '");
  SerialBT.print(cmd);
  SerialBT.println("' \r\n");

  if (cmd.startsWith("set1=")) {
    ki = cmd.substring(5).toFloat();
    SerialBT.printf("Updated var1 = %.2f\r\n", kp);
  } 
  else if (cmd.startsWith("set2=")) {
    kd = cmd.substring(5).toFloat();
    SerialBT.printf("Updated var2 = %.2f\r\n", kd);
  } 
  else if (cmd.startsWith("set3=")) {
    ki = cmd.substring(5).toFloat();
    SerialBT.printf("Updated var3 = %.2f\r\n", ki);
  } 
  else if (cmd.equalsIgnoreCase("get")) {
    SerialBT.printf("var1  (kp) = %.2f\r\n", kp);
    SerialBT.printf("var2  (kd)= %.2f\r\n", kd);
    SerialBT.printf("var3  (ki)= %.2f\r\n", ki);
    SerialBT.printf("var4  (angle)= %.2f (read-only)\r\n", targetAngle);
  } 
  else if (cmd.equalsIgnoreCase("h")) {
    SerialBT.println("hello world\r\n");
  } 
  else if (cmd.equalsIgnoreCase("a")) {
    SerialBT.printf("angle = %.2f  |  output = %.2f \r\n", currentAngle, output);
  }
  else {
    SerialBT.println("Unknown command\r\n");
  }
}
