/*

TODO: 
- add a globabl boolean that can be used to print out the angle and output to the serial blueooth monitor
- why isn't motor1 (ENA) moving backwards at all? -- beacuse the LED pin is the same as the motor direction pin oops

*/


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
float kp = -5.0;  // var1 
float kd = 0.0;   // var2 
float ki = 0.0;   // var3

float error, previousError = 0, integral = 0, derivative = 0; 
unsigned long lastTime = 0, startTime = 0, lastPIDtime = 0; 
bool calibrated = false; 
float targetAngle = 67.00; 


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

float getAngle(); 



// ======================= setup loop 
void setup() {

  Serial.begin(115200);
  // pinMode(LEDpin, OUTPUT); 
  digitalWrite(LEDpin, HIGH); 
  

  SerialBT.begin("ESP32-BT-Ilan2");  // Bluetooth device name
  Serial.printf("Bluetooth ready. Pair and open a serial terminal.");

  Wire.begin(21, 22);  // set SDA and SCL for ESP32
  bno.begin();

  if (!bno.begin()) {
    Serial.println("Couldn't find BNO055. Check wiring!");
    while (1);
  }
  Serial.println("hello world");

  bno.setExtCrystalUse(true); // Optional but improves accuracy

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  startTime = millis(); 



}


// =============================== main loop 
void loop() {
  // digitalWrite(LEDpin, LOW);

  // calibrate if just started 
  if (!calibrated && (millis() - startTime > 5000)){

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    // targetAngle = euler.y();  // Set roll as upright reference
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
  
  if (millis() - lastPIDtime >= PID_INTERVAL_MS){
    lastPIDtime = millis(); 
    runPIDloop();
  }

}


void runPIDloop(){
  // time calculations 
  float dt = (millis() - lastTime) / 1000.0; 
  lastTime = millis(); 

  float currentAngle = getAngle();  // pitch





  // PID calculations 
  error = targetAngle - currentAngle; 
  integral += error * dt; 
  derivative = (error - previousError) / dt; 
  float output = kp * error + kd * derivative + ki * integral; 
  previousError = error; 


  float outputCap = constrain(output, -255, 255);

  // SerialBT.printf("angle = %.2f  |  output = %.2f \r\n", currentAngle, output);
  Serial.printf("targetAngle = %.2f  |  angle = %.2f  |  outputCap = %.2f  |  output = %.2f  |  error = %.2f \r\n" , targetAngle, currentAngle, outputCap, output, error);


  writeMotors(output);

}




void writeMotors(int motorSpeed){

  if (motorSpeed > 100 && motorSpeed < -100){
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);

  }

  // move backwards if negative number 
  if (motorSpeed < -100){
    // make the speed positive 
    motorSpeed *= -1; 
    Serial.println("BACKWARDS ====================================================");

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);


    // analogWrite(ENA, abs(motorSpeed));
    analogWrite(ENA, abs(motorSpeed));
    analogWrite(ENB, abs(motorSpeed));


  }

  else if (motorSpeed >= 100){

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    analogWrite(ENA, motorSpeed);  
    analogWrite(ENB, motorSpeed);

  }
}



float getAngle(){
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  sensors_event_t accel_event;
  bno.getEvent(&accel_event, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  float accel_z = accel_event.acceleration.z;

  // if leaning forwards, a_z is negative 
  if (accel_z < 0){
    return (180.0 - euler.y());
  }

  // if leaning backwards, a_z is positive 
  else {
   return euler.y(); 

  }


}


void handleCommand(String cmd) {
  cmd.trim();  // Remove leading/trailing spaces/newlines

  SerialBT.print("Received command: '");
  SerialBT.print(cmd);
  SerialBT.println("' \r\n");

  if (cmd.startsWith("set1=")) {
    kp = cmd.substring(5).toFloat();
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
  else {
    SerialBT.println("Unknown command\r\n");
  }
}
