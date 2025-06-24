// simple script to make it so that the accelerometer can tell the difference between leaning forward and backwards

// leaning forwards creates obtuse angle (90-180 deg), upright is 90 degrees, leaning backwards creates acute angle 

// eventually might be useful to combine euler angle and accelerometer data by using somethin like a kalman filter

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

const int LEDbuiltIn = 2;

float finalAngle = 0.0; 



void setup() {
  pinMode(LEDbuiltIn, OUTPUT);
  digitalWrite(LEDbuiltIn, HIGH);

  Wire.begin(21, 22);  // set SDA and SCL for ESP32
  bno.begin();

  Serial.begin(115200);
  Serial.println("hello world");
  delay(1000);

  if (!bno.begin()) {
    Serial.println("Couldn't find BNO055. Check wiring!");
    while (1);
  }

  bno.setExtCrystalUse(true); // Optional but improves accuracy

  digitalWrite(LEDbuiltIn, LOW);
}

void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);



  Serial.printf("x_angle = %.2f  |  y_angle = %.2f  |  z_angle = %.2f  |  " , euler.x(), euler.y(), euler.z());




  sensors_event_t accel_event;
  bno.getEvent(&accel_event, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  float accel_x = accel_event.acceleration.x;
  float accel_y = accel_event.acceleration.y;
  float accel_z = accel_event.acceleration.z;
  Serial.printf("a_x = %.2f  |  a_y = %.2f  |  a_z = %.2f  |  " , accel_x, accel_y, accel_z);


  // try to use gravity to calculate angle 
  float pitch = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0 / PI;
  Serial.printf("pitch = %.2f", pitch);

  delay(100);

  // if leaning forwards, a_z is negative 
  if (accel_z < 0){
    finalAngle = 180.0 - euler.y();
  }

  // if leaning backwards, a_z is positive 
  else {
    finalAngle = euler.y(); 

  }

  Serial.printf("final angle = %.2f \r\n", finalAngle);



}
