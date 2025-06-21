// simple script to get readings from the BNO055 sensor 

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);



void setup() {
  Wire.begin(21, 22);  // set SDA and SCL for ESP32
  bno.begin();

  Serial.begin(115200);
  delay(1000);

  if (!bno.begin()) {
    Serial.println("Couldn't find BNO055. Check wiring!");
    while (1);
  }

  bno.setExtCrystalUse(true); // Optional but improves accuracy
}

void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  Serial.print("Heading: ");
  Serial.print(euler.x());
  Serial.print(" | Pitch: ");
  Serial.print(euler.y());
  Serial.print(" | Roll: ");
  Serial.println(euler.z());

  delay(100);
}
