#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

float var1 = 1.23;
float var2 = 4.56;
float var3 = 7.89;
float var4 = 0.00;  // Read-only

const int LEDpin = 2; 

void setup() {
  pinMode(LEDpin, OUTPUT); 
  Serial.begin(115200);
  SerialBT.begin("ESP32-BT-Ilan");  // Bluetooth device name
  Serial.println("Bluetooth ready. Pair and open a serial terminal.");
}

void loop() {
  digitalWrite(LEDpin, HIGH);
  if (SerialBT.available()) {
    String input = SerialBT.readStringUntil('\n');

    if (input.startsWith("set1=")){
      var1 = input.substring(5).toFloat();
      SerialBT.printf("var1 = %.2f\r\n", var1);

    } 

    else if (input.startsWith("set2=")){
      var2 = input.substring(5).toFloat();
      SerialBT.printf("var2 = %.2f\r\n", var2);


    } 
    else if (input.startsWith("set3=")){
      var3 = input.substring(5).toFloat();
      SerialBT.printf("var3 = %.2f\r\n", var3);

    } 
    else if (input == "get") {
      SerialBT.printf("var1 = %.2f\r\n", var1);
      SerialBT.printf("var2 = %.2f\r\n", var2);
      SerialBT.printf("var3 = %.2f\r\n", var3);
      SerialBT.printf("var4 = %.2f (read-only)\r\n", var4);
    }
    else if (input == "h") {
      SerialBT.printf("hello world! \r\n");
    }
  }
}
