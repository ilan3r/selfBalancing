#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

bool oneTime = false; 

void handleCommand(String cmd);
String inputBuffer = " "; 

float var1 = 1.23;    // kp term 
float var2 = 4.56;    // kd term 
float var3 = 7.89;    // ki term 
float var4 = 25.00;    // set angle 

const int LEDpin = 2; 

void setup() {
  pinMode(LEDpin, OUTPUT); 
  Serial.begin(115200);
  SerialBT.begin("ESP32-BT-Ilan");  // Bluetooth device name
  Serial.println("Bluetooth ready. Pair and open a serial terminal.");
}



void loop() {

  digitalWrite(LEDpin, HIGH);

  while (SerialBT.available()) {
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
}


void handleCommand(String cmd) {
  cmd.trim();  // Remove leading/trailing spaces/newlines

  SerialBT.print("Received command: '");
  SerialBT.print(cmd);
  SerialBT.println("' \r\n");

  if (cmd.startsWith("set1=")) {
    var1 = cmd.substring(5).toFloat();
    SerialBT.printf("Updated var1 = %.2f\r\n", var1);
  } else if (cmd.startsWith("set2=")) {
    var2 = cmd.substring(5).toFloat();
    SerialBT.printf("Updated var2 = %.2f\r\n", var2);
  } else if (cmd.startsWith("set3=")) {
    var3 = cmd.substring(5).toFloat();
    SerialBT.printf("Updated var3 = %.2f\r\n", var3);
  } else if (cmd.equalsIgnoreCase("get")) {
    SerialBT.printf("var1 = %.2f\r\n", var1);
    SerialBT.printf("var2 = %.2f\r\n", var2);
    SerialBT.printf("var3 = %.2f\r\n", var3);
    SerialBT.printf("var4 = %.2f (read-only)\r\n", var4);
  } else if (cmd.equalsIgnoreCase("h")) {
    SerialBT.println("hello world\r\n");
  } else {
    SerialBT.println("Unknown command\r\n");
  }
}

 
