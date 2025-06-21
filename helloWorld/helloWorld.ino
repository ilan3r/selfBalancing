// test LED and serial connection

const int LEDbuiltIn = 2; 

void setup() {
  // put your setup code here, to run once:
  // connect to UART port 
  // in windows device manager, make sure it says silicon labs CP210x
  Serial.begin(9600);
  Serial.println("Hello World!");
  pinMode(LEDbuiltIn, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  digitalWrite(LEDbuiltIn, LOW); // Turn the LED on
  delay(100);                      // Wait for 1 second
  digitalWrite(LEDbuiltIn, HIGH);  // Turn the LED off
  delay(500);                      // Wait for 1 second
}

