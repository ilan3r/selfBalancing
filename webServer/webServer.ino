// try to use a web server for the PID values 


#include <WiFi.h>
#include <WebServer.h>  

const int LEDbuiltIn = 2; 

const char* ssid = "Ilan's iphone";
const char* password = "abuhajar459";

WebServer server(80);

// Simple HTML page
const char* html = R"rawliteral(
<!DOCTYPE html>
<html>
  <head>
    <title>ESP32 Web Server</title>
  </head>
  <body>
    <h2>ESP32 Web Server</h2>
    <p>This is a test page running on the ESP32.</p>
  </body>
</html>
)rawliteral";

void handleRoot() {
  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}
