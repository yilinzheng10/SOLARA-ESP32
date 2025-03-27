#include <WiFiUdp.h>
#include <NTPClient.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <Ticker.h>

Ticker sunTrackingTicker; 
const char* ssid = "ATTItaAPVc";
const char* password = "t88vj3bfkfnv";

WiFiUDP udp;
NTPClient timeClient(udp, "pool.ntp.org", 0, 3600); // 3600 is for GMT+1, adjust if needed
const long timeOffset = -6 * 3600;  // CST (Standard Time)
const long dstOffset = -5 * 3600; 

WebServer server(80);
Servo myservo;

String currentMode = "Closed";
int currentDegree = 90;
const int minDegree = 0;
const int maxDegree = 180;
const int closedDegree = 90;


void startSunTracking() {
  sunTrackingTicker.attach(60, updateSunPosition); // Call updateSunPosition every 60 seconds
}

void stopSunTracking() {
  sunTrackingTicker.detach(); // Stop the timer
}

// Root route handler
void handleRoot() {
  server.send(200, "text/plain", "ESP32 is connected!");
}

void positionClose() {
  if(currentMode == "Sun Tracking"){
    stopSunTracking();
  }

  currentMode = "Closed";
  currentDegree = 90;
  myservo.write(90);
  Serial.println(currentDegree);
  server.send(200, "text/plain", "Servo is 90 degrees");
}

void positionOpen() {
  if(currentMode == "Sun Tracking"){
    stopSunTracking();
  }
  currentMode = "Open";
  myservo.write(180);
  currentDegree = 180;
  server.send(200, "text/plain", "Servo is 180 degrees");  // Fix response to reflect actual position
}

void positionManual(int degree) {
  myservo.write(degree);
  currentDegree = degree;
  server.send(200, "text/plain", "Servo is \(degree) degrees");
}

void setPositionManual() {
  if(currentMode == "Sun Tracking"){
    stopSunTracking();
  }
  currentMode = "Manual";
  server.send(200, "text/plain", "Servo is manual");
}

void setSunTracking() {
  currentMode = "Sun Tracking";
  startSunTracking();
  server.send(200, "text/plain", "Servo is automatic");
}

void getMode() {
  server.send(200, "text/plain", currentMode);
  Serial.println(currentMode);
}

void getCurrentDegree() {
  String degreeString = String(currentDegree); // Convert int to String
  server.send(200, "text/plain", degreeString); // Send the string
  Serial.println(currentDegree); // This remains unchanged, as Serial.print can handle int
}

int calculateSunPosition() {
  unsigned long epochTime = timeClient.getEpochTime(); // Get time in seconds since epoch
  
  // Cast to time_t (since ESP32 uses 64-bit time_t)
  time_t time = (time_t)epochTime;
  struct tm* timeinfo = localtime(&time); // Convert to local time
  int hour = timeinfo->tm_hour;
  int minute = timeinfo->tm_min;

  // Calculate the sun position based on the hour and minute
  const int totalMinutesInDay = 12 * 60; // 720 minutes (6 AM to 6 PM)
  int currentMinutesInDay = (hour - 6) * 60 + minute;

  // Ensure currentMinutesInDay is within bounds
  if (currentMinutesInDay < 0) currentMinutesInDay = 0;
  if (currentMinutesInDay > totalMinutesInDay) currentMinutesInDay = totalMinutesInDay;

  const float minDegree = 30.0;
  const float maxDegree = 150.0;
  float degreeRange = maxDegree - minDegree;
  float degreeIncrement = (degreeRange / totalMinutesInDay) * currentMinutesInDay;

  return round(minDegree + degreeIncrement);
}


void updateSunPosition() {
  if (currentMode == "Sun Tracking") {
    int position = calculateSunPosition();
    myservo.write(position); 
    currentDegree = position;
  }
}

void setup() {
  Serial.begin(115200); // Start Serial communication for debugging
  pinMode(2, OUTPUT); // Set GPIO2 as output
  digitalWrite(2, HIGH); // Turn off the LED initially

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("WiFi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  
  timeClient.begin();
  timeClient.setTimeOffset(timeOffset);
  timeClient.update();

 // Define routes for the web server
  server.on("/", HTTP_GET, handleRoot);
  server.on("/servo/closed",positionClose);
  server.on("/servo/open", positionOpen);
  server.on("/servo/manual", HTTP_GET, []() {
    if (server.hasArg("degree")) {
      int degree = server.arg("degree").toInt(); // Get the degree from query string
      positionManual(degree);  // Call the open function with the degree value
    } else {
      server.send(400, "text/plain", "Degree not provided");
    }
  });
  server.on("/servo/mode", getMode);
  server.on("/servo/degree", getCurrentDegree);
  server.on("/servo/setManual", setPositionManual);
  server.on("/servo/setSunTracking", setSunTracking);
  
  server.begin();  // Start the server
  Serial.println("HTTP server started");

  myservo.attach(13); // Change the pin if needed
  myservo.write(90); // Set initial servo position
}

void loop() {
  server.handleClient(); // Handle incoming HTTP requests
}
