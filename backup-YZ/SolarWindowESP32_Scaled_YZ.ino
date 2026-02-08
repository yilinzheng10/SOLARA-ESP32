#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <NTPClient.h>
#include <ESP32Servo.h>
#include <Ticker.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <WiFiClientSecure.h>

// INA226 I2C address (default is 0x40)
#define INA226_ADDRESS 0x40

// INA226 Register addresses
#define INA226_REG_CONFIG 0x00
#define INA226_REG_SHUNT_VOLTAGE 0x01
#define INA226_REG_BUS_VOLTAGE 0x02
#define INA226_REG_POWER 0x03
#define INA226_REG_CURRENT 0x04
#define INA226_REG_CALIBRATION 0x05
#define CURRENT_LSB 0.00005
#define CALIBRATION 51200

// Configuration values
#define INA226_CONFIG_DEFAULT 0x4127
#define SHUNT_RESISTANCE 0.002
#define MAX_CURRENT 0.5

// Electronic Load Configuration
#define PWM_PIN 25              // GPIO pin for MOSFET control
#define PWM_FREQ 25000          // 25kHz PWM frequency
#define PWM_RESOLUTION 8        // 8-bit resolution (0-255)


// BLE UUIDs
#define SERVICE_UUID        "91bad492-b950-4226-aa2b-4ede9fa42f59"
#define WIFI_CRED_UUID      "cba1d466-344c-4be3-ab3f-189f80dd7518"
#define IP_ADDR_UUID        "12345678-1234-5678-1234-56789abcdef0" 

Preferences preferences;
WebServer server(80);
WiFiUDP udp;
NTPClient timeClient(udp, "pool.ntp.org", 0, 3600); // Will adjust offset later

Servo myservo;
Ticker sunTrackingTicker;

//11/30 update
String currentMode = "Closed";
int currentDegree = 90;
const int minDegree = 0;
const int maxDegree = 180;
bool wifiConnected = false;
// Prevent overlapping servo movements (global lock)
bool servoBusy = false;

// Manual mode state machine
unsigned long lastManualStep = 0;
const int manualStepDelay = 12; // ms per degree

// String currentMode = "Closed";
// int currentDegree = 90;
// const int minDegree = 0;
// const int maxDegree = 180;
// bool wifiConnected = false;

// Global variables for measurements
float voltage = 0;
float current = 0;
float power = 0;
float energy = 0;
unsigned long lastTime = 0;
unsigned long startTime = 0;

<<<<<<< HEAD:SolarWindowESP32/SolarWindowESP32.ino
//11/27 update smooth movement
//manual
bool manualActive = false;
int manualTarget = -1;
unsigned long lastManualCmdTime = 0;
const int manualDeadband = 2;         // ignore <2° change
const unsigned long manualMinInterval = 80; // ms between commands
void smoothMove(int fromDeg, int toDeg, int stepDelay);
=======
// MPPT variables
int currentPWM = 0;
bool mpptEnabled = false;
float maxPowerVoltage = 0;
float maxPowerCurrent = 0;
float maxPower = 0;
int optimalPWM = 0;
>>>>>>> b70fcf495414aee971d17357b72ff9b040d1c731:SolarWindowESP32.ino

BLEServer* pServer = nullptr;
BLECharacteristic* wifiCredCharacteristic = nullptr;
BLECharacteristic* ipAddrCharacteristic = nullptr;
void connectToWiFi(const char* ssid, const char* password);

// --- Pipedream & Location Settings ---
const char* PIPEDREAM_URL = "https://eogaas8mnfwdl7z.m.pipedream.net";

// Your Location (Used for both Sun tracking and Data logging)
const float DEVICE_LAT = 30.030474;
const float DEVICE_LNG = -90.241234;

// Upload Timer (e.g., 5 minutes = 300000 ms)
unsigned long lastUploadTime = 0;
const unsigned long UPLOAD_INTERVAL = 100000;

class WiFiCredsCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    std::string value = std::string(pCharacteristic->getValue().c_str());
    Serial.print("Received BLE data: ");
    Serial.println(value.c_str());

    if (value.length() > 0) {
      const size_t capacity = JSON_OBJECT_SIZE(2) + 60;
      StaticJsonDocument<capacity> doc;
      DeserializationError error = deserializeJson(doc, value.c_str());

      if (error) {
        Serial.print("JSON parsing failed: ");
        Serial.println(error.c_str());
        return;
      }

      String ssid = String((const char*)doc["ssid"]);
      String password = String((const char*)doc["password"]);

      Serial.println("Parsed SSID: " + ssid);
      Serial.println("Parsed Password: " + password);

      preferences.begin("wifi", false);
      preferences.putString("ssid", ssid);
      preferences.putString("password", password);
      preferences.end();

      connectToWiFi(ssid.c_str(), password.c_str());
    }
  }
};

void connectToWiFi(const char* ssid, const char* password) {
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    delay(1000);
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    wifiConnected = true;

    // Update IP BLE characteristic and notify
    if (ipAddrCharacteristic) {
      String ipStr = WiFi.localIP().toString();
      Serial.println(ipStr);
      ipAddrCharacteristic->setValue(ipStr.c_str());
      ipAddrCharacteristic->notify();
      Serial.println("Updated IP address characteristic");
    }

    startWebServer();
  } else {
    Serial.println("\nFailed to connect to WiFi.");
    wifiConnected = false;
  }
}


void startBLEServer() {
  uint64_t chipid = ESP.getEfuseMac();
  char name[30];
  snprintf(name, sizeof(name), "ESP32-%04X", (uint16_t)(chipid >> 32));
  BLEDevice::init(name);

  pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  wifiCredCharacteristic = pService->createCharacteristic(
    WIFI_CRED_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  wifiCredCharacteristic->setCallbacks(new WiFiCredsCallback());

  ipAddrCharacteristic = pService->createCharacteristic(
    IP_ADDR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  ipAddrCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  Serial.println("BLE advertising started");
}

void uploadToPipedream() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Skipping upload: WiFi not connected");
    return;
  }

  Serial.println("Preparing data upload...");

  // 1. Get formatted time
  timeClient.update();
  unsigned long epochTime = timeClient.getEpochTime();
  struct tm *ptm = gmtime((time_t *)&epochTime);
  char timeString[25];
  sprintf(timeString, "%04d-%02d-%02dT%02d:%02d:%02dZ", 
          ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday, 
          ptm->tm_hour, ptm->tm_min, ptm->tm_sec);

  // 2. Create the JSON Document
  // We use a larger buffer to hold all the data
  StaticJsonDocument<512> doc;

  doc["device_id"] = "solara_esp32_v1";
  doc["timestamp"] = timeString;
  
  // Location Data
  JsonObject location = doc.createNestedObject("location");
  location["lat"] = DEVICE_LAT;
  location["lng"] = DEVICE_LNG;

  // Energy Data (These variables must match your global ones)
  JsonObject data = doc.createNestedObject("energy_data");
  data["voltage_V"] = voltage;
  data["current_A"] = current;
  data["power_W"] = power;
  data["energy_Wh"] = energy; // Accumulated energy
  data["pwm_value"] = currentPWM;
  data["mppt_status"] = mpptEnabled ? "ON" : "OFF";

  // 3. Convert JSON to String
  String payload;
  serializeJson(doc, payload);

  // 4. Send HTTPS POST to Pipedream
  WiFiClientSecure client;
  client.setInsecure(); // Skip certificate check (crucial for Pipedream)
  
  HTTPClient http;
  http.begin(client, PIPEDREAM_URL);
  http.addHeader("Content-Type", "application/json");
  
  int httpResponseCode = http.POST(payload);
  
  if (httpResponseCode > 0) {
    Serial.print("Data sent! Response: ");
    Serial.println(httpResponseCode);
  } else {
    Serial.print("Upload failed. Error: ");
    Serial.println(http.errorToString(httpResponseCode));
  }
  
  http.end();
}

void requestSunInfo() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected!");
    return;
  }

  WiFiClientSecure client;
  client.setInsecure();  // Skip certificate validation (for testing only)
  
  HTTPClient http;
  
  // Use HTTPS URL with WiFiClientSecure
  String sunUrl = "https://api.sunrise-sunset.org/json?lat=" + String(DEVICE_LAT, 6) + 
                "&lng=" + String(DEVICE_LNG, 6) + "&date=today&tzid=America/Chicago";
  http.begin(client, sunUrl);
  http.setTimeout(10000);
  
  Serial.println("Making secure API request...");
  int httpResponseCode = http.GET();
  
  Serial.print("HTTPS Response Code: ");
  Serial.println(httpResponseCode);
  
  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println("Response:");
    Serial.println(response);
  } else {
    Serial.print("HTTPS request failed with error: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}

void smoothServoSweep(int startAngle, int midAngle, int endAngle, int delayMs = 90) {
    // Move from start → mid
    if (startAngle < midAngle) {
        for (int pos = startAngle; pos <= midAngle; pos++) {
            myservo.write(pos);
            delay(delayMs);
        }
    } else {
        for (int pos = startAngle; pos >= midAngle; pos--) {
            myservo.write(pos);
            delay(delayMs);
        }
    }

    // Pause briefly at peak
    delay(300);

    // Move from mid → end
    if (midAngle < endAngle) {
        for (int pos = midAngle; pos <= endAngle; pos++) {
            myservo.write(pos);
            delay(delayMs);
        }
    } else {
        for (int pos = midAngle; pos >= endAngle; pos--) {
            myservo.write(pos);
            delay(delayMs);
        }
    }
}


void startWebServer() {
  timeClient.begin();
  timeClient.setTimeOffset(-6 * 3600);
  timeClient.update();

  myservo.attach(13);
  myservo.write(90);

  server.on("/", HTTP_GET, []() {
    server.send(200, "text/plain", "ESP32 is connected!");
  });

<<<<<<< HEAD:SolarWindowESP32/SolarWindowESP32.ino
  // server.on("/servo/closed", []() {
  //   stopSunTracking();
  //   currentMode = "Closed";
  //   currentDegree = 90;
  //   myservo.write(90);
  //   Serial.println(currentDegree);
  //   server.send(200, "text/plain", "Servo is 90 degrees");
  // });

 server.on("/servo/closed", []() {
=======
  server.on("/energy_data", []() {
    String json = "{";
    json += "\"voltage\":" + String(voltage, 3) + ",";
    json += "\"current\":" + String(current, 3) + ",";
    json += "\"power\":" + String(power, 3) + ",";
    json += "\"energy\":" + String(energy, 4) + ",";
    json += "\"pwm\":" + String(currentPWM) + ",";
    json += "\"pwm_percent\":" + String((currentPWM*100)/255) + ",";
    json += "\"mppt_enabled\":" + String(mpptEnabled ? "true" : "false") + ",";
    json += "\"max_power\":" + String(maxPower, 3) + ",";
    json += "\"optimal_pwm\":" + String(optimalPWM);
    json += "}";
    server.send(200, "application/json", json);
  });

  server.on("/reset_energy", []() {
    energy = 0;
    startTime = millis();
    server.send(200, "text/plain", "Energy counter reset");
  });

  server.on("/set_pwm", []() {
    if (server.hasArg("value")) {
      int pwm = server.arg("value").toInt();
      if (pwm >= 0 && pwm <= 255) {
        mpptEnabled = false;
        currentPWM = pwm;
        ledcWrite(PWM_PIN, currentPWM);
        String response = "PWM set to " + String(pwm) + " (" + String((pwm*100)/255) + "%)";
        server.send(200, "text/plain", response);
      } else {
        server.send(400, "text/plain", "PWM must be 0-255");
      }
    } else {
      server.send(400, "text/plain", "Missing 'value' parameter. Example: /set_pwm?value=128");
    }
  });

  server.on("/mppt_sweep", []() {
    performMPPTSweep();
    String response = "MPPT Sweep Complete!\n\n";
    response += "Optimal PWM: " + String(optimalPWM) + " (" + String((optimalPWM*100)/255) + "%)\n";
    response += "Max Power: " + String(maxPower, 3) + " W\n";
    response += "At Voltage: " + String(maxPowerVoltage, 3) + " V\n";
    response += "At Current: " + String(maxPowerCurrent, 3) + " A\n\n";
    if (maxPowerCurrent > 0.001) {
      float equivalentR = maxPowerVoltage / maxPowerCurrent;
      response += "Equivalent Load Resistance: " + String(equivalentR, 1) + " Ohms\n";
    }
    response += "\nCheck Serial Monitor for detailed sweep data.";
    server.send(200, "text/plain", response);
  });

  server.on("/mppt_enable", []() {
    mpptEnabled = true;
    server.send(200, "text/plain", "Auto MPPT tracking enabled");
  });

  server.on("/mppt_disable", []() {
    mpptEnabled = false;
    server.send(200, "text/plain", "Auto MPPT tracking disabled");
  });

  server.on("/reset_mppt", []() {
    maxPower = 0;
    maxPowerVoltage = 0;
    maxPowerCurrent = 0;
    optimalPWM = 0;
    server.send(200, "text/plain", "MPPT tracking data reset");
  });

  server.on("/servo/closed", []() {
>>>>>>> b70fcf495414aee971d17357b72ff9b040d1c731:SolarWindowESP32.ino
    stopSunTracking();
    manualActive = false;  
    currentMode = "Closed";

    server.send(200, "text/plain", "Servo is 0 degrees");

    //*** 11/27 update smooth movement
    int target = 100;
    smoothMove(currentDegree, target, 15);
    //**
  });

  // server.on("/servo/open", []() {
  //   stopSunTracking();
  //   currentMode = "Open";
  //   currentDegree = 135;
  //   myservo.write(135);
  //   server.send(200, "text/plain", "Servo is 135 degrees");
  //   requestSunInfo();
    
  // });

  server.on("/wifi/reset", []() {
    resetWiFiCredentials();
    server.send(200, "text/plain", "WiFi credentials cleared. BLE provisioning active.");
  });

  // server.on("/servo/demo", []() {
  //   stopSunTracking();
  //   currentMode = "Demo";

  //   // Smoothly sweep from 90 -> 135 -> 90
  //   smoothServoSweep(90, 130, 90);

  //   currentDegree = 0; // End position
  //   server.send(200, "text/plain", "Servo performed smooth demo sweep");
  // });

    server.on("/servo/demo", []() {
    stopSunTracking();
    manualActive = false;  
    currentMode = "Open";
    //*** 11/27 update smooth movement
    int target = 135;
    smoothMove(currentDegree, target, 15);
    //**
    server.send(200, "text/plain", "Servo is 45 degrees");
    requestSunInfo();
  });

  // server.on("/servo/manual", HTTP_GET, []() {
  //   if (server.hasArg("degree")) {
  //     int degree = server.arg("degree").toInt();
  //     stopSunTracking();
  //     currentMode = "Manual";
  //     myservo.write(degree);
  //     currentDegree = degree;
  //     server.send(200, "text/plain", "Servo set to " + String(degree));
  //   } else {
  //     server.send(400, "text/plain", "Degree not provided");
  //   }
  // });

    server.on("/servo/manual", HTTP_GET, []() {
    if (!server.hasArg("degree")) {
        server.send(400, "text/plain", "degree missing");
        return;
    }

    int degree = server.arg("degree").toInt();
    degree = constrain(degree, 0, 180);

    stopSunTracking();
    currentMode = "Manual";

    unsigned long now = millis();

    // 1) Debounce: ignore too frequent updates
    if (now - lastManualCmdTime < manualMinInterval) {
        server.send(200, "text/plain", "ignored (too fast)");
        return;
    }
    lastManualCmdTime = now;

    // 2) Deadband: ignore tiny changes
    if (abs(degree - currentDegree) < manualDeadband) {
        server.send(200, "text/plain", "ignored (within deadband)");
        return;
    }

    // 3) Set smooth-move target
    manualTarget = degree;
    manualActive = true;

    server.send(200, "text/plain", "target set to " + String(degree));
});

  server.on("/servo/apicall", []() {
    requestSunInfo();
  });

  server.on("/servo/mode", []() {
    server.send(200, "text/plain", currentMode);
  });

  server.on("/servo/degree", []() {
    server.send(200, "text/plain", String(currentDegree));
  });

  server.on("/servo/setManual", []() {
    stopSunTracking();
    currentMode = "Manual";
    server.send(200, "text/plain", "Manual mode set");
  });

  server.on("/servo/setSunTracking", []() {
    currentMode = "Sun Tracking";
    startSunTracking();
    server.send(200, "text/plain", "Sun Tracking mode set");
  });

  server.begin();
  Serial.println("HTTP server started");
}

void startSunTracking() {
  sunTrackingTicker.attach(60, updateSunPosition); 
}

void stopSunTracking() {
  sunTrackingTicker.detach(); 
}

int calculateSunPosition() {
  if (timeClient.update()) {
    unsigned long epochTime = timeClient.getEpochTime();
    struct tm* timeinfo = localtime((time_t*)&epochTime);
    int hour = timeinfo->tm_hour;
    int minute = timeinfo->tm_min;

    const int totalMinutesInDay = 12 * 60;
    int currentMinutes = (hour - 6) * 60 + minute;
    if (currentMinutes < 0) currentMinutes = 0;
    if (currentMinutes > totalMinutesInDay) currentMinutes = totalMinutesInDay;

    float degreeRange = 150.0 - 30.0;
    float position = 30.0 + (degreeRange * currentMinutes) / totalMinutesInDay;
    return round(position);
  } else {
    Serial.println("NTP update failed");
    return currentDegree;
  }
}

void updateSunPosition() {
  if (currentMode == "Sun Tracking") {
    int position = calculateSunPosition();
    myservo.write(position);
    currentDegree = position;
    Serial.println("Sun position updated: " + String(position));
  }
}

<<<<<<< HEAD:SolarWindowESP32/SolarWindowESP32.ino
bool initINA226() {
  // Check if device is present
  Wire.beginTransmission(INA226_ADDRESS);
  if (Wire.endTransmission() != 0) {
    return false;
  }
  
  // Configure INA226
  writeRegister(INA226_REG_CONFIG, INA226_CONFIG_DEFAULT);
  
  // Calculate calibration value
  // Cal = 0.00512 / (Current_LSB * Rshunt)
  // Current_LSB = Max_Current / 32768
  // float currentLSB = MAX_CURRENT / 32768.0;
  // uint16_t cal = (uint16_t)(0.00512 / (currentLSB * SHUNT_RESISTANCE));
  

  
  writeRegister(INA226_REG_CALIBRATION, CALIBRATION);
  
  delay(100);
  return true;
}

//***11/27 update: smooth servo movement***
void smoothMove(int fromDeg, int toDeg, int stepDelay) {
  if (servoBusy) return;      // Prevent overlap

  servoBusy = true;           // LOCK servo

  int step = (fromDeg < toDeg) ? 1 : -1;
  for (int pos = fromDeg; pos != toDeg; pos += step) {
    myservo.write(pos);
    delay(stepDelay);
  }

  myservo.write(toDeg);
  delay(20);

  currentDegree = toDeg;
  servoBusy = false;          // UNLOCK servo
}
//***

void readINA226() {
  // Read bus voltage (solar panel voltage)
  uint16_t busVoltageRaw = readRegister(INA226_REG_BUS_VOLTAGE);
  voltage = (busVoltageRaw * 1.25) / 1000.0; // Convert to volts
  
  // Read current
  int16_t currentRaw = (int16_t)readRegister(INA226_REG_CURRENT);
  Serial.println(currentRaw);
  // float currentLSB = MAX_CURRENT / 32768.0;
  current = currentRaw * CURRENT_LSB; // amps

  
  // Read power (or calculate it)
  uint16_t powerRaw = readRegister(INA226_REG_POWER);
  float powerLSB = CURRENT_LSB * 25;
  power = powerRaw * powerLSB; // Power in Watts
  
  // Alternative: calculate power manually
  // power = voltage * current;
}

void calculateEnergy() {
  // Calculate energy in Wh (Watt-hours)
  static unsigned long previousTime = 0;
  unsigned long currentTime = millis();
  
  if (previousTime > 0) {
    float timeHours = (currentTime - previousTime) / 3600000.0;
    energy += power * timeHours;
  }
  
  previousTime = currentTime;
}

void printMeasurements() {
  // Calculate runtime
  unsigned long runtime = (millis() - startTime) / 1000; // in seconds
  int hours = runtime / 3600;
  int minutes = (runtime % 3600) / 60;
  int seconds = runtime % 60;
  
  //Clear previous output (optional - comment out if you want scrolling data)
  Serial.print("\033[2J\033[H");
  
  Serial.println("=== Solar Panel Real-Time Data ===");
  Serial.print("Runtime: ");
  Serial.printf("%02d:%02d:%02d\n", hours, minutes, seconds);
  Serial.println();
  
  Serial.print("🔋 Voltage:  ");
  Serial.print(voltage, 3);
  Serial.println(" V");
  
  Serial.print("⚡ Current:  ");
  Serial.print(current, 3);
  Serial.println(" A");
  
  Serial.print("💡 Power:    ");
  Serial.print(power, 3);
  Serial.println(" W");
  
  Serial.print("📊 Energy:   ");
  Serial.print(energy, 4);
  Serial.println(" Wh");
  
  // LED Diagnostics
  Serial.println();
  // Serial.println("--- LED Diagnostics ---");
  // if (voltage < 1.8) {
  //   Serial.println("⚠ Voltage too low for any LED");
  //   Serial.println("  → Try brighter light on solar panel");
  // } else if (voltage < 2.2) {
  //   Serial.println("✓ Voltage OK for RED LED only");
  // } else if (voltage < 3.0) {
  //   Serial.println("✓ Voltage OK for RED/GREEN/YELLOW LEDs");
  // } else {
  //   Serial.println("✓ Voltage OK for all LED colors");
  // }
  
  // if (current < 0.001) {
  //   Serial.println("⚠ No current flow - check connections");
  //   Serial.println("  → Verify LED polarity (long leg = +)");
  //   Serial.println("  → Try LED without resistor first");
  // } else if (current < 0.005) {
  //   Serial.println("⚠ Very low current (<5mA) - LED won't light");
  //   Serial.println("  → Solar panel may be too weak");
  // } else if (current < 0.015) {
  //   Serial.println("⚠ Low current (5-15mA) - LED may be very dim");
  // } else {
  //   Serial.println("✓ Good current flow (>15mA)");
  // }
  
  // Solar panel capability assessment
  float estimatedPower = voltage * current * 1000; // in mW
  Serial.println();
  Serial.print("Solar Panel Assessment: ");
  Serial.print(estimatedPower, 1);
  Serial.println(" mW available");
  
  // if (estimatedPower < 50) {
  //   Serial.println("⚠ Solar panel too weak for LED");
  //   Serial.println("  → Need brighter light or bigger panel");
  // } else {
  //   Serial.println("✓ Solar panel should be able to drive LED");
  // }
  
  // // Performance indicators
  // Serial.println();
  // if (power > 0) {
  //   Serial.print("Status: ✓ GENERATING (");
  //   if (power > 10) Serial.print("High");
  //   else if (power > 5) Serial.print("Medium");
  //   else Serial.print("Low");
  //   Serial.println(" output)");
  // } else {
  //   Serial.println("Status: ⚠ NO OUTPUT");
  // }
  
  Serial.println("===================================");
  Serial.println();
}

=======
>>>>>>> b70fcf495414aee971d17357b72ff9b040d1c731:SolarWindowESP32.ino
void resetWiFiCredentials() {
  Serial.println("Resetting WiFi credentials...");

  // Erase stored SSID and password
  preferences.begin("wifi", false);  // write mode
  preferences.remove("ssid");
  preferences.remove("password");
  preferences.end();

  // Disconnect WiFi if currently connected
  WiFi.disconnect(true);  // erase runtime credentials
  wifiConnected = false;

  Serial.println("WiFi credentials removed.");

  // Start BLE provisioning again
  startBLEServer();
  Serial.println("BLE server started. Ready for new WiFi setup.");
}

bool initINA226() {
  Serial.println("\n=== INA226 Initialization ===");
  
  Wire.beginTransmission(INA226_ADDRESS);
  byte error = Wire.endTransmission();
  
  Serial.print("I2C Address 0x");
  Serial.print(INA226_ADDRESS, HEX);
  Serial.print(" - ");
  
  if (error != 0) {
    Serial.print("ERROR: ");
    if (error == 2) Serial.println("NACK on address (device not found)");
    else if (error == 5) Serial.println("Timeout");
    else Serial.println("Unknown error");
    return false;
  }
  
  Serial.println("Device found!");
  
  Serial.println("Resetting INA226...");
  writeRegister(INA226_REG_CONFIG, 0x8000);
  delay(10);
  
  Serial.println("Configuring INA226...");
  writeRegister(INA226_REG_CONFIG, INA226_CONFIG_DEFAULT);
  delay(10);
  
  Serial.print("Setting calibration to: ");
  Serial.println(CALIBRATION);
  writeRegister(INA226_REG_CALIBRATION, CALIBRATION);
  delay(10);
  
  uint16_t configRead = readRegister(INA226_REG_CONFIG);
  Serial.print("Config register readback: 0x");
  Serial.println(configRead, HEX);
  
  uint16_t calRead = readRegister(INA226_REG_CALIBRATION);
  Serial.print("Calibration register readback: 0x");
  Serial.println(calRead, HEX);
  
  Serial.println("=== Initialization Complete ===\n");
  delay(100);
  return true;
}

void readINA226() {
  uint16_t busVoltageRaw = readRegister(INA226_REG_BUS_VOLTAGE);
  voltage = (busVoltageRaw * 1.25) / 1000.0;
  
  int16_t shuntVoltageRaw = (int16_t)readRegister(INA226_REG_SHUNT_VOLTAGE);
  float shuntVoltage = shuntVoltageRaw * 2.5 / 1000000.0;
  
  int16_t currentRaw = (int16_t)readRegister(INA226_REG_CURRENT);
  current = currentRaw * CURRENT_LSB;
  
  uint16_t powerRaw = readRegister(INA226_REG_POWER);
  float powerLSB = CURRENT_LSB * 25;
  power = powerRaw * powerLSB;
  
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 10000) {
    // Serial.println("\n--- INA226 Raw Register Values ---");
    // Serial.print("Bus Voltage Raw: 0x");
    // Serial.print(busVoltageRaw, HEX);
    // Serial.print(" (");
    // Serial.print(busVoltageRaw);
    // Serial.println(")");
    
    // Serial.print("Shunt Voltage Raw: 0x");
    // Serial.print(shuntVoltageRaw, HEX);
    // Serial.print(" (");
    // Serial.print(shuntVoltageRaw);
    // Serial.print(") = ");
    // Serial.print(shuntVoltage * 1000000, 2);
    // Serial.println(" µV");
    
    // Serial.print("Current Raw: 0x");
    // Serial.print(currentRaw, HEX);
    // Serial.print(" (");
    // Serial.print(currentRaw);
    // Serial.println(")");
    
    // Serial.print("Power Raw: 0x");
    // Serial.print(powerRaw, HEX);
    // Serial.print(" (");
    // Serial.print(powerRaw);
    // Serial.println(")");
    // Serial.println("----------------------------------\n");
    
    lastDebug = millis();
  }
}

void calculateEnergy() {
  static unsigned long previousMicros = 0;
  unsigned long currentMicros = micros();
  
  if (previousMicros == 0) {
    previousMicros = currentMicros;
    return;
  }
  
  float timeHours = (currentMicros - previousMicros) / 3600000000.0;
  
  if (power > 0) {
    energy += power * timeHours;
  }
  
  previousMicros = currentMicros;
}

void printMeasurements() {
  unsigned long runtime = (millis() - startTime) / 1000;
  int hours = runtime / 3600;
  int minutes = (runtime % 3600) / 60;
  int seconds = runtime % 60;
  
  Serial.print("\033[2J\033[H");
  
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║    ESP32 SOLAR ENERGY MONITOR          ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.print("Runtime: ");
  Serial.printf("%02d:%02d:%02d\n", hours, minutes, seconds);
  Serial.println();
  
  Serial.print("🔋 Voltage:  ");
  Serial.print(voltage, 3);
  Serial.println(" V");
  
  Serial.print("⚡ Current:  ");
  Serial.print(current, 3);
  Serial.println(" A");
  
  Serial.print("💡 Power:    ");
  Serial.print(power, 3);
  Serial.println(" W");
  
  Serial.print("📊 Energy:   ");
  Serial.print(energy, 4);
  Serial.println(" Wh");
  
  Serial.println("\n--- Electronic Load Status ---");
  Serial.print("PWM Load: ");
  Serial.print(currentPWM);
  Serial.print("/255 (");
  Serial.print((currentPWM * 100) / 255);
  Serial.println("%)");
  
  Serial.print("Auto MPPT: ");
  Serial.println(mpptEnabled ? "ENABLED ✓" : "DISABLED");
  
  if (maxPower > 0) {
    Serial.println("\n--- Maximum Power Point ---");
    Serial.print("Best Power: ");
    Serial.print(maxPower, 3);
    Serial.println(" W");
    Serial.print("At PWM: ");
    Serial.print(optimalPWM);
    Serial.print(" (");
    Serial.print((optimalPWM * 100) / 255);
    Serial.println("%)");
  }
  
  float estimatedPower = voltage * current * 1000;
  Serial.println("\n--- Assessment ---");
  Serial.print("Instant Power: ");
  Serial.print(estimatedPower, 1);
  Serial.println(" mW");
  
  if (voltage < 0.5) {
    Serial.println("\n⚠️  WARNING: Voltage too low!");
    Serial.println("   Check power source connection");
  } else if (voltage > 4.5 && voltage < 5.5) {
    Serial.println("\n✓ Voltage good (5V source)");
  }
  
  if (current > 0.05) {
    float resistorValue = voltage / current;
    float powerDissipation = current * current * resistorValue;
    
    Serial.println();
    Serial.print("Effective Load: ~");
    Serial.print(resistorValue, 1);
    Serial.println(" Ω");
    
    Serial.print("Load Power: ");
    Serial.print(powerDissipation, 3);
    Serial.println(" W");
    
    if (powerDissipation > 1.0) {
      Serial.println("⚠️  High power - ensure MOSFET has heatsink!");
    }
  }
  
  Serial.println("\n═══════════════════════════════════════");
  Serial.println();
}

void writeRegister(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(INA226_ADDRESS);
  Wire.write(reg);
  Wire.write((value >> 8) & 0xFF);
  Wire.write(value & 0xFF);
  Wire.endTransmission();
}

uint16_t readRegister(uint8_t reg) {
  Wire.beginTransmission(INA226_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  
  Wire.requestFrom(INA226_ADDRESS, 2);
  uint16_t value = 0;
  
  if (Wire.available() == 2) {
    value = Wire.read() << 8;
    value |= Wire.read();
  }
  
  return value;
}

void performMPPTSweep() {
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║   MPPT SWEEP - FINDING OPTIMAL LOAD   ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  
  maxPower = 0;
  maxPowerVoltage = 0;
  maxPowerCurrent = 0;
  optimalPWM = 0;
  
  Serial.println("Sweeping PWM from 0% to 100%...\n");
  
  for (int pwm = 0; pwm <= 255; pwm += 13) {
    ledcWrite(PWM_PIN, pwm);
    delay(300);
    
    readINA226();
    
    Serial.print("PWM: ");
    Serial.print(pwm);
    Serial.print(" (");
    Serial.print((pwm * 100) / 255);
    Serial.print("%)");
    Serial.print(" → V: ");
    Serial.print(voltage, 3);
    Serial.print("V, I: ");
    Serial.print(current, 3);
    Serial.print("A, P: ");
    Serial.print(power, 3);
    Serial.print("W");
    
    if (power > maxPower) {
      maxPower = power;
      maxPowerVoltage = voltage;
      maxPowerCurrent = current;
      optimalPWM = pwm;
      Serial.print(" ← ★ NEW MAXIMUM!");
    }
    Serial.println();
  }
  
  currentPWM = optimalPWM;
  ledcWrite(PWM_PIN, currentPWM);
  
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║           MPPT RESULTS                 ║");
  Serial.println("╠════════════════════════════════════════╣");
  Serial.print("║ Optimal PWM:    ");
  Serial.print(optimalPWM);
  Serial.print(" (");
  Serial.print((optimalPWM * 100) / 255);
  Serial.println("%)");
  Serial.print("║ Max Power:      ");
  Serial.print(maxPower, 3);
  Serial.println(" W");
  Serial.print("║ At Voltage:     ");
  Serial.print(maxPowerVoltage, 3);
  Serial.println(" V");
  Serial.print("║ At Current:     ");
  Serial.print(maxPowerCurrent, 3);
  Serial.println(" A");
  
  if (maxPowerCurrent > 0.001) {
    float equivalentR = maxPowerVoltage / maxPowerCurrent;
    Serial.print("║ Equiv. Load:    ");
    Serial.print(equivalentR, 1);
    Serial.println(" Ω");
  }
  Serial.println("╚════════════════════════════════════════╝\n");
  
  Serial.println("Load automatically set to optimal PWM.");
  Serial.println("Use /mppt_enable to track changes automatically.\n");
}

void performAutoMPPT() {
  static float lastPower = 0;
  static int perturbDirection = 1;
  static unsigned long lastMPPTTime = 0;
  
  if (millis() - lastMPPTTime < 3000) return;
  lastMPPTTime = millis();
  
  if (power > lastPower) {
    currentPWM += perturbDirection * 3;
  } else {
    perturbDirection = -perturbDirection;
    currentPWM += perturbDirection * 3;
  }
  
  if (currentPWM < 0) currentPWM = 0;
  if (currentPWM > 255) currentPWM = 255;
  
  ledcWrite(PWM_PIN, currentPWM);
  lastPower = power;
  
  Serial.print("⚡ Auto MPPT: PWM=");
  Serial.print(currentPWM);
  Serial.print(" (");
  Serial.print((currentPWM * 100) / 255);
  Serial.print("%), P=");
  Serial.print(power, 3);
  Serial.println("W");
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("Scanning I2C bus...");
  Wire.begin();
  byte deviceCount = 0;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      deviceCount++;
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("ERROR: No I2C devices found!");
    Serial.println("Check INA226 wiring!");
  } else {
    Serial.print("Found ");
    Serial.print(deviceCount);
    Serial.println(" I2C device(s)\n");
  }

  // ---- Load stored WiFi credentials ----
  preferences.begin("wifi", true);   // read-only
  String ssid = preferences.getString("ssid", "");
  String password = preferences.getString("password", "");
  preferences.end();

  Serial.println("Stored SSID: " + ssid);

  // ---- Case 1: We have saved credentials ----
  if (ssid.length() > 0 && password.length() > 0) {
    Serial.println("Connecting with saved WiFi credentials...");
    connectToWiFi(ssid.c_str(), password.c_str());

    // If connection succeeds, no BLE needed
    if (wifiConnected) {
      Serial.println("Connected to WiFi!");
      pinMode(2, OUTPUT);
      digitalWrite(2, HIGH);
      return;
    }

    Serial.println("Failed to connect with saved WiFi. Starting BLE provisioning...");
  }

  // ---- Case 2: No saved credentials OR connection failed ----
  startBLEServer();
  Serial.println("BLE server started. Waiting for WiFi credentials...");

<<<<<<< HEAD:SolarWindowESP32/SolarWindowESP32.ino
  //****INA226
  // Serial.begin(115200);
  // Wire.begin();
  // preferences.begin("wifi", false);
  // preferences.remove("ssid");
  // preferences.remove("password");
  // preferences.end();

  // preferences.begin("wifi", true);
  // String ssid = preferences.getString("ssid", "");
  // String password = preferences.getString("password", "");
  // preferences.end();

  // if (ssid.length() > 0 && password.length() > 0) {
  //   connectToWiFi(ssid.c_str(), password.c_str());
  // }

  // if (!wifiConnected) {
  //   Serial.println("BLE server started");
  //   startBLEServer();
  // }

  // if (initINA226()) {
  //   Serial.println("✓ INA226 initialized successfully");
  // } else {
  //   Serial.println("✗ Failed to initialize INA226");
  //   Serial.println("Check wiring and I2C connections");
  //   return;
  // }

  // lastTime = millis();
  // startTime = millis();
  //****
=======
  if (initINA226()) {
    Serial.println("✓ INA226 initialized successfully\n");
  } else {
    Serial.println("✗ Failed to initialize INA226");
    Serial.println("System halted - fix INA226 connection\n");
    while(1) {
      delay(1000);
    }
  }
    // Initialize PWM - compatible with ESP32 Core 3.x
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(PWM_PIN, 0);
  
  lastTime = millis();
  startTime = millis();
>>>>>>> b70fcf495414aee971d17357b72ff9b040d1c731:SolarWindowESP32.ino

  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  Serial.println("✓ System ready! Starting measurements...\n");
}

void loop() {
  if (wifiConnected) {
    server.handleClient();
  }

<<<<<<< HEAD:SolarWindowESP32/SolarWindowESP32.ino
  // Handle smooth manual move in the background
  if (manualActive) {
    unsigned long now = millis();
    if (now - lastManualStep >= manualStepDelay) {
      if (currentDegree != manualTarget) {
        int step = (manualTarget > currentDegree) ? 1 : -1;
        currentDegree += step;
        myservo.write(currentDegree);
      } else {
        manualActive = false; // reached target
      }
      lastManualStep = now;
    }
  }

//****INA226
  if (millis() - lastTime >= 60000) {
=======
  unsigned long currentMillis = millis();

  // 1. Existing Measurement Logic (Runs every 1 second)
  if (currentMillis - lastTime >= 1000) {
>>>>>>> b70fcf495414aee971d17357b72ff9b040d1c731:SolarWindowESP32.ino
    readINA226();
    calculateEnergy();
    if (mpptEnabled) {
      performAutoMPPT();
    }
    // printMeasurements();
    lastTime = currentMillis;
  }

  if (wifiConnected && (currentMillis - lastUploadTime >= UPLOAD_INTERVAL)) {
    uploadToPipedream();
    lastUploadTime = currentMillis;
  }
//***
}

// manual reset wifi if it is not cleared from device
// http://ip address/wifi/reset

