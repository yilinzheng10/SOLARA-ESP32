// #include <WiFi.h>
// #include <WebServer.h>
// #include <Preferences.h>
// #include <Wire.h>
// #include <ArduinoJson.h>

// // INA226 I2C address (default is 0x40)
// #define INA226_ADDRESS 0x40

// // INA226 Register addresses
// #define INA226_REG_CONFIG 0x00
// #define INA226_REG_SHUNT_VOLTAGE 0x01
// #define INA226_REG_BUS_VOLTAGE 0x02
// #define INA226_REG_POWER 0x03
// #define INA226_REG_CURRENT 0x04
// #define INA226_REG_CALIBRATION 0x05
// #define CURRENT_LSB 0.00005
// #define CALIBRATION 51200

// // Configuration values
// #define INA226_CONFIG_DEFAULT 0x4127
// #define SHUNT_RESISTANCE 0.002
// #define MAX_CURRENT 0.5

// Preferences preferences;
// WebServer server(80);

// bool wifiConnected = false;

// // Global variables for measurements
// float voltage = 0;
// float current = 0;
// float power = 0;
// float energy = 0;
// unsigned long lastTime = 0;
// unsigned long startTime = 0;

// void connectToWiFi(const char* ssid, const char* password) {
//   WiFi.begin(ssid, password);
//   Serial.println("Connecting to WiFi...");
//   int retries = 0;
//   while (WiFi.status() != WL_CONNECTED && retries < 20) {
//     delay(1000);
//     Serial.print(".");
//     retries++;
//   }

//   if (WiFi.status() == WL_CONNECTED) {
//     Serial.println("\nWiFi connected!");
//     Serial.print("IP Address: ");
//     Serial.println(WiFi.localIP());
//     wifiConnected = true;
//     startWebServer();
//   } else {
//     Serial.println("\nFailed to connect to WiFi.");
//     wifiConnected = false;
//   }
// }

// void startWebServer() {
//   server.on("/", HTTP_GET, []() {
//     server.send(200, "text/plain", "ESP32 Solar Energy Monitor");
//   });

//   server.on("/energy_data", []() {
//     String json = "{";
//     json += "\"voltage\":" + String(voltage, 3) + ",";
//     json += "\"current\":" + String(current, 3) + ",";
//     json += "\"power\":" + String(power, 3) + ",";
//     json += "\"energy\":" + String(energy, 4);
//     json += "}";
//     server.send(200, "application/json", json);
//   });

//   server.on("/reset_energy", []() {
//     energy = 0;
//     startTime = millis();
//     server.send(200, "text/plain", "Energy counter reset");
//   });

//   server.on("/wifi/reset", []() {
//     resetWiFiCredentials();
//     server.send(200, "text/plain", "WiFi credentials cleared. Restarting...");
//     delay(1000);
//     ESP.restart();
//   });

//   server.begin();
//   Serial.println("HTTP server started");
// }

// bool initINA226() {
//   Serial.println("\n=== INA226 Initialization ===");
  
//   // Check if device is present
//   Wire.beginTransmission(INA226_ADDRESS);
//   byte error = Wire.endTransmission();
  
//   Serial.print("I2C Address 0x");
//   Serial.print(INA226_ADDRESS, HEX);
//   Serial.print(" - ");
  
//   if (error != 0) {
//     Serial.print("ERROR: ");
//     if (error == 2) Serial.println("NACK on address (device not found)");
//     else if (error == 5) Serial.println("Timeout");
//     else Serial.println("Unknown error");
//     return false;
//   }
  
//   Serial.println("Device found!");
  
//   // Reset the device
//   Serial.println("Resetting INA226...");
//   writeRegister(INA226_REG_CONFIG, 0x8000);  // Reset bit
//   delay(10);
  
//   // Configure INA226
//   Serial.println("Configuring INA226...");
//   writeRegister(INA226_REG_CONFIG, INA226_CONFIG_DEFAULT);
//   delay(10);
  
//   // Set calibration
//   Serial.print("Setting calibration to: ");
//   Serial.println(CALIBRATION);
//   writeRegister(INA226_REG_CALIBRATION, CALIBRATION);
//   delay(10);
  
//   // Verify configuration
//   uint16_t configRead = readRegister(INA226_REG_CONFIG);
//   Serial.print("Config register readback: 0x");
//   Serial.println(configRead, HEX);
  
//   uint16_t calRead = readRegister(INA226_REG_CALIBRATION);
//   Serial.print("Calibration register readback: 0x");
//   Serial.println(calRead, HEX);
  
//   Serial.println("=== Initialization Complete ===\n");
//   delay(100);
//   return true;
// }

// // void readINA226() {
// //   // Read bus voltage (solar panel voltage)
// //   uint16_t busVoltageRaw = readRegister(INA226_REG_BUS_VOLTAGE);
// //   voltage = (busVoltageRaw * 1.25) / 1000.0;
  
// //   // Read shunt voltage (for debugging)
// //   int16_t shuntVoltageRaw = (int16_t)readRegister(INA226_REG_SHUNT_VOLTAGE);
// //   float shuntVoltage = shuntVoltageRaw * 2.5 / 1000000.0; // Convert to volts
  
// //   // Read current
// //   int16_t currentRaw = (int16_t)readRegister(INA226_REG_CURRENT);
// //   current = currentRaw * CURRENT_LSB;
  
// //   // Read power
// //   uint16_t powerRaw = readRegister(INA226_REG_POWER);
// //   float powerLSB = CURRENT_LSB * 25;
// //   power = powerRaw * powerLSB;
  
// //   // Debug output every 5 seconds
// //   static unsigned long lastDebug = 0;
// //   if (millis() - lastDebug > 5000) {
// //     Serial.println("\n--- INA226 Raw Register Values ---");
// //     Serial.print("Bus Voltage Raw: 0x");
// //     Serial.print(busVoltageRaw, HEX);
// //     Serial.print(" (");
// //     Serial.print(busVoltageRaw);
// //     Serial.println(")");
    
// //     Serial.print("Shunt Voltage Raw: 0x");
// //     Serial.print(shuntVoltageRaw, HEX);
// //     Serial.print(" (");
// //     Serial.print(shuntVoltageRaw);
// //     Serial.print(") = ");
// //     Serial.print(shuntVoltage * 1000000, 2);
// //     Serial.println(" µV");
    
// //     Serial.print("Current Raw: 0x");
// //     Serial.print(currentRaw, HEX);
// //     Serial.print(" (");
// //     Serial.print(currentRaw);
// //     Serial.println(")");
    
// //     Serial.print("Power Raw: 0x");
// //     Serial.print(powerRaw, HEX);
// //     Serial.print(" (");
// //     Serial.print(powerRaw);
// //     Serial.println(")");
// //     Serial.println("----------------------------------\n");
    
// //     lastDebug = millis();
// //   }
// // }

// // void calculateEnergy() {
// //   static unsigned long previousMicros = 0;
// //   unsigned long currentMicros = micros();
  
// //   // Initialize on first run
// //   if (previousMicros == 0) {
// //     previousMicros = currentMicros;
// //     return;
// //   }
  
// //   // Calculate time in hours
// //   float timeHours = (currentMicros - previousMicros) / 3600000000.0;
  
// //   // Accumulate energy (only if power is positive)
// //   if (power > 0) {
// //     energy += power * timeHours;
// //   }
  
// //   previousMicros = currentMicros;
// // }

// // void printMeasurements() {
// //   // Calculate runtime
// //   unsigned long runtime = (millis() - startTime) / 1000;
// //   int hours = runtime / 3600;
// //   int minutes = (runtime % 3600) / 60;
// //   int seconds = runtime % 60;
  
// //   // Clear screen (optional)
// //   Serial.print("\033[2J\033[H");
  
// //   Serial.println("=== Solar Panel Real-Time Data ===");
// //   Serial.print("Runtime: ");
// //   Serial.printf("%02d:%02d:%02d\n", hours, minutes, seconds);
// //   Serial.println();
  
// //   Serial.print("🔋 Voltage:  ");
// //   Serial.print(voltage, 3);
// //   Serial.println(" V");
  
// //   Serial.print("⚡ Current:  ");
// //   Serial.print(current, 3);
// //   Serial.println(" A");
  
// //   Serial.print("💡 Power:    ");
// //   Serial.print(power, 3);
// //   Serial.println(" W");
  
// //   Serial.print("📊 Energy:   ");
// //   Serial.print(energy, 4);
// //   Serial.println(" Wh");
  
// //   float estimatedPower = voltage * current * 1000;
// //   Serial.println();
// //   Serial.print("Solar Panel Assessment: ");
// //   Serial.print(estimatedPower, 1);
// //   Serial.println(" mW available");
  
// //   Serial.println("===================================");
// //   Serial.println();
// // }

// void resetWiFiCredentials() {
//   Serial.println("Resetting WiFi credentials...");
//   preferences.begin("wifi", false);
//   preferences.remove("ssid");
//   preferences.remove("password");
//   preferences.end();
//   WiFi.disconnect(true);
//   wifiConnected = false;
//   Serial.println("WiFi credentials removed.");
// }

// void writeRegister(uint8_t reg, uint16_t value) {
//   Wire.beginTransmission(INA226_ADDRESS);
//   Wire.write(reg);
//   Wire.write((value >> 8) & 0xFF);
//   Wire.write(value & 0xFF);
//   Wire.endTransmission();
// }

// uint16_t readRegister(uint8_t reg) {
//   Wire.beginTransmission(INA226_ADDRESS);
//   Wire.write(reg);
//   Wire.endTransmission(false);
  
//   Wire.requestFrom(INA226_ADDRESS, 2);
//   uint16_t value = 0;
  
//   if (Wire.available() == 2) {
//     value = Wire.read() << 8;
//     value |= Wire.read();
//   }
  
//   return value;
// }

// void setup() {
//   Serial.begin(115200);
//   delay(1000); // Give serial time to initialize
  
//   // Scan I2C bus first
//   // Serial.println("\nScanning I2C bus...");
//   // Wire.begin();
//   // byte deviceCount = 0;
//   // for (byte address = 1; address < 127; address++) {
//   //   Wire.beginTransmission(address);
//   //   byte error = Wire.endTransmission();
//   //   if (error == 0) {
//   //     Serial.print("I2C device found at 0x");
//   //     if (address < 16) Serial.print("0");
//   //     Serial.println(address, HEX);
//   //     deviceCount++;
//   //   }
//   // }
  
//   // if (deviceCount == 0) {
//   //   Serial.println("ERROR: No I2C devices found!");
//   //   Serial.println("Check your wiring:");
//   //   Serial.println("  - SDA should be connected to GPIO 21 (or your chosen SDA pin)");
//   //   Serial.println("  - SCL should be connected to GPIO 22 (or your chosen SCL pin)");
//   //   Serial.println("  - VCC to 3.3V");
//   //   Serial.println("  - GND to GND");
//   // } else {
//   //   Serial.print("Found ");
//   //   Serial.print(deviceCount);
//   //   Serial.println(" I2C device(s)");
//   // }
//   // Serial.println();

//   // Load stored WiFi credentials
//   preferences.begin("wifi", true);
//   String ssid = preferences.getString("ssid", "");
//   String password = preferences.getString("password", "");
//   preferences.end();

//   Serial.println("Stored SSID: " + ssid);

//   // Connect to WiFi if credentials exist
//   if (ssid.length() > 0 && password.length() > 0) {
//     Serial.println("Connecting with saved WiFi credentials...");
//     connectToWiFi(ssid.c_str(), password.c_str());

//     if (wifiConnected) {
//       Serial.println("Connected to WiFi!");
//       pinMode(2, OUTPUT);
//       digitalWrite(2, HIGH);
//     } else {
//       Serial.println("Failed to connect. Please use /wifi/reset or manually set credentials.");
//     }
//   } else {
//     Serial.println("No WiFi credentials found.");
//     Serial.println("Please store credentials using Preferences or upload them manually.");
//   }

//   // Initialize INA226
//   // if (initINA226()) {
//   //   Serial.println("✓ INA226 initialized successfully");
//   // } else {
//   //   Serial.println("✗ Failed to initialize INA226");
//   //   Serial.println("Check wiring and I2C connections");
//   //   Serial.println("\nTroubleshooting:");
//   //   Serial.println("1. Verify I2C address is 0x40 (default)");
//   //   Serial.println("2. Check SDA/SCL connections");
//   //   Serial.println("3. Ensure 3.3V power supply");
//   //   Serial.println("4. Try adding pull-up resistors (4.7kΩ) on SDA/SCL");
//   //   while(1) {
//   //     delay(1000); // Halt if sensor not found
//   //   }
//   }

//   // lastTime = millis();
//   // startTime = millis();

//   pinMode(2, OUTPUT);
//   digitalWrite(2, HIGH);
  
//   // Serial.println("\nStarting measurements...\n");
// }

// void loop() {
//   if (wifiConnected) {
//     server.handleClient();
//   }

//   // if (millis() - lastTime >= 1000) {
//   //   readINA226();
//   //   calculateEnergy();
//   //   printMeasurements();
//   //   lastTime = millis();
//   // }
// }