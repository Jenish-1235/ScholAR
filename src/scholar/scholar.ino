// ScholAR AI Smart Glasses Firmware
// 
// This firmware controls the AI-powered smart glasses system
// 

#include <WiFi.h>

// WiFi Configuration
const char *ssid = "Vedan_Guest";
const char *password = "Vedantu@2025";

// LED Configuration
const int LED_PIN = LED_BUILTIN;
unsigned long lastBlink = 0;
unsigned long blinkInterval = 300; // fast blink until connected
bool ledState = false;

// Connection Status
bool wifiConnected = false;

void setup() {
  // Initialize Serial Communication
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize WiFi
  Serial.println("\n[ScholAR] Starting AI Smart Glasses...");
  Serial.println("[WiFi] Starting Wi-Fi connection...");
  
  WiFi.begin(ssid, password);
  Serial.printf("[WiFi] Connecting to SSID: %s\n", ssid);
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Handle LED blinking for status indication
  if (currentMillis - lastBlink >= blinkInterval) {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
    lastBlink = currentMillis;
  }
  
  // Check WiFi connection status
  if (!wifiConnected && WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    blinkInterval = 1000; // slow blink when connected
    Serial.println("\n[WiFi] Connected!");
    Serial.print("[WiFi] IP Address: ");
    Serial.println(WiFi.localIP());
  }
  
  if (!wifiConnected && WiFi.status() != WL_CONNECTED) {
    // Still trying to connect
    static unsigned long lastPrint = 0;
    if (currentMillis - lastPrint >= 1000) {
      Serial.print(".");
      lastPrint = currentMillis;
    }
  }
  
  // Main program logic will go here
}
