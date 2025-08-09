/*
  test_wifi_connection.ino
  Purpose: Verify Wi-Fi connectivity on ESP32-S3
  SSID: testing
  Password: jenish1235
*/

#include <WiFi.h>

const char *ssid = "testing";
const char *password = "jenish1235";

const int LED_PIN = LED_BUILTIN; // Change if your board uses a different LED pin

unsigned long lastBlink = 0;
unsigned long blinkInterval = 300; // fast blink until connected
bool ledState = false;

bool wifiConnected = false;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.println("\n[WiFi Test] Starting Wi-Fi connection test...");

    WiFi.begin(ssid, password);

    Serial.printf("[WiFi Test] Connecting to SSID: %s\n", ssid);
}

void loop()
{
    unsigned long currentMillis = millis();

    // Blink LED
    if (currentMillis - lastBlink >= blinkInterval)
    {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
        lastBlink = currentMillis;
    }

    // Check connection status
    if (!wifiConnected && WiFi.status() == WL_CONNECTED)
    {
        wifiConnected = true;
        blinkInterval = 1000; // slow blink when connected
        Serial.println("\n[WiFi Test] Connected!");
        Serial.print("[WiFi Test] IP Address: ");
        Serial.println(WiFi.localIP());
    }

    if (!wifiConnected && WiFi.status() != WL_CONNECTED)
    {
        // Still trying to connect
        static unsigned long lastPrint = 0;
        if (currentMillis - lastPrint >= 1000)
        {
            Serial.print(".");
            lastPrint = currentMillis;
        }
    }
}
