#include <WiFi.h>

const char* ssid = "testing";
const char* password = "jenish1235";

const int LED_PIN = LED_BUILTIN;

unsigned long lastBlink = 0;
unsigned long blinkInterval = 1000;
bool ledState = false;

bool wifiConnected = false;

void setup(){
  Serial.begin(115200);

  while (!Serial){
    delay(10);
  }

  Serial.println("\n Starting Glasses");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  connectToWifi();
}

void loop(){

  unsigned long currentMillis = millis();
  if(currentMillis - lastBlink >= blinkInterval){
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
    lastBlink = currentMillis;
  }
}

void connectToWifi(){
  Serial.printf("Connecting to SSI: %s\n", ssid);
  WiFi.begin(ssid, password);

  int attempts = 0;
  const int maxAttempts = 20;

  while(WiFi.status() != WL_CONNECTED && attempts < maxAttempts){
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if(WiFi.status() == WL_CONNECTED){
    wifiConnected = true;
    blinkInterval = 1000;
    Serial.printf("Connected to Wifi");
  }else{
    wifiConnected = false;
    blinkInterval = 300;
    Serial.println("\n Wifi connection failed.");
  }

}