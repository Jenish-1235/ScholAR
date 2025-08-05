#include <WiFi.h>
#include <esp_camera.h>


// Wifi connection variables
const char* ssid = "Create Impact";
const char* password = "RICE123!@#";

const int LED_PIN = LED_BUILTIN;

unsigned long lastBlink = 0;
unsigned long blinkInterval = 1000;
bool ledState = false;

bool wifiConnected = false;

// Camera pin mapping for Seeed XIAO ESP32-S3
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39

#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15

#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

camera_fb_t* prevFrame = nullptr;
unsigned long lastFrameCapture = 0;
const unsigned long frameInterval = 2000; // 2 seconds time between frame captures
const int changeThreshold = 500;



void setup(){
  Serial.begin(115200);

  while (!Serial){
    delay(10);
  }

  Serial.println("\n Starting Glasses");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  connectToWifi();
  if(!setupCamera()){
    Serial.println("Camera Failed.");
    while(true) delay(1000);
  }
}

void loop(){

  unsigned long currentMillis = millis();
  if(currentMillis - lastBlink >= blinkInterval){
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
    lastBlink = currentMillis;
  }

  if(currentMillis - lastFrameCapture >= frameInterval){
    camera_fb_t* current = captureFrame();
    if(current){
      bool changed = hasFrameChanged(current, prevFrame);
      Serial.printf("Frame Changed: %s\n", changed ? "YES" : "NO");

      if (prevFrame) {
        free(prevFrame->buf);  // Free JPEG data
        free(prevFrame);       // Free frame struct
      }

      prevFrame = cloneFrame(current); // Copy current frame

      esp_camera_fb_return(current); 
    }
    lastFrameCapture = currentMillis;
  }
}

void connectToWifi(){
  Serial.printf("Connecting to SSID: %s\n", ssid);
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

bool setupCamera(){

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 15;
  config.fb_count = 1;


  esp_err_t err = esp_camera_init(&config);
  if(err != ESP_OK){
    Serial.println("Failed to initialise camera");
    return false;
  }

  Serial.println("Initialised Camera");
  return true;


}

camera_fb_t* captureFrame(){
  camera_fb_t* fb = esp_camera_fb_get();
  if(!fb){
    Serial.println("Failed to capture frame.");
  return nullptr;
  }

  Serial.printf("Captured : %dx%d | %u bytes\n", 
  fb->width, fb->height, fb->len);
  return fb;
}

bool hasFrameChanged(camera_fb_t* current, camera_fb_t* previous) {
  if(!current || !previous) return true;
  if(current->len != previous->len) return true;

  int diffCount = 0;
  for(size_t i = 0; i < current->len; i++){
    if(current->buf[i] != previous->buf[i]){
      diffCount++;
      if(diffCount > changeThreshold) return true;
    }
  }
  return false;
}

camera_fb_t* cloneFrame(camera_fb_t* source) {
  if (!source) return nullptr;

  camera_fb_t* clone = (camera_fb_t*) malloc(sizeof(camera_fb_t));
  if (!clone) return nullptr;

  clone->buf = (uint8_t*) malloc(source->len);
  if (!clone->buf) {
    free(clone);
    return nullptr;
  }

  memcpy(clone->buf, source->buf, source->len);
  clone->len = source->len;
  clone->width = source->width;
  clone->height = source->height;
  clone->format = source->format;

  return clone;
}

// Will be used to upload the captured camera frame to backend.
bool uploadFrame(){
  return true;
}


