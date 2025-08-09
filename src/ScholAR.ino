// ScholAR AI Smart Glasses Firmware
// 
// This firmware controls the AI-powered smart glasses system
// 

#define CAMERA_MODEL_XIAO_ESP32S3
#include <WiFi.h>
#include <WiFiClient.h>
#include "esp_camera.h"
#include "camera_pins.h"
#include <WebSocketsClient.h>
#include <esp_idf_version.h>
#include <math.h>

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)
  #define USE_IDF5_NEW_I2S 1
  #include "driver/i2s_common.h"
  #include "driver/i2s_pdm.h"
#else
  #define USE_IDF5_NEW_I2S 0
  #include "driver/i2s.h"
#endif

// WiFi Configuration
const char *ssid = "Vedan_Guest";
const char *password = "Vedantu@2025";

// Server Configuration
static const char *SERVER_HOST = "10.10.30.172";
static const uint16_t SERVER_PORT = 8000;
static const char *SERVER_PATH = "/upload/image";

// Camera Tuning Configuration
static const int AE_LEVEL = 1;    // -2..+2; bump exposure a touch
static const int WB_MODE = 2;     // 0=auto; try 2 (fluorescent) if green cast
static const int JPEG_Q_UXGA = 8; // lower number = higher quality (8–12 good)
static const int JPEG_Q_SVGA = 10;

// Audio Configuration
#define SAMPLE_RATE     16000
#define CHUNK_MS        20
#define CHUNK_SAMPLES   ((SAMPLE_RATE * CHUNK_MS) / 1000)  // 320
#define CHUNK_BYTES     (CHUNK_SAMPLES * 2)                // 640 (s16le)
#define VOLUME_SHIFT    1  // Reduced from 2 to 1 (2x amplification instead of 4x)
#define NOISE_GATE_THRESHOLD 200  // Reduced from 500 to 200 for less aggressive noise gate
#define DC_REMOVAL_ALPHA 0.999f   // Increased from 0.995f for smoother DC removal
#define RECORDING_DURATION_MS 10000  // 10 seconds recording session

// XIAO ESP32S3 Sense PDM mic pins
#define PDM_CLK_PIN     42
#define PDM_DATA_PIN    41

// WebSocket Configuration
static const char* WS_PATH = "/stream/audio";

// LED Configuration
const int LED_PIN = LED_BUILTIN;
unsigned long lastBlink = 0;
unsigned long blinkInterval = 300; // fast blink until connected
bool ledState = false;

// Connection Status
bool wifiConnected = false;

// WebSocket and Audio
WebSocketsClient ws;
volatile bool wsConnected = false;
volatile bool audioReady = false;
volatile bool isRecording = false;
volatile unsigned long recordingStartTime = 0;

#if USE_IDF5_NEW_I2S
  static i2s_chan_handle_t rx_handle = NULL;
#else
  static const i2s_port_t I2S_PORT = I2S_NUM_0;
#endif

static uint8_t* audioBuf = nullptr;
static float dcOffset = 0.0f;
static unsigned long lastAudioLevelReport = 0;

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
  
  // Wait for WiFi connection before initializing camera
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.printf("\n[WiFi] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
  wifiConnected = true;
  blinkInterval = 1000; // slow blink when connected
  
  // Initialize Camera
  Serial.println("[Camera] Initializing camera for OCR...");
  if (!initCameraForOCR()) {
    Serial.println("[Camera] Camera init failed!");
    while (true) {
      delay(1000);
    }
  }
  Serial.println("[Camera] Camera initialized successfully!");
  
  // Warm-up frames to let AEC/AWB settle
  for (int i = 0; i < 2; ++i) {
    camera_fb_t *warm = esp_camera_fb_get();
    if (warm) {
      esp_camera_fb_return(warm);
    }
    delay(120);
  }
  
  // Initialize Audio
  Serial.println("[Audio] Initializing PDM microphone...");
  audioBuf = (uint8_t*) ps_malloc(CHUNK_BYTES);
  if (!audioBuf) audioBuf = (uint8_t*) malloc(CHUNK_BYTES);
  if (!audioBuf) { 
    Serial.println("[Audio] Buffer allocation failed!");
    while (1) delay(1000);
  }
  
  configureI2S_PDM_RX();
  Serial.println("[Audio] PDM microphone initialized!");
  
  // Initialize WebSocket for audio streaming
  Serial.println("[WebSocket] Configuring audio WebSocket...");
  ws.begin(SERVER_HOST, SERVER_PORT, WS_PATH);
  ws.onEvent(wsEvent);
  ws.setReconnectInterval(3000);
  ws.enableHeartbeat(15000, 3000, 2);
  Serial.printf("[WebSocket] Connecting to ws://%s:%u%s\n", SERVER_HOST, SERVER_PORT, WS_PATH);
}

// Initialize Camera for OCR
bool initCameraForOCR() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Prefer max detail if PSRAM is available; else solid fallback.
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA; // 1600x1200
    config.jpeg_quality = JPEG_Q_UXGA;  // 8–12
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    config.frame_size = FRAMESIZE_SVGA; // 800x600
    config.jpeg_quality = JPEG_Q_SVGA;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }
  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_camera_deinit(); // clean slate just in case
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[Camera] Init failed: 0x%x\n", err);
    return false;
  }

  // --- Sensor tuning for OCR ---
  sensor_t *s = esp_camera_sensor_get();

  // Auto exposure/gain
  s->set_exposure_ctrl(s, 1);
  s->set_aec2(s, 1);
  s->set_ae_level(s, AE_LEVEL);
  s->set_gain_ctrl(s, 1);
  s->set_gainceiling(s, GAINCEILING_32X);

  // White balance
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_wb_mode(s, WB_MODE); // 0 auto; 2 fluorescent if green cast

  // Cleanups / sharpening helpers
  s->set_lenc(s, 1); // lens correction
  s->set_bpc(s, 1);  // black pixel correction
  s->set_wpc(s, 1);  // white pixel correction
  s->set_raw_gma(s, 1);
  s->set_dcw(s, 1); // better downsize quality
  s->set_colorbar(s, 0);

  // Make text pop a bit
  s->set_brightness(s, 1);  // -2..2
  s->set_contrast(s, 1);    // -2..2
  s->set_saturation(s, -1); // -2..2 (slight desat helps OCR)

  return true;
}

// Upload Image via Multipart Form Data
bool uploadImageMultipart(camera_fb_t *fb) {
  if (!fb) return false;

  const char *boundary = "----ESP32Boundary7MA4YWxkTrZu0gW";
  String preamble;
  preamble += "--";
  preamble += boundary;
  preamble += "\r\n";
  preamble += "Content-Disposition: form-data; name=\"file\"; filename=\"frame.jpg\"\r\n";
  preamble += "Content-Type: image/jpeg\r\n\r\n";
  String closing = "\r\n--";
  closing += boundary;
  closing += "--\r\n";
  size_t contentLength = preamble.length() + fb->len + closing.length();

  WiFiClient client;
  Serial.printf("[Upload] Connecting to %s:%u\n", SERVER_HOST, SERVER_PORT);
  if (!client.connect(SERVER_HOST, SERVER_PORT)) {
    Serial.println("[Upload] Connection failed");
    return false;
  }

  // Request + headers
  client.printf("POST %s HTTP/1.1\r\n", SERVER_PATH);
  client.printf("Host: %s\r\n", SERVER_HOST);
  client.println("Connection: close");
  client.printf("Content-Type: multipart/form-data; boundary=%s\r\n", boundary);
  client.printf("Content-Length: %u\r\n", (unsigned)contentLength);
  client.print("\r\n");

  // Body streamed (no large temp buffer)
  client.print(preamble);
  client.write(fb->buf, fb->len);
  client.print(closing);

  // Read status
  String status = client.readStringUntil('\n');
  Serial.print("[Upload] Response: ");
  Serial.println(status);
  bool ok = status.startsWith("HTTP/1.1 200");

  // Optional: drain remaining response
  unsigned long t0 = millis();
  while (client.connected() && millis() - t0 < 3000) {
    while (client.available()) {
      String line = client.readStringUntil('\n');
      Serial.println(line);
      t0 = millis();
    }
  }
  client.stop();
  Serial.printf("[Upload] %s\n", ok ? "Success" : "Failed");
  return ok;
}

// WebSocket Event Handler
void wsEvent(WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED) {
    wsConnected = true;
    audioReady = true;
    recordingStartTime = 0;
    isRecording = false;
    Serial.printf("[WS] Connected: ws://%s:%u%s\n", SERVER_HOST, SERVER_PORT, WS_PATH);
    Serial.println("[Audio] Ready! You can start speaking now.");
  } else if (type == WStype_DISCONNECTED) {
    wsConnected = false;
    audioReady = false;
    isRecording = false;
    Serial.println("[WS] Disconnected");
  } else if (type == WStype_TEXT) {
    Serial.printf("[WS] %.*s\n", (int)length, (char*)payload);
  }
}

// Audio Processing Functions
void removeDCOffset(int16_t* samples, size_t count) {
  float chunkSum = 0.0f;
  for (size_t i = 0; i < count; i++) {
    chunkSum += samples[i];
  }
  float chunkDcOffset = chunkSum / count;
  
  dcOffset = DC_REMOVAL_ALPHA * dcOffset + (1.0f - DC_REMOVAL_ALPHA) * chunkDcOffset;
  
  for (size_t i = 0; i < count; i++) {
    int32_t sample = samples[i] - (int16_t)dcOffset;
    if (sample > 32767) sample = 32767;
    if (sample < -32768) sample = -32768;
    samples[i] = (int16_t)sample;
  }
}

void applyNoiseGate(int16_t* samples, size_t count) {
  for (size_t i = 0; i < count; i++) {
    int16_t absSample = abs((int)samples[i]);
    if (absSample < NOISE_GATE_THRESHOLD) {
      float reduction = (float)absSample / NOISE_GATE_THRESHOLD;
      samples[i] = (int16_t)(samples[i] * reduction);
    }
  }
}

void processAudio(int16_t* samples, size_t count) {
  removeDCOffset(samples, count);
  applyNoiseGate(samples, count);
  
  if (VOLUME_SHIFT > 0) {
    for (size_t i = 0; i < count; i++) {
      int32_t v = (int32_t)samples[i] << VOLUME_SHIFT;
      if (v > 32767) v = 32767;
      if (v < -32768) v = -32768;
      samples[i] = (int16_t)v;
    }
  }
}

float calculateAudioLevel(int16_t* samples, size_t count) {
  float sum = 0.0f;
  for (size_t i = 0; i < count; i++) {
    sum += (float)abs((int)samples[i]);
  }
  return sum / count;
}

#if USE_IDF5_NEW_I2S
// IDF 5.x PDM Configuration
void configureI2S_PDM_RX() {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_handle));

  i2s_pdm_rx_clk_config_t clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(SAMPLE_RATE);
  i2s_pdm_rx_slot_config_t slot_cfg = 
      I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);

  i2s_pdm_rx_gpio_config_t gpio_cfg = {
    .clk = (gpio_num_t)PDM_CLK_PIN,
    .din = (gpio_num_t)PDM_DATA_PIN,
    .invert_flags = { .clk_inv = false },
  };

  i2s_pdm_rx_config_t pdm_cfg = {
    .clk_cfg  = clk_cfg,
    .slot_cfg = slot_cfg,
    .gpio_cfg = gpio_cfg,
  };

  ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(rx_handle, &pdm_cfg));
  ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
  Serial.println("[I2S] PDM RX enabled @16k/16-bit MONO");
}
#else
// Legacy I2S Configuration
void configureI2S_PDM_RX() {
  i2s_driver_uninstall(I2S_PORT);

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_cfg = {
    .bck_io_num   = I2S_PIN_NO_CHANGE,
    .ws_io_num    = PDM_CLK_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num  = PDM_DATA_PIN
  };

  ESP_ERROR_CHECK(i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL));
  ESP_ERROR_CHECK(i2s_set_pin(I2S_PORT, &pin_cfg));
  ESP_ERROR_CHECK(i2s_set_clk(I2S_PORT, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO));
  Serial.println("[I2S] PDM RX enabled @16k/16-bit MONO");
}
#endif

void loop() {
  unsigned long currentMillis = millis();
  
  // Handle WebSocket
  ws.loop();
  
  // Handle LED blinking for status indication
  if (currentMillis - lastBlink >= blinkInterval) {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
    lastBlink = currentMillis;
  }
  
  // Handle Audio Streaming
  if (wsConnected && audioReady) {
    size_t totalRead = 0;
    while (totalRead < CHUNK_BYTES) {
      size_t justRead = 0;
#if USE_IDF5_NEW_I2S
      esp_err_t res = i2s_channel_read(rx_handle, audioBuf + totalRead,
                                       CHUNK_BYTES - totalRead, &justRead, portMAX_DELAY);
#else
      esp_err_t res = i2s_read(I2S_PORT, audioBuf + totalRead,
                               CHUNK_BYTES - totalRead, &justRead, portMAX_DELAY);
#endif
      if (res != ESP_OK) { 
        Serial.printf("[I2S] Read error: 0x%x\n", res); 
        break; 
      }
      totalRead += justRead;
    }

    if (totalRead == CHUNK_BYTES) {
      // Start recording session if not already recording
      if (!isRecording) {
        isRecording = true;
        recordingStartTime = millis();
        Serial.println("[Audio] Starting 10-second recording session...");
        blinkInterval = 100; // Fast blink during recording
      }

      // Check if recording session should end
      if (isRecording && (millis() - recordingStartTime >= RECORDING_DURATION_MS)) {
        isRecording = false;
        Serial.println("[Audio] Recording session completed (10 seconds).");
        blinkInterval = 1000; // Back to slow blink
      }

      if (isRecording) {
        // Process and send audio
        processAudio((int16_t*)audioBuf, CHUNK_SAMPLES);
        
        // Report audio level periodically
        if (millis() - lastAudioLevelReport > 2000) {
          float level = calculateAudioLevel((int16_t*)audioBuf, CHUNK_SAMPLES);
          if (level > 0) {
            int db = (int)(20 * log10(level / 32768.0f));
            Serial.printf("[Audio] Level: %d dB (avg: %.0f)\n", db, level);
          }
          lastAudioLevelReport = millis();
        }
        
        // Send audio data
        ws.sendBIN(audioBuf, CHUNK_BYTES);
      }
    }
  }
  
  // Handle Image Capture (reduced frequency during audio recording)
  static unsigned long lastCapture = 0;
  if (wifiConnected && !isRecording && currentMillis - lastCapture >= 10000) {
    lastCapture = currentMillis;
    
    Serial.println("\n[Camera] Capturing image...");
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("[Camera] Failed to capture image");
    } else {
      Serial.printf("[Camera] Captured %dx%d, %u bytes\n", fb->width, fb->height, fb->len);
      
      // Upload the image
      if (uploadImageMultipart(fb)) {
        Serial.println("[Camera] Image uploaded successfully!");
      } else {
        Serial.println("[Camera] Image upload failed");
      }
      
      // Release the frame buffer
      esp_camera_fb_return(fb);
    }
  }
}
