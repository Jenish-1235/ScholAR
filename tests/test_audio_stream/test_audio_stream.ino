/*
  XIAO ESP32S3 Sense PDM Microphone - Clean Implementation
  
  This implementation focuses on getting the PDM microphone working correctly
  with proper configuration and minimal complexity.
  
  Hardware: Seeed Studio XIAO ESP32S3 Sense
  Microphone: Built-in PDM microphone (MSM261S4030H0)
  Pins: CLK=42, DATA=41
*/

#include <WiFi.h>
#include <WebSocketsClient.h>
#include <driver/i2s.h>

// WiFi Configuration
const char* ssid = "Vedan_Guest";
const char* password = "Vedantu@2025";

// WebSocket Configuration  
const char* websocket_host = "10.10.30.172";
const int websocket_port = 8000;
const char* websocket_path = "/stream/audio";

// Audio Configuration
#define SAMPLE_RATE 16000
#define BITS_PER_SAMPLE 16
#define CHANNEL_COUNT 1
#define BYTES_PER_SAMPLE (BITS_PER_SAMPLE / 8)

// Chunk configuration (send audio in chunks)
#define CHUNK_DURATION_MS 100  // 100ms chunks
#define SAMPLES_PER_CHUNK (SAMPLE_RATE * CHUNK_DURATION_MS / 1000)
#define BYTES_PER_CHUNK (SAMPLES_PER_CHUNK * BYTES_PER_SAMPLE * CHANNEL_COUNT)

// I2S Configuration for PDM
#define I2S_PORT I2S_NUM_0
#define PDM_CLK_PIN 42
#define PDM_DATA_PIN 41

// DMA Buffer Configuration
#define DMA_BUF_COUNT 4
#define DMA_BUF_LEN 1024

// Global Variables
WebSocketsClient webSocket;
bool isConnected = false;
int16_t* audioBuffer = nullptr;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== XIAO ESP32S3 PDM Microphone Test ===");
  
  // Initialize LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // Allocate audio buffer
  audioBuffer = (int16_t*)malloc(BYTES_PER_CHUNK);
  if (!audioBuffer) {
    Serial.println("ERROR: Failed to allocate audio buffer!");
    while(1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
  }
  
  // Connect to WiFi
  setupWiFi();
  
  // Initialize I2S for PDM microphone
  if (setupI2S()) {
    Serial.println("I2S PDM setup successful");
  } else {
    Serial.println("ERROR: I2S PDM setup failed!");
    while(1) delay(1000);
  }
  
  // Setup WebSocket
  setupWebSocket();
  
  Serial.println("Setup complete - starting audio capture...");
}

void setupWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.print("WiFi connected! IP: ");
  Serial.println(WiFi.localIP());
}

bool setupI2S() {
  Serial.println("Configuring I2S for PDM microphone...");
  
  // I2S configuration for PDM microphone - ESP32S3 specific
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = DMA_BUF_COUNT,
    .dma_buf_len = DMA_BUF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  
  // Install I2S driver first
  esp_err_t result = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (result != ESP_OK) {
    Serial.print("ERROR: I2S driver install failed: ");
    Serial.println(esp_err_to_name(result));
    return false;
  }
  
  // Pin configuration for PDM
  i2s_pin_config_t pin_config = {
    .mck_io_num = I2S_PIN_NO_CHANGE,
    .bck_io_num = I2S_PIN_NO_CHANGE,
    .ws_io_num = PDM_CLK_PIN,      // PDM Clock
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = PDM_DATA_PIN    // PDM Data
  };
  
  // Set pins
  result = i2s_set_pin(I2S_PORT, &pin_config);
  if (result != ESP_OK) {
    Serial.print("ERROR: I2S pin config failed: ");
    Serial.println(esp_err_to_name(result));
    i2s_driver_uninstall(I2S_PORT);
    return false;
  }
  
  // CRITICAL: Configure PDM for ESP32S3
  result = i2s_set_pdm_rx_down_sample(I2S_PORT, I2S_PDM_DSR_8S);
  if (result != ESP_OK) {
    Serial.print("ERROR: PDM down sample config failed: ");
    Serial.println(esp_err_to_name(result));
    i2s_driver_uninstall(I2S_PORT);
    return false;
  }
  
  // Set clock last
  result = i2s_set_clk(I2S_PORT, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
  if (result != ESP_OK) {
    Serial.print("ERROR: I2S clock config failed: ");
    Serial.println(esp_err_to_name(result));
    i2s_driver_uninstall(I2S_PORT);
    return false;
  }
  
  // Start I2S
  result = i2s_start(I2S_PORT);
  if (result != ESP_OK) {
    Serial.print("ERROR: I2S start failed: ");
    Serial.println(esp_err_to_name(result));
    i2s_driver_uninstall(I2S_PORT);
    return false;
  }
  
  Serial.printf("I2S PDM configured successfully: %d Hz, %d-bit, mono\n", SAMPLE_RATE, BITS_PER_SAMPLE);
  
  // Clear any initial data in the buffer
  size_t bytes_read;
  uint8_t dummy_buffer[1024];
  i2s_read(I2S_PORT, dummy_buffer, sizeof(dummy_buffer), &bytes_read, 0);
  
  return true;
}

void setupWebSocket() {
  webSocket.begin(websocket_host, websocket_port, websocket_path);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
  webSocket.enableHeartbeat(15000, 3000, 2);
  
  Serial.printf("WebSocket connecting to ws://%s:%d%s\n", 
                websocket_host, websocket_port, websocket_path);
}

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("[WS] Disconnected");
      isConnected = false;
      digitalWrite(LED_BUILTIN, LOW);
      break;
      
    case WStype_CONNECTED:
      Serial.printf("[WS] Connected to: %s\n", payload);
      isConnected = true;
      break;
      
    case WStype_TEXT:
      Serial.printf("[WS] Received text: %s\n", payload);
      break;
      
    case WStype_BIN:
      Serial.printf("[WS] Received binary data: %u bytes\n", length);
      break;
      
    case WStype_ERROR:
      Serial.println("[WS] Error occurred");
      isConnected = false;
      break;
      
    default:
      break;
  }
}

void loop() {
  // Handle WebSocket
  webSocket.loop();
  
  // Capture and send audio if connected
  if (isConnected) {
    captureAndSendAudio();
  } else {
    delay(100);  // Wait if not connected
  }
}

void captureAndSendAudio() {
  size_t bytesRead = 0;
  size_t totalBytesRead = 0;
  
  // Read audio data from I2S
  while (totalBytesRead < BYTES_PER_CHUNK) {
    esp_err_t result = i2s_read(I2S_PORT, 
                               (uint8_t*)audioBuffer + totalBytesRead,
                               BYTES_PER_CHUNK - totalBytesRead,
                               &bytesRead,
                               pdMS_TO_TICKS(1000));  // 1 second timeout
    
    if (result != ESP_OK) {
      Serial.printf("I2S read error: %s\n", esp_err_to_name(result));
      return;
    }
    
    totalBytesRead += bytesRead;
  }
  
  // Apply basic audio processing
  processAudio(audioBuffer, SAMPLES_PER_CHUNK);
  
  // Send via WebSocket
  webSocket.sendBIN((uint8_t*)audioBuffer, BYTES_PER_CHUNK);
  
  // Blink LED to show activity
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 200) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    lastBlink = millis();
  }
}

void processAudio(int16_t* buffer, size_t samples) {
  // Apply basic DC removal and gain
  static int32_t dcSum = 0;
  
  // Calculate DC offset
  int64_t sum = 0;
  for (size_t i = 0; i < samples; i++) {
    sum += buffer[i];
  }
  dcSum = sum / samples;
  
  // Remove DC offset and apply gain
  for (size_t i = 0; i < samples; i++) {
    // Remove DC offset
    int32_t sample = buffer[i] - dcSum;
    
    // Apply 2x gain with clipping
    sample *= 2;
    if (sample > 32767) sample = 32767;
    if (sample < -32768) sample = -32768;
    
    buffer[i] = (int16_t)sample;
  }
}