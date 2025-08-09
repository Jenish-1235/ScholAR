/*
  test_mic_websocket_pdm_driver_only.ino
  - NO <I2S.h> wrapper — uses ESP-IDF I2S drivers only
  - Supports IDF 5.x (new PDM API) and legacy API
  - Streams 16 kHz / 16-bit mono PCM (20 ms = 640B) over WebSocket
  - Enhanced with audio cleaning, volume boost, and user feedback

  Wi-Fi   : SSID=testing  PASS=jenish1235
  WebSock : ws://10.217.177.125:8000/stream/audio
*/

#include <WiFi.h>
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

// ---------- Wi-Fi ----------
static const char* WIFI_SSID = "Vedan_Guest";
static const char* WIFI_PASS = "Vedantu@2025";

// ---------- WebSocket ----------
static const char* SERVER_HOST = "10.10.30.172";
static const uint16_t SERVER_PORT = 8000;
static const char* SERVER_PATH = "/stream/audio";

// ---------- Audio ----------
#define SAMPLE_RATE     16000
#define CHUNK_MS        20
#define CHUNK_SAMPLES   ((SAMPLE_RATE * CHUNK_MS) / 1000)  // 320
#define CHUNK_BYTES     (CHUNK_SAMPLES * 2)                // 640 (s16le)
#define VOLUME_SHIFT    1  // Reduced from 2 to 1 (2x amplification instead of 4x)
#define NOISE_GATE_THRESHOLD 200  // Reduced from 500 to 200 for less aggressive noise gate
#define DC_REMOVAL_ALPHA 0.999f   // Increased from 0.995f for smoother DC removal
#define RECORDING_DURATION_MS 10000  // 10 seconds recording session

// XIAO ESP32S3 Sense PDM mic pins (CLK=42, DATA=41)
#define PDM_CLK_PIN     42
#define PDM_DATA_PIN    41

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

WebSocketsClient ws;
volatile bool wsConnected = false;
volatile bool audioReady = false;
volatile bool userPrompted = false;
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
static unsigned long lastUserPrompt = 0;

// ---------- Helpers ----------
void connectWiFi() {
  Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(400); Serial.print("."); }
  Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
}

void wsEvent(WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED) {
    wsConnected = true;
    audioReady = true;
    resetRecordingSession();
    Serial.printf("[WS] Connected: ws://%s:%u%s\n", SERVER_HOST, SERVER_PORT, SERVER_PATH);
    Serial.println("[AUDIO] 🎤 Audio system ready! You can start speaking now.");
    Serial.println("[AUDIO] 💡 LED will blink rapidly when audio is being streamed.");
    Serial.println("[AUDIO] ⏱️  Recording session will be 10 seconds.");
    // ws.sendTXT(String("{\"type\":\"meta\",\"codec\":\"pcm_s16le\",\"sample_rate\":") +
    //            SAMPLE_RATE + ",\"channels\":1,\"chunk_ms\":" + CHUNK_MS + "}");
  } else if (type == WStype_DISCONNECTED) {
    wsConnected = false;
    audioReady = false;
    resetRecordingSession();
    Serial.println("[WS] Disconnected");
  } else if (type == WStype_TEXT) {
    Serial.printf("[WS] %.*s\n", (int)length, (char*)payload);
  }
}

void configureWebSocket() {
  ws.begin(SERVER_HOST, SERVER_PORT, SERVER_PATH);
  ws.onEvent(wsEvent);
  ws.setReconnectInterval(3000);
  ws.enableHeartbeat(15000, 3000, 2);
  Serial.printf("[WS] Connecting to ws://%s:%u%s ...\n", SERVER_HOST, SERVER_PORT, SERVER_PATH);
}

// ---------- Audio Processing Functions ----------
float calculateAudioLevel(int16_t* samples, size_t count) {
  float sum = 0.0f;
  for (size_t i = 0; i < count; i++) {
    sum += (float)abs((int)samples[i]);
  }
  return sum / count;
}

void removeDCOffset(int16_t* samples, size_t count) {
  // Calculate DC offset for this chunk
  float chunkSum = 0.0f;
  for (size_t i = 0; i < count; i++) {
    chunkSum += samples[i];
  }
  float chunkDcOffset = chunkSum / count;
  
  // Apply low-pass filter to DC offset (smoother)
  dcOffset = DC_REMOVAL_ALPHA * dcOffset + (1.0f - DC_REMOVAL_ALPHA) * chunkDcOffset;
  
  // Remove DC offset from samples (with smoothing)
  for (size_t i = 0; i < count; i++) {
    int32_t sample = samples[i] - (int16_t)dcOffset;
    // Prevent extreme values
    if (sample > 32767) sample = 32767;
    if (sample < -32768) sample = -32768;
    samples[i] = (int16_t)sample;
  }
}

void applyNoiseGate(int16_t* samples, size_t count) {
  for (size_t i = 0; i < count; i++) {
    int16_t absSample = abs((int)samples[i]);
    if (absSample < NOISE_GATE_THRESHOLD) {
      // Apply gradual reduction instead of hard cutoff to prevent distortion
      float reduction = (float)absSample / NOISE_GATE_THRESHOLD;
      samples[i] = (int16_t)(samples[i] * reduction);
    }
  }
}

void processAudio(int16_t* samples, size_t count) {
  // Remove DC offset first
  removeDCOffset(samples, count);
  
  // Apply noise gate (less aggressive)
  applyNoiseGate(samples, count);
  
  // Apply volume boost (reduced amplification)
  if (VOLUME_SHIFT > 0) {
    for (size_t i = 0; i < count; i++) {
      int32_t v = (int32_t)samples[i] << VOLUME_SHIFT;
      // Prevent clipping with soft limiting
      if (v > 32767) v = 32767;
      if (v < -32768) v = -32768;
      samples[i] = (int16_t)v;
    }
  }
}

void reportAudioLevel(int16_t* samples, size_t count) {
  if (millis() - lastAudioLevelReport > 2000) { // Report every 2 seconds
    float level = calculateAudioLevel(samples, count);
    // Prevent log(0) or negative values
    if (level > 0) {
      int db = (int)(20 * log10(level / 32768.0f));
      Serial.printf("[AUDIO] Level: %d dB (avg: %.0f)\n", db, level);
    } else {
      Serial.printf("[AUDIO] Level: Silent (avg: %.0f)\n", level);
    }
    lastAudioLevelReport = millis();
  }
}

void promptUserToSpeak() {
  if (!userPrompted && audioReady && wsConnected) {
    if (millis() - lastUserPrompt > 5000) { // Prompt every 5 seconds if not speaking
      Serial.println("🎤 [READY] Start speaking now! Audio is being streamed...");
      Serial.println("💡 LED blinking = audio streaming active");
      lastUserPrompt = millis();
    }
  }
}

#if USE_IDF5_NEW_I2S
// ---------- IDF 5.x (new PDM API) ----------
void configureI2S_PDM_RX_IDF5() {
  // Create RX channel
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_handle));

  // Default clock: sets sample rate etc.
  i2s_pdm_rx_clk_config_t clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(SAMPLE_RATE);

  // **IMPORTANT**: slot config needs TWO args (bits, mono/stereo)
  i2s_pdm_rx_slot_config_t slot_cfg =
      I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);

  // GPIO config
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
  Serial.println("[I2S][IDF5] PDM RX enabled @16k/16-bit MONO");
}
#else
// ---------- Legacy I2S API ----------
void configureI2S_PDM_RX_Legacy() {
  // uninstall may log a warning if not previously installed; that's fine
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
    .bck_io_num   = I2S_PIN_NO_CHANGE, // PDM RX doesn't use BCK
    .ws_io_num    = PDM_CLK_PIN,       // PDM clock
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num  = PDM_DATA_PIN
  };

  ESP_ERROR_CHECK(i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL));
  ESP_ERROR_CHECK(i2s_set_pin(I2S_PORT, &pin_cfg));
  // This call prevents the pdm2pcm assert
  ESP_ERROR_CHECK(i2s_set_clk(I2S_PORT, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO));

  Serial.println("[I2S][LEGACY] PDM RX enabled @16k/16-bit MONO");
}
#endif

void resetRecordingSession() {
  isRecording = false;
  recordingStartTime = 0;
  Serial.println("[RESET] Recording session reset - ready for new session");
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("\n=== Enhanced Audio Test System ===");
  Serial.println("🎤 PDM Microphone with Audio Enhancement");
  Serial.printf("📊 Volume Boost: %dx (shift: %d) - Reduced for cleaner audio\n", 1 << VOLUME_SHIFT, VOLUME_SHIFT);
  Serial.printf("🔇 Noise Gate: %d (reduced for less distortion)\n", NOISE_GATE_THRESHOLD);
  Serial.printf("🎵 Sample Rate: %d Hz, Chunk: %d ms\n", SAMPLE_RATE, CHUNK_MS);
  Serial.printf("⏱️ Recording Duration: %d seconds\n", RECORDING_DURATION_MS / 1000);
  Serial.println("================================\n");

  connectWiFi();
  configureWebSocket();

  audioBuf = (uint8_t*) ps_malloc(CHUNK_BYTES);
  if (!audioBuf) audioBuf = (uint8_t*) malloc(CHUNK_BYTES);
  if (!audioBuf) { Serial.println("[Mem] buffer alloc failed"); while (1) delay(1000); }

#if USE_IDF5_NEW_I2S
  configureI2S_PDM_RX_IDF5();
#else
  configureI2S_PDM_RX_Legacy();
#endif

  Serial.println("[SETUP] Audio system initialized successfully!");
  Serial.println("[SETUP] Waiting for WebSocket connection...");
  Serial.println("[SETUP] After connection, recording will start automatically for 10 seconds.");
}

void loop() {
  ws.loop();

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
    if (res != ESP_OK) { Serial.printf("[I2S] read error: 0x%x\n", res); break; }
    totalRead += justRead;
  }

  if (totalRead == CHUNK_BYTES && wsConnected) {
    // Start recording session if not already recording
    if (!isRecording) {
      isRecording = true;
      recordingStartTime = millis();
      Serial.println("🎙️ [RECORDING] Starting 10-second recording session...");
      Serial.println("🎤 [READY] Start speaking now! Audio is being streamed...");
    }

    // Check if recording session should end
    if (isRecording && (millis() - recordingStartTime >= RECORDING_DURATION_MS)) {
      isRecording = false;
      Serial.println("⏹️ [RECORDING] Session completed (10 seconds). Stopping stream.");
      Serial.println("🔄 [STATUS] Ready for next recording session.");
      // Don't send more audio - let the backend timeout and process
      return;
    }

    if (isRecording) {
      // Process audio with enhancements
      processAudio((int16_t*)audioBuf, CHUNK_SAMPLES);
      
      // Report audio level periodically
      reportAudioLevel((int16_t*)audioBuf, CHUNK_SAMPLES);
      
      // Send processed audio
      ws.sendBIN(audioBuf, CHUNK_BYTES);
      
      // Enhanced LED feedback - rapid blink when streaming
      static unsigned long lastBlink = 0;
      if (millis() - lastBlink > 100) { // Faster blink (100ms) when streaming
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        lastBlink = millis();
      }
      
      // Show recording progress
      static unsigned long lastProgressReport = 0;
      if (millis() - lastProgressReport > 2000) { // Every 2 seconds
        unsigned long elapsed = millis() - recordingStartTime;
        unsigned long remaining = RECORDING_DURATION_MS - elapsed;
        Serial.printf("⏱️ [RECORDING] %lu seconds remaining\n", remaining / 1000);
        lastProgressReport = millis();
      }
    }
  } else if (totalRead == CHUNK_BYTES && !wsConnected) {
    // Slow blink when not connected
    static unsigned long lastSlowBlink = 0;
    if (millis() - lastSlowBlink > 1000) { // Slow blink (1s) when not connected
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      lastSlowBlink = millis();
    }
  }
}