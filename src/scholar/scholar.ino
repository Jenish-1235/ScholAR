/*
  xiao_edge_wake_snap_stream.ino
  XIAO ESP32S3 (Sense) — Arduino IDE
  Pipeline:
    Wake word (Edge Impulse, "Hello Scholar")
      -> capture OCR-tuned JPEG
      -> upload multipart/form-data to FastAPI
      -> stream 16k/16-bit PCM over WebSocket for ~10s

  Tools:
    Board: Seeed XIAO ESP32S3
    PSRAM: Enabled (OPI)
    CPU Freq: 240 MHz
    Partition: Huge APP (or large enough)

  Wi-Fi: SSID=testing, PASS=jenish1235
  Image upload: http://10.217.177.125:8000/upload/image  (field "file")
  Audio WS    : ws://10.217.177.125:8000/stream/audio     (binary s16le @ 16k)
*/

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebSocketsClient.h>
#include <math.h>
#include <esp_idf_version.h>

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)
  #define USE_IDF5_NEW_I2S 1
  #include "driver/i2s_common.h"
  #include "driver/i2s_pdm.h"
#else
  #define USE_IDF5_NEW_I2S 0
  #include "driver/i2s.h"
#endif

// ==== CAMERA ====
#define CAMERA_MODEL_XIAO_ESP32S3
#include "esp_camera.h"
#include "camera_pins.h"

// ==== EDGE IMPULSE (install your ZIP, then include the _inferencing.h) ====
#include <jenish-1235-project-1_inferencing.h>

// ===== Wi-Fi =====
static const char* WIFI_SSID = "Vedan_Guest";
static const char* WIFI_PASS = "Vedantu@2025";

// ===== Server endpoints =====
static const char* IMG_HOST    = "10.217.177.125";
static const uint16_t IMG_PORT = 8000;
static const char* IMG_PATH    = "/upload/image";   // multipart/form-data, field "file"

static const char* WS_HOST     = "10.217.177.125";
static const uint16_t WS_PORT  = 8000;
static const char* WS_PATH     = "/stream/audio";   // binary PCM 16k s16le

// ===== Audio =====
#define SAMPLE_RATE     16000
#define CHUNK_MS        20
#define CHUNK_SAMPLES   ((SAMPLE_RATE * CHUNK_MS) / 1000)  // 320
#define CHUNK_BYTES     (CHUNK_SAMPLES * 2)                // 640 (s16le)
#define PDM_CLK_PIN     42
#define PDM_DATA_PIN    41

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// ===== KWS / STATE =====
static const char* WAKE_LABEL = "Hello Scholar";  // exact label from EI (case/spaces matter)
static const float KWS_THRESH = 0.75f;
static const uint8_t KWS_DEBOUNCE_FRAMES = 12;    // ~12*20ms = ~240ms
static const uint32_t AWAKE_WINDOW_MS = 10000;    // stream mic for 10s; extended by VAD

enum State { LISTENING, AWAKE };
static State state = LISTENING;
static uint32_t awake_until_ms = 0;

// ring buffer ~1s for KWS (EI default window is typically 1s @ 16k)
static int16_t rb[16000];
static size_t rb_write = 0;

// ===== Globals =====
WebSocketsClient ws;
volatile bool wsConnected = false;
#if USE_IDF5_NEW_I2S
  static i2s_chan_handle_t rx_handle = NULL;
#else
  static const i2s_port_t I2S_PORT = I2S_NUM_0;
#endif
static uint8_t* audioBuf = nullptr;

// ===== Camera OCR tuning knobs =====
static const int AE_LEVEL = 1;    // -2..+2
static const int WB_MODE  = 0;    // 0 auto; 2 fluorescent
static const int JPEG_Q_UXGA = 8; // lower = better quality
static const int JPEG_Q_SVGA = 10;

// ---------- Wi-Fi ----------
void connectWiFi() {
  Serial.printf("Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.printf("\nWiFi IP: %s\n", WiFi.localIP().toString().c_str());
}

// ---------- WebSocket ----------
void wsEvent(WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED) { wsConnected = true;  Serial.printf("[WS] Connected: ws://%s:%u%s\n", WS_HOST, WS_PORT, WS_PATH); }
  if (type == WStype_DISCONNECTED) { wsConnected = false; Serial.println("[WS] Disconnected"); }
}
void wsBegin() {
  ws.begin(WS_HOST, WS_PORT, WS_PATH);
  ws.onEvent(wsEvent);
  ws.setReconnectInterval(2000);
  ws.enableHeartbeat(15000, 3000, 2);
}

// ---------- I2S (PDM) ----------
#if USE_IDF5_NEW_I2S
void i2sInit() {
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

  i2s_pdm_rx_config_t pdm_cfg = { .clk_cfg = clk_cfg, .slot_cfg = slot_cfg, .gpio_cfg = gpio_cfg };
  ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(rx_handle, &pdm_cfg));
  ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
}
#else
void i2sInit() {
  i2s_driver_uninstall(I2S_PORT);
  i2s_config_t cfg = {
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
  i2s_pin_config_t pins = {
    .bck_io_num = I2S_PIN_NO_CHANGE,
    .ws_io_num  = PDM_CLK_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num  = PDM_DATA_PIN
  };
  ESP_ERROR_CHECK(i2s_driver_install(I2S_PORT, &cfg, 0, NULL));
  ESP_ERROR_CHECK(i2s_set_pin(I2S_PORT, &pins));
  ESP_ERROR_CHECK(i2s_set_clk(I2S_PORT, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO));
}
#endif

// ---------- Camera (OCR tuned) ----------
bool initCameraForOCR() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href  = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;   // 1600x1200
    config.jpeg_quality = JPEG_Q_UXGA;    // 8–12 is good
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    config.frame_size = FRAMESIZE_SVGA;   // 800x600
    config.jpeg_quality = JPEG_Q_SVGA;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }
  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_camera_deinit();
  if (esp_camera_init(&config) != ESP_OK) return false;

  sensor_t* s = esp_camera_sensor_get();
  s->set_exposure_ctrl(s, 1);
  s->set_aec2(s, 1);
  s->set_ae_level(s, AE_LEVEL);
  s->set_gain_ctrl(s, 1);
  s->set_gainceiling(s, GAINCEILING_32X);
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_wb_mode(s, WB_MODE);
  s->set_lenc(s, 1);
  s->set_bpc(s, 1);
  s->set_wpc(s, 1);
  s->set_raw_gma(s, 1);
  s->set_dcw(s, 1);
  s->set_colorbar(s, 0);
  s->set_brightness(s, 1);
  s->set_contrast(s, 1);
  s->set_saturation(s, -1);
  return true;
}

// ---------- Upload image (multipart/form-data) ----------
bool uploadImageMultipart(camera_fb_t *fb) {
  if (!fb) return false;
  const char *boundary = "----ESP32Boundary7MA4YWxkTrZu0gW";
  String pre;
  pre += "--"; pre += boundary; pre += "\r\n";
  pre += "Content-Disposition: form-data; name=\"file\"; filename=\"frame.jpg\"\r\n";
  pre += "Content-Type: image/jpeg\r\n\r\n";
  String post = "\r\n--"; post += boundary; post += "--\r\n";
  size_t contentLength = pre.length() + fb->len + post.length();

  WiFiClient client;
  Serial.printf("[HTTP] %s:%u\n", IMG_HOST, IMG_PORT);
  if (!client.connect(IMG_HOST, IMG_PORT)) { Serial.println("[HTTP] connect fail"); return false; }
  client.printf("POST %s HTTP/1.1\r\n", IMG_PATH);
  client.printf("Host: %s\r\n", IMG_HOST);
  client.println("Connection: close");
  client.printf("Content-Type: multipart/form-data; boundary=%s\r\n", boundary);
  client.printf("Content-Length: %u\r\n\r\n", (unsigned)contentLength);
  client.print(pre); client.write(fb->buf, fb->len); client.print(post);

  String status = client.readStringUntil('\n');
  Serial.print(status);
  bool ok = status.startsWith("HTTP/1.1 200");
  unsigned long t0 = millis();
  while (client.connected() && millis() - t0 < 2000) {
    while (client.available()) { Serial.write(client.read()); t0 = millis(); }
  }
  client.stop();
  Serial.printf("[HTTP] Upload %s\n", ok ? "OK" : "FAILED");
  return ok;
}

// ---------- Simple VAD (RMS) to extend awake window ----------
bool frame_has_voice(const int16_t* s, size_t n, int rms_thresh = 300) {
  uint64_t acc=0; for (size_t i=0;i<n;i++){ int32_t v=s[i]; acc += (uint32_t)((v*v)>>10); }
  int rms = (int)sqrt((double)acc / n);
  return rms > rms_thresh;
}

// ---------- Edge Impulse glue ----------
static int ei_get_data(size_t offset, size_t length, float *out) {
  // rb[] holds 1s of int16_t at 16kHz. Convert to float [-1,1].
  for (size_t i = 0; i < length; i++) {
    size_t idx = rb_write + offset + i;
    if (idx >= 16000) idx -= 16000;
    out[i] = (float)rb[idx] / 32768.0f;
  }
  return 0;
}

bool kws_detect_from_ringbuffer() {
  // Make sure model frequency matches our SAMPLE_RATE
  if (EI_CLASSIFIER_FREQUENCY != SAMPLE_RATE) {
    // You can change SAMPLE_RATE or re-train/export the model at 16k
    // Serial.printf("[EI] Model expects %d Hz, we are %d Hz\n", EI_CLASSIFIER_FREQUENCY, SAMPLE_RATE);
  }

  signal_t signal;
  signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
  signal.get_data = &ei_get_data;

  ei_impulse_result_t result = { 0 };
  EI_IMPULSE_ERROR err = run_classifier_continuous(&signal, &result, false);
  if (err != EI_IMPULSE_OK) {
    // Serial.printf("[EI] error: %d\n", err);
    return false;
  }

  float score = 0.f;
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    if (strcmp(result.classification[ix].label, WAKE_LABEL) == 0) {
      score = result.classification[ix].value;
      break;
    }
  }

  // Serial.printf("[EI] %s=%.2f\n", WAKE_LABEL, score); // debug
  return score >= KWS_THRESH;
}

// ---------- Setup / Loop ----------
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN, LOW);

  connectWiFi();
  wsBegin();
  i2sInit();

  audioBuf = (uint8_t*) ps_malloc(CHUNK_BYTES);
  if (!audioBuf) audioBuf = (uint8_t*) malloc(CHUNK_BYTES);
  if (!audioBuf) { Serial.println("[Mem] audio buf fail"); while(1) delay(1000); }

  if (!initCameraForOCR()) { Serial.println("Camera init failed"); while(1) delay(1000); }

  // Warm-up camera AE/AWB
  for (int i=0;i<2;i++){ camera_fb_t* f=esp_camera_fb_get(); if(f) esp_camera_fb_return(f); delay(120); }

  Serial.printf("[EI] Model freq=%d Hz, window=%d, frame size=%d\n",
    EI_CLASSIFIER_FREQUENCY, EI_CLASSIFIER_RAW_SAMPLE_COUNT, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
}

void loop() {
  ws.loop();

  // ---- Read one audio frame ----
  size_t totalRead = 0;
  while (totalRead < CHUNK_BYTES) {
    size_t justRead = 0;
#if USE_IDF5_NEW_I2S
    if (i2s_channel_read(rx_handle, audioBuf + totalRead, CHUNK_BYTES - totalRead, &justRead, portMAX_DELAY) != ESP_OK) break;
#else
    if (i2s_read(I2S_PORT, audioBuf + totalRead, CHUNK_BYTES - totalRead, &justRead, portMAX_DELAY) != ESP_OK) break;
#endif
    totalRead += justRead;
  }
  if (totalRead != CHUNK_BYTES) return;

  int16_t* pcm = (int16_t*)audioBuf;

  // ---- Update ring buffer (1s rolling) ----
  memcpy(&rb[rb_write], pcm, CHUNK_BYTES);
  rb_write += CHUNK_SAMPLES;
  if (rb_write >= 16000) rb_write = 0;

  // ---- STATE MACHINE ----
  static uint8_t debounce_hit = 0;

  if (state == LISTENING) {
    // cheap VAD gate: only run EI when some energy present
    bool has_voice = frame_has_voice(pcm, CHUNK_SAMPLES, 300);
    bool hit = false;
    if (has_voice) {
      hit = kws_detect_from_ringbuffer();
    }
    // debounce ~240ms
    if (hit) { if (debounce_hit < KWS_DEBOUNCE_FRAMES) debounce_hit++; }
    else      { if (debounce_hit > 0) debounce_hit--; }

    if (debounce_hit >= KWS_DEBOUNCE_FRAMES) {
      // ---- WAKE: capture + upload image ----
      camera_fb_t* fb = esp_camera_fb_get();
      if (fb) { uploadImageMultipart(fb); esp_camera_fb_return(fb); }

      // start audio streaming window
      state = AWAKE;
      awake_until_ms = millis() + AWAKE_WINDOW_MS;
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.println("[KWS] WAKE -> streaming");
    }
  } else { // AWAKE
    // stream mic to WS
    if (wsConnected) ws.sendBIN(audioBuf, CHUNK_BYTES);

    // hold awake while talking
    if (frame_has_voice(pcm, CHUNK_SAMPLES, 300)) {
      awake_until_ms = millis() + 1000; // extend 1s per voiced frame
    }
    if (millis() > awake_until_ms) {
      state = LISTENING;
      debounce_hit = 0;
      digitalWrite(LED_BUILTIN, LOW);
      Serial.println("[KWS] back to LISTENING");
    }
  }

  // Tiny yield
  delay(1);
}
