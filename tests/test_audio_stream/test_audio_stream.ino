/*
  test_mic_websocket_pdm_driver_only.ino
  - NO <I2S.h> wrapper — uses ESP-IDF I2S drivers only
  - Supports IDF 5.x (new PDM API) and legacy API
  - Streams 16 kHz / 16-bit mono PCM (20 ms = 640B) over WebSocket

  Wi-Fi   : SSID=testing  PASS=jenish1235
  WebSock : ws://10.217.177.125:8000/stream/audio
*/

#include <WiFi.h>
#include <WebSocketsClient.h>
#include <esp_idf_version.h>

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)
  #define USE_IDF5_NEW_I2S 1
  #include "driver/i2s_common.h"
  #include "driver/i2s_pdm.h"
#else
  #define USE_IDF5_NEW_I2S 0
  #include "driver/i2s.h"
#endif

// ---------- Wi-Fi ----------
static const char* WIFI_SSID = "testing";
static const char* WIFI_PASS = "jenish1235";

// ---------- WebSocket ----------
static const char* SERVER_HOST = "10.217.177.125";
static const uint16_t SERVER_PORT = 8000;
static const char* SERVER_PATH = "/stream/audio";

// ---------- Audio ----------
#define SAMPLE_RATE     16000
#define CHUNK_MS        20
#define CHUNK_SAMPLES   ((SAMPLE_RATE * CHUNK_MS) / 1000)  // 320
#define CHUNK_BYTES     (CHUNK_SAMPLES * 2)                // 640 (s16le)
#define VOLUME_SHIFT    0  // optional 0..3

// XIAO ESP32S3 Sense PDM mic pins (CLK=42, DATA=41)
#define PDM_CLK_PIN     42
#define PDM_DATA_PIN    41

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

WebSocketsClient ws;
volatile bool wsConnected = false;

#if USE_IDF5_NEW_I2S
  static i2s_chan_handle_t rx_handle = NULL;
#else
  static const i2s_port_t I2S_PORT = I2S_NUM_0;
#endif

static uint8_t* audioBuf = nullptr;

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
    Serial.printf("[WS] Connected: ws://%s:%u%s\n", SERVER_HOST, SERVER_PORT, SERVER_PATH);
    // ws.se   ndTXT(String("{\"type\":\"meta\",\"codec\":\"pcm_s16le\",\"sample_rate\":") +
    //            SAMPLE_RATE + ",\"channels\":1,\"chunk_ms\":" + CHUNK_MS + "}");
  } else if (type == WStype_DISCONNECTED) {
    wsConnected = false;
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

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

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
    if (VOLUME_SHIFT > 0) {
      int16_t* s = (int16_t*) audioBuf;
      for (size_t i = 0; i < CHUNK_SAMPLES; ++i) {
        int32_t v = (int32_t)s[i] << VOLUME_SHIFT;
        if (v > 32767) v = 32767;
        if (v < -32768) v = -32768;
        s[i] = (int16_t)v;
      }
    }
    ws.sendBIN(audioBuf, CHUNK_BYTES);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // blink while streaming
  }
}
