/*
  test_camera_upload_ocr_single.ino
  One good-quality capture (OCR tuned) → multipart/form-data upload.

  Wi-Fi: SSID=testing, PASS=jenish1235
  Server: http://10.217.177.125:8000/upload/image
  Field name: "file"
*/

#define CAMERA_MODEL_XIAO_ESP32S3
#include <WiFi.h>
#include <WiFiClient.h>
#include "esp_camera.h"
#include "camera_pins.h"

// ===== Wi-Fi =====
static const char *WIFI_SSID = "Vedan_Guest";
static const char *WIFI_PASS = "Vedantu@2025";

// ===== Server =====
static const char *SERVER_HOST = "10.10.30.172";
static const uint16_t SERVER_PORT = 8000;
static const char *SERVER_PATH = "/upload/image"; // matches your FastAPI router

// ===== Tuning knobs =====
static const int AE_LEVEL = 1;    // -2..+2; bump exposure a touch
static const int WB_MODE = 2;     // 0=auto; try 2 (fluorescent) if green cast
static const int JPEG_Q_UXGA = 8; // lower number = higher quality (8–12 good)
static const int JPEG_Q_SVGA = 10;

// --------- Wi-Fi ----------
void connectWiFi()
{
  Serial.printf("Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(400);
    Serial.print(".");
  }
  Serial.printf("\nWiFi connected. IP: %s\n", WiFi.localIP().toString().c_str());
}

// --------- Camera init (OCR tuned) ----------
bool initCameraForOCR()
{
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
  if (psramFound())
  {
    config.frame_size = FRAMESIZE_UXGA; // 1600x1200
    config.jpeg_quality = JPEG_Q_UXGA;  // 8–12
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
  }
  else
  {
    config.frame_size = FRAMESIZE_SVGA; // 800x600
    config.jpeg_quality = JPEG_Q_SVGA;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }
  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_camera_deinit(); // clean slate just in case
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed: 0x%x\n", err);
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

// --------- Multipart upload (field "file") ----------
bool uploadImageMultipart(camera_fb_t *fb)
{
  if (!fb)
    return false;

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
  Serial.printf("[HTTP] Connecting %s:%u\n", SERVER_HOST, SERVER_PORT);
  if (!client.connect(SERVER_HOST, SERVER_PORT))
  {
    Serial.println("[HTTP] Connect failed");
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
  Serial.print(status);
  bool ok = status.startsWith("HTTP/1.1 200");

  // Optional: drain remaining response
  unsigned long t0 = millis();
  while (client.connected() && millis() - t0 < 3000)
  {
    while (client.available())
    {
      String line = client.readStringUntil('\n');
      Serial.print(line);
      t0 = millis();
    }
  }
  client.stop();
  Serial.printf("[HTTP] Upload %s\n", ok ? "OK" : "FAILED");
  return ok;
}

void setup()
{
  Serial.begin(115200);
  delay(300);

  connectWiFi();
  if (!initCameraForOCR())
  {
    Serial.println("Camera init failed, halting.");
    while (true)
      delay(1000);
  }

  // Warm-up frames to let AEC/AWB settle
  for (int i = 0; i < 2; ++i)
  {
    camera_fb_t *warm = esp_camera_fb_get();
    if (warm)
      esp_camera_fb_return(warm);
    delay(120);
  }

  // Capture one good frame
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb)
  {
    Serial.println("Failed to capture image.");
    return;
  }
  Serial.printf("Captured %dx%d, %u bytes\n", fb->width, fb->height, fb->len);

  // Upload it
  uploadImageMultipart(fb);

  // Release buffer
  esp_camera_fb_return(fb);
}

void loop()
{
  // One-shot test; nothing to do.
}
