/*
  test_camera_upload.ino
  Purpose: Capture an image and POST it to FastAPI server.
  SSID: testing
  Password: jenish1235
*/

#include <WiFi.h>
#include <esp_camera.h>
#include <HTTPClient.h>

// ==== Wi-Fi credentials ====
const char *ssid = "testing";
const char *password = "jenish1235";

// ==== FastAPI server (replace with your PC's LAN IP) ====
const char *serverUrl = "http://10.217.177.125:8000/upload/image";

// ==== Camera pin mapping for Seeed XIAO ESP32-S3 ====
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 10
#define SIOD_GPIO_NUM 40
#define SIOC_GPIO_NUM 39

#define Y9_GPIO_NUM 48
#define Y8_GPIO_NUM 11
#define Y7_GPIO_NUM 12
#define Y6_GPIO_NUM 14
#define Y5_GPIO_NUM 16
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 17
#define Y2_GPIO_NUM 15

#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM 47
#define PCLK_GPIO_NUM 13

void connectToWiFi()
{
    Serial.printf("Connecting to %s", ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

bool initCamera()
{
    camera_config_t config;
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

    // QVGA for smaller upload size
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 15;
    config.fb_count = 1;

    if (esp_camera_init(&config) != ESP_OK)
    {
        Serial.println("Camera init failed!");
        return false;
    }
    Serial.println("Camera init success.");
    return true;
}

bool uploadImage(camera_fb_t *fb)
{
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("WiFi not connected.");
        return false;
    }

    HTTPClient http;
    http.begin(serverUrl);
    http.addHeader("Content-Type", "image/jpeg");

    int httpResponseCode = http.POST(fb->buf, fb->len);
    if (httpResponseCode > 0)
    {
        Serial.printf("Upload finished. Response code: %d\n", httpResponseCode);
        Serial.println(http.getString());
    }
    else
    {
        Serial.printf("Upload failed. Error: %s\n", http.errorToString(httpResponseCode).c_str());
    }

    http.end();
    return (httpResponseCode == 200);
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    connectToWiFi();
    if (!initCamera())
    {
        while (true)
            delay(1000);
    }

    // Capture a frame
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb)
    {
        Serial.println("Failed to capture image.");
        return;
    }
    Serial.printf("Captured %dx%d image, %u bytes\n", fb->width, fb->height, fb->len);

    // Upload it
    uploadImage(fb);

    esp_camera_fb_return(fb);
}

void loop()
{
    // Nothing in loop — one-time test
}
