#include <WiFi.h>
#include "esp_camera.h"
#include <HTTPClient.h>
#include <WiFiClientSecure.h>  //Library to send a request using https

#define CONNECTION_TIMEOUT 7  //Time to wait for wifi connection

#define FACE_BUTTON 13  //Face recognition button
#define LOCATION_BUTTON 14  //Location button

//Camera pins
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 21
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 19
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 5
#define Y2_GPIO_NUM 4
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

//WiFi information
const char* ssid = "iPhone 13";
const char* password = "sadstory";

//Server url
String serverName = "blind-assistant-backend-production.up.railway.app";
String facePath = "/recognise-face";
String locationPath = "/set-location-request";

//Specific device id of the user.
String deviceId = "122115";

String locationRequestUrl = "https://" + serverName + locationPath + "?deviceId=" + deviceId;

WiFiClientSecure client;

bool pressed = false;

void setup() {
  pinMode(FACE_BUTTON, INPUT);

  Serial.begin(115200);  //Begin the serial monitor
  delay(1000);

  client.setInsecure();  //Disable the certificate check

  WiFi.begin(ssid, password);
  Serial.println("\nConnecting");
  int timeout_counter = 0;

  //Check the WiFi status for some time.
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(200);
    timeout_counter++;
    if (timeout_counter >= CONNECTION_TIMEOUT * 5) {
      ESP.restart();  //If it is not connected, restart the device.
    }
  }

  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());

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

  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);

  // If camera initialization fails, restart the device.
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }

  sensor_t* s = esp_camera_sensor_get();
  s->set_vflip(s, 0);                   //Flip the camera
  s->set_hmirror(s, 0);                 //Mirror the camera
  s->set_framesize(s, FRAMESIZE_SVGA);  //Set resolution
}

void loop() {
  if (digitalRead(FACE_BUTTON) == pressed) {
    sendPhoto();
    delay(500);
  }

  if (digitalRead(LOCATION_BUTTON) == pressed) {
    sendLocationRequest();
    delay(500);
  }
}

void sendLocationRequest() {
  HTTPClient http;

  http.begin(locationRequestUrl.c_str());

  int httpResponseCode = http.GET();

  if (httpResponseCode > 0) {
    String payload = http.getString();
    Serial.println(payload);
  }

  http.end();
}


String sendPhoto() {
  String getAll;
  String getBody;

  camera_fb_t* fb = NULL;
  fb = esp_camera_fb_get();  //Pointer to the camera frame buffer

  //If camera capture fails, restart the device.
  if (!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
  }

  Serial.println("Connecting to server: " + serverName);

  if (client.connect(serverName.c_str(), 443)) {
    Serial.println("Connection successful!");
    String head = "--BlindAssistant\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--BlindAssistant--\r\n";

    uint32_t imageLen = fb->len;
    uint32_t extraLen = head.length() + tail.length();
    uint32_t totalLen = imageLen + extraLen;  //Total length of the request's content

    client.println("POST " + facePath + "?deviceId=" + deviceId + " HTTP/1.1");
    client.println("Host: " + serverName);
    client.println("Content-Length: " + String(totalLen));
    client.println("Content-Type: multipart/form-data; boundary=BlindAssistant");
    client.println();
    client.print(head);

    uint8_t* fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n = 0; n < fbLen; n = n + 1024) {
      if (n + 1024 < fbLen) {
        client.write(fbBuf, 1024);
        fbBuf += 1024;
      } else if (fbLen % 1024 > 0) {
        size_t remainder = fbLen % 1024;
        client.write(fbBuf, remainder);
      }
    }
    client.print(tail);

    esp_camera_fb_return(fb);

    int timeoutTimer = 40000;  //Time to wait for a response
    long startTimer = millis();
    boolean state = false;

    while ((startTimer + timeoutTimer) > millis()) {
      Serial.print(".");
      delay(100);
      while (client.available()) {
        char c = client.read();
        if (c == '\n') {
          if (getAll.length() == 0) { state = true; }
          getAll = "";
        } else if (c != '\r') {
          getAll += String(c);
        }
        if (state == true) { getBody += String(c); }
        startTimer = millis();
      }
      if (getBody.length() > 0) { break; }
    }
    Serial.println();
    client.stop();
    Serial.println(getBody);  //Print the response
  } else {
    getBody = "Connection to " + serverName + " failed.";
    Serial.println(getBody);
  }
  return getBody;
}