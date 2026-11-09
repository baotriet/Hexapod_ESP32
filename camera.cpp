/*********
  Rui Santos
  Complete project details at
https://RandomNerdTutorials.com/esp32-cam-video-streaming-web-server-camera-home-assistant/

  IMPORTANT!!!
   - Select Board "AI Thinker ESP32-CAM"
   - GPIO 0 must be connected to GND to upload a sketch
   - After connecting GPIO 0 to GND, press the ESP32-CAM on-board RESET button to put your board in
flashing mode

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/
#include "camera.h"
#include "setup.h"
#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include <WiFi.h>

#define PART_BOUNDARY "123456789000000000000987654321"

static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static bool camera_doneInit = false;

static esp_err_t camera_init();

void crop_image(camera_fb_t *fb, unsigned short cropLeft, unsigned short cropRight,
                unsigned short cropTop, unsigned short cropBottom)
{
  unsigned int width = fb->width;
  unsigned int height = fb->height;
  unsigned int maxTopIndex = cropTop * width * 2;
  unsigned int minBottomIndex = ((width * height) - (cropBottom * width)) * 2;
  unsigned short maxX = width - cropRight;
  unsigned short newWidth = width - cropLeft - cropRight;
  unsigned short newHeight = height - cropTop - cropBottom;
  size_t newJpgSize = newWidth * newHeight * 2;

  unsigned int writeIndex = 0;
  // Loop over all bytes
  for (int i = 0; i < fb->len; i += 2)
  {
    // Calculate current X, Y pixel position
    int x = (i / 2) % width;

    // Crop from the top
    if (i < maxTopIndex)
    {
      continue;
    }

    // Crop from the bottom
    if (i > minBottomIndex)
    {
      continue;
    }

    // Crop from the left
    if (x <= cropLeft)
    {
      continue;
    }

    // Crop from the right
    if (x > maxX)
    {
      continue;
    }

    // If we get here, keep the pixels
    fb->buf[writeIndex++] = fb->buf[i];
    fb->buf[writeIndex++] = fb->buf[i + 1];
  }

  // Set the new dimensions of the framebuffer for further use.
  fb->width = newWidth;
  fb->height = newHeight;
  fb->len = newJpgSize;
}

/**
 * @brief Initialize the ESP32 camera peripheral and configure frame settings.
 *
 * This function configures pins, frame size, JPEG quality, and initializes
 * the camera driver. It returns the esp_err_t code from esp_camera_init.
 *
 * @return ESP_OK on success or an esp_err_t error code on failure
 */
static esp_err_t camera_init()
{
#ifdef DEBUG_CAMERA
  Serial.setDebugOutput(false);
#endif // DEBUG_CAMERA
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
  // A 320x240 preview is sufficient between phone joysticks and leaves bandwidth for control.
  config.frame_size = FRAMESIZE_VGA;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  if (config.pixel_format == PIXFORMAT_JPEG)
  {
    if (psramFound())
    {
      config.frame_size = FRAMESIZE_VGA;
      config.fb_location = CAMERA_FB_IN_PSRAM;
      config.grab_mode = CAMERA_GRAB_LATEST;
      config.jpeg_quality = 14;
      // One buffer avoids continuous double-buffer capture competing with walking work.
      config.fb_count = 1;
    }
    else
    {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_HQVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
      config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
      config.jpeg_quality = 12;
      config.fb_count = 1;
    }
  }

  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
#ifdef DEBUG_CAMERA
    Serial.printf("Camera init failed with error 0x%x", err);
#endif // DEBUG_CAMERA
    return err;
  }

  sensor_t *s = esp_camera_sensor_get();
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG)
  {
    // s->set_vflip(s, 1);  // flip it back
    // s->set_contrast(s, 2);
    // s->set_saturation(s, -2); // lower the saturation
  }
  camera_doneInit = true;
  return ESP_OK;
}

bool camera_isInitDone()
{
  return camera_doneInit;
}

esp_err_t camera_stream_handler(httpd_req_t *req)
{
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  byte *_jpg_buf = NULL;
  char part_buf[128];

  if (camera_doneInit == false)
  {
    res = camera_init();
    if (res != ESP_OK)
    {
      return res;
    }
  }

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK)
  {
    return res;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  // Cap delivery at 10 fps instead of saturating Wi-Fi with an unbounded MJPEG loop.
  httpd_resp_set_hdr(req, "X-Framerate", "10");

  while (true)
  {
    int64_t frameStarted = esp_timer_get_time();
    fb = esp_camera_fb_get();
    if (!fb)
    {
#ifdef DEBUG_CAMERA
      Serial.println(F("Camera capture failed"));
#endif
      res = ESP_FAIL;
    }
    else
    {
      _jpg_buf_len = fb->len;
      _jpg_buf = fb->buf;
    }

    if (res == ESP_OK)
    {
      size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    if (res == ESP_OK)
    {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (res == ESP_OK)
    {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (fb)
    {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    }
    else if (_jpg_buf)
    {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK)
    {
      break;
    }
    int64_t remainingUs = 100000 - (esp_timer_get_time() - frameStarted);
    if (remainingUs > 0)
    {
      delay((remainingUs + 999) / 1000);
    }
  }
  return res;
}