#include "camera.h"

/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-cam-video-streaming-web-server-camera-home-assistant/

  IMPORTANT!!!
   - Select Board "AI Thinker ESP32-CAM"
   - GPIO 0 must be connected to GND to upload a sketch
   - After connecting GPIO 0 to GND, press the ESP32-CAM on-board RESET button to put your board in flashing mode

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/
#include "camera.h"
#include "setup.h"

static bool camera_doneInit = false;

void crop_image(camera_fb_t *fb, unsigned short cropLeft, unsigned short cropRight, unsigned short cropTop, unsigned short cropBottom)
{
    unsigned int maxTopIndex = cropTop * fb->width * 2;
    unsigned int minBottomIndex = ((fb->width * fb->height) - (cropBottom * fb->width)) * 2;
    unsigned short maxX = fb->width - cropRight; // In pixels
    unsigned short newWidth = fb->width - cropLeft - cropRight;
    unsigned short newHeight = fb->height - cropTop - cropBottom;
    size_t newJpgSize = newWidth * newHeight * 2;

    unsigned int writeIndex = 0;
    // Loop over all bytes
    for (int i = 0; i < fb->len; i += 2)
    {
        // Calculate current X, Y pixel position
        int x = (i / 2) % fb->width;

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

void camera_init()
{
#ifdef DEBUG_CAMERA
    Serial.setDebugOutput(false);
#endif // DEBUG_CAMERA
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
    config.frame_size = FRAMESIZE_VGA;
    config.pixel_format = PIXFORMAT_JPEG; // for streaming
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 12;
    config.fb_count = 1;

    // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
    //                      for larger pre-allocated frame buffer.
    if (config.pixel_format == PIXFORMAT_JPEG)
    {
        if (psramFound())
        {
            config.jpeg_quality = 10;
            config.fb_count = 1;
            config.grab_mode = CAMERA_GRAB_LATEST;
        }
        else
        {
            // Limit the frame size when PSRAM is not available
            config.frame_size = FRAMESIZE_CIF;
            config.fb_location = CAMERA_FB_IN_DRAM;
        }
    }

    // Camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
#ifdef DEBUG_CAMERA
        Serial.printf("Camera init failed with error 0x%x", err);
#endif // DEBUG_CAMERA
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    // drop down frame size for higher initial frame rate
    if (config.pixel_format == PIXFORMAT_JPEG)
    {
        s->set_framesize(s, FRAMESIZE_QVGA);
    }
    camera_doneInit = true;
}

bool camera_isInitDone()
{
    return camera_doneInit;
}