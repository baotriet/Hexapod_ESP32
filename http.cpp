#include "http.h"

StaticJsonDocument<512> doc;
#define PART_BOUNDARY "123456789000000000000987654321"

static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static bool httpDataReady = false;
static byte httpInputData[7];

httpd_handle_t hex_httpd = NULL;

static esp_err_t data_handler(httpd_req_t *req)
{
    // Read the data from the request
    const size_t bufferSize = 1024;
    char buf[bufferSize];
    size_t recvLen = req->content_len;

    if (recvLen > bufferSize)
    {
        recvLen = bufferSize;
    }

    int ret = httpd_req_recv(req, buf, recvLen);
    if (ret <= 0)
    {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT)
        {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }

    buf[ret] = '\0';

    DynamicJsonDocument doc(bufferSize);
    DeserializationError error = deserializeJson(doc, buf);
    if (error)
    {
#ifdef DEBUG_HTTP_DATA
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
#endif // DEBUG_HTTP_DATA
        return ESP_FAIL;
    }
#ifdef DEBUG_HTTP_DATA
    Serial.print(F("Values received: "));
#endif // DEBUG_HTTP_DATA
    for (int i = 0; i < 7; i++)
    {
        String key = "value" + String(i);
        httpInputData[i] = doc[key];
#ifdef DEBUG_HTTP_DATA
        Serial.print(httpInputData[i]);
#endif // DEBUG_HTTP_DATA
    }
#ifdef DEBUG_HTTP_DATA
    Serial.println();
#endif // DEBUG_HTTP_DATA

    httpDataReady = true;
    httpd_resp_send(req, "Commands received", HTTPD_RESP_USE_STRLEN);
    
    sensor_t *s = esp_camera_sensor_get();

    s->set_vflip(s, 1); // flip it back
    // s->set_contrast(s, 2);
    // s->set_saturation(s, -2); // lower the saturation

    return ESP_OK;
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;
    char *part_buf[128];

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK)
    {
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "60");

    while (true)
    {
        fb = esp_camera_fb_get();
        if (!fb)
        {
            Serial.println(F("Camera capture failed"));
            res = ESP_FAIL;
        }
        else
        {
            _jpg_buf_len = fb->len;
            _jpg_buf = fb->buf;
        }

        if (res == ESP_OK)
        {
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
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
        delay(100);
    }
    return res;
}

void http_init()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    httpd_uri_t data_uri = {
        .uri = "/values",
        .method = HTTP_POST,
        .handler = data_handler,
        .user_ctx = NULL};

    httpd_uri_t stream_uri = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = stream_handler,
        .user_ctx = NULL};

    if (httpd_start(&hex_httpd, &config) == ESP_OK)
    {
        httpd_register_uri_handler(hex_httpd, &data_uri);
    }

    config.server_port += 1;
    config.ctrl_port += 1;
    if (httpd_start(&hex_httpd, &config) == ESP_OK)
    {
        httpd_register_uri_handler(hex_httpd, &stream_uri);
    }
}

byte *http_inputData()
{
    return httpInputData;
}

bool http_dataReady()
{
    return httpDataReady;
}

void http_setDataReady(bool dataStatus)
{
    httpDataReady = dataStatus;
}

void http_resetInputData()
{
    memset(httpInputData, 0, sizeof(httpInputData));
}