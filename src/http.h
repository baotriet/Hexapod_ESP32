#ifndef HTTP_H_ /* Include guard */
#define HTTP_H_

#include <ArduinoJson.h>

/**
 * @brief Initialize the onboard HTTP servers (data and stream endpoints).
 */
void http_init();
bool http_takeInput(byte *command);
void http_setVoltages(float cell1, float cell2);

/**
 * @brief Get a pointer to the last received input data buffer (7 bytes).
 * @return pointer to internal input buffer
 */
byte *http_inputData();

/**
 * @brief Query whether HTTP input data has been received and is ready.
 * @return true when data is ready
 */
bool http_dataReady();

/**
 * @brief Mark HTTP input data ready/consumed.
 * @param dataStatus true when data is ready, false to clear
 */
void http_setDataReady(bool dataStatus);

/**
 * @brief Reset the internal input data buffer to zeros.
 */
void http_resetInputData();

/**
 * @brief Send a JsonDocument to a remote HTTP endpoint.
 *
 * Serializes `payload` to JSON and POSTs it to `url` with Content-Type: application/json.
 * Preconditions: WiFi must be connected. Returns true for 2xx response codes.
 *
 * @param url Null-terminated URL to POST to (e.g. "http://host/path").
 * @param payload JSON document to send (will be serialized).
 * @return true on HTTP 2xx, false otherwise
 */
bool http_sendJson(const char *url, const JsonDocument &payload);

#endif // HTTP_H_