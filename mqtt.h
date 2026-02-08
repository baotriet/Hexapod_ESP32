#ifndef MQTT_H_   /* Include guard */
#define MQTT_H_

/**
 * @brief Initialize and configure the MQTT client (connects to broker).
 */
void mqtt_init();

/**
 * @brief Attempt to (re)connect to the configured MQTT broker.
 */
void mqtt_connect2mqtt();

/**
 * @brief MQTT message callback invoked for subscribed topics.
 *
 * @param topic Topic string
 * @param payload Raw payload bytes
 * @param length Payload length in bytes
 */
void mqtt_callback(char* topic, byte* payload, unsigned int length);

/**
 * @brief Check whether MQTT input data has been populated by the callback.
 * @return true when MQTT data is ready
 */
bool mqtt_dataReady();

/**
 * @brief Get pointer to internal MQTT input buffer (7 bytes).
 * @return pointer to buffer
 */
byte *mqtt_inputData();

/**
 * @brief Call periodically to maintain the MQTT client loop and reconnections.
 */
void mqtt_update();

/**
 * @brief Set the internal MQTT "data ready" flag.
 * @param dataStatus true to mark data ready, false to clear
 */
void mqtt_setDataReady(bool dataStatus);

/**
 * @brief Publish the two battery cell voltages as a compact JSON string to `topic`.
 *
 * Example payload: {"cell1":3.720,"cell2":3.710}
 *
 * @param topic MQTT topic to publish to (null-terminated string)
 * @param cell1 Voltage of cell 1 in volts
 * @param cell2 Voltage of cell 2 in volts
 * @return true if publish was accepted by the MQTT client
 */
bool mqtt_publishVoltages(const char* topic, float cell1, float cell2);

#endif // MQTT_H_