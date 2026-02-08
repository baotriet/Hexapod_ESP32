#ifndef BATT_MAN_H_   /* Include guard */
#define BATT_MAN_H_

/**
 * @brief Check the voltage of battery whether it is safe to run
 * 
 * @return true if the voltage is equal to or higher than the minimum guard
 * @return false if otherwise
 */
bool battMan_isPowerOk(void);

/**
 * @brief Read both battery cell voltages (in volts).
 * @param cell1 Pointer where cell1 voltage will be stored (may be NULL to skip)
 * @param cell2 Pointer where cell2 voltage will be stored (may be NULL to skip)
 */
void battMan_getVoltages(float *cell1, float *cell2);

// Send voltages via HTTP (url), MQTT (topic) and Bluetooth. Any nullptr/NULL parameter is skipped.
// Returns true if at least one transport succeeded.
/**
 * @brief Transport selector used by battMan_sendVoltages to choose how to send data.
 */
typedef enum {
    BATT_TRANSPORT_HTTP = 0,   /**< Send via HTTP POST to provided URL */
    BATT_TRANSPORT_MQTT,       /**< Publish to the provided MQTT topic */
    BATT_TRANSPORT_BLUETOOTH,  /**< Send over Bluetooth serial (endpoint ignored) */
} BattTransport;

// Send voltages using a single transport selected by 'transport'.
// For HTTP, 'endpoint' should be the URL (e.g. "http://host/volt").
// For MQTT, 'endpoint' should be the topic (e.g. "hex/voltages").
// For Bluetooth, 'endpoint' may be NULL and is ignored.
// Returns true if the chosen transport succeeded.
/**
 * @brief Send battery cell voltages using a single transport.
 *
 * For HTTP, `endpoint` must be a full URL (e.g. "http://host/path").
 * For MQTT, `endpoint` must be the MQTT topic (e.g. "hex/voltages").
 * For Bluetooth, `endpoint` is ignored and may be NULL.
 *
 * @param transport Transport to use
 * @param endpoint Endpoint (URL or topic) for the transport; may be NULL for Bluetooth
 * @return true if the chosen transport succeeded
 */
bool battMan_sendVoltages(BattTransport transport, const char* endpoint);

#endif // BATT_MAN_H_