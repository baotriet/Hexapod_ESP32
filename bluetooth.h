#ifndef BLUETOOTH_H_   /* Include guard */
#define BLUETOOTH_H_

#include "stdbool.h"
#include "setup.h"

/**
 * @brief Initialize Bluetooth serial with given connect behavior.
 * @param needConnect If true, block until the Bluetooth serial is available
 */
void bluetooth_init(bool needConnect);

/**
 * @brief Read a Bluetooth command packet into the provided buffer.
 * @param inputData Pointer to a buffer of at least 7 bytes where data will be stored
 * @return true if a full packet was received and stored
 */
bool bluetooth_prepareData(byte *inputData);

/**
 * @brief Send two battery cell voltages over Bluetooth as a JSON string.
 *
 * Example sent string: {"cell1":3.720,"cell2":3.710}
 *
 * @param cell1 Voltage of cell 1 in volts
 * @param cell2 Voltage of cell 2 in volts
 * @return true if any bytes were written to the Bluetooth serial
 */
bool bluetooth_sendVoltages(float cell1, float cell2);

#endif // BLUETOOTH_H_