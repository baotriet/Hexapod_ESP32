#ifndef BATT_MAN_H_ /* Include guard */
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

#endif // BATT_MAN_H_