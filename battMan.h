#ifndef BATT_MAN_H_ /* Include guard */
#define BATT_MAN_H_

// Call only on the control thread, including during timed moves.
void battMan_update();

/**
 * @brief Check the voltage of battery whether it is safe to run
 *
 * @return true if the voltage is equal to or higher than the minimum guard
 * @return false if otherwise
 */
bool battMan_isPowerOk(void);

/**
 * @brief Return the shared voltage snapshot, or NaN until ready/on error/staleness.
 * @param cell1 Pointer where cell1 voltage will be stored (may be NULL to skip)
 * @param cell2 Pointer where cell2 voltage will be stored (may be NULL to skip)
 */
void battMan_getVoltages(float *cell1, float *cell2);

#endif // BATT_MAN_H_