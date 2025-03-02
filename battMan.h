#ifndef BATT_MAN_H_   /* Include guard */
#define BATT_MAN_H_

/**
 * @brief Check the voltage of battery whether it is safe to run
 * 
 * @return true if the voltage is equal to or higher than the minimum guard
 * @return false if otherwise
 */
bool battMan_isPowerOk(void);

#endif // BATT_MAN_H_