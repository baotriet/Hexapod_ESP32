#ifndef ADS1115_H_ /* Include guard */
#define ADS1115_H_

/**
 * @brief Initialize ads1115 ADC
 *
 */
void ads1115_init();

/**
 * @brief Read the voltage from a specific cell
 *
 * @param cell      the cell needed to be measured
 * @return float    the voltage of the cell
 */
float ads1115_readVolt(short cell);

#endif // ADS1115_H_