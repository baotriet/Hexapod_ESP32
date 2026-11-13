#ifndef ADS1115_H_ /* Include guard */
#define ADS1115_H_

/**
 * @brief Initialize ads1115 ADC
 *
 */
void ads1115_init();

enum class AdcReadStatus
{
  Pending,
  Ready,
  Error
};

bool ads1115_start(short cell);
AdcReadStatus ads1115_poll(float &voltage);

#endif // ADS1115_H_