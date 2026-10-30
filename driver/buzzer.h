#ifndef BUZZER_H_ /* Include guard */
#define BUZZER_H_

/**
 * @brief turn on the buzzer
 *
 */
void buzzer_on(void);

/**
 * @brief turn off the buzzer
 *
 */
void buzzer_off(void);

/**
 * @brief on and off the buzzer multiple times during a specific time
 *
 * @param toggle    how many times the buzzer switches on and off
 * @param time      how long the buzzer switches on and off
 */
void buzzer_toggle(int toggle, int timeOn, int timeOff);

#endif // BUZZER_H_