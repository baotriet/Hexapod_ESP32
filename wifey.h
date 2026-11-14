#ifndef WIFI_H_ /* Include guard */
#define WIFI_H_

/**
 * @brief Initialize wifi module
 *
 */
void wifi_init();
bool wifi_ready();
void wifi_update(bool allowConfiguration);

#endif // WIFI_H_