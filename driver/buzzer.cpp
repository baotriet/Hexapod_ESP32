#include "buzzer.h"
#include "pcf8575.h"
#include <esp32-hal.h>

#define BUZZER_PIN 10

void buzzer_on(void)
{
    pcf8575_setOutput(BUZZER_PIN, 0);
}

void buzzer_off(void)
{
    pcf8575_setOutput(BUZZER_PIN, 1);
}

void buzzer_toggle(int toggle, int timeOn, int timeOff)
{
    for (int i = 0; i < toggle; i++)
    {
        buzzer_on();
        delay(timeOn);
        buzzer_off();
        delay(timeOff);
    }

}