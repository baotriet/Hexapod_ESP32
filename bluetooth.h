#ifndef BLUETOOTH_H_   /* Include guard */
#define BLUETOOTH_H_

#include "stdbool.h"
#include "setup.h"

void bluetooth_init(bool needConnect);
bool bluetooth_prepareData(byte *inputData);

#endif // BLUETOOTH_H_