#ifndef HEX_H_
#define HEX_H_

// Arduino entry points. Runtime state and motion helpers are private to Hex.cpp.

// Initialize hardware, controller state, and the startup pose.
void setup();

// Service input and run the active motion scheduler.
void loop();

#endif // HEX_H_
