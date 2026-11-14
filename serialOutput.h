#ifndef SERIAL_OUTPUT_H_
#define SERIAL_OUTPUT_H_

#include <Arduino.h>

#ifndef BUILDCFG_SERIAL_OUTPUT_ENABLED
#define BUILDCFG_SERIAL_OUTPUT_ENABLED 1
#endif

#if !BUILDCFG_SERIAL_OUTPUT_ENABLED
// Include after library headers so suppression applies only to application code.
template <typename Port>
class SilentSerialOutput
{
public:
  explicit SilentSerialOutput(Port &serial) : port(serial) {}

  void begin(unsigned long baud) { port.begin(baud); }
  void setDebugOutput(bool) { port.setDebugOutput(false); }
  int available() { return port.available(); }
  int read() { return port.read(); }
  String readStringUntil(char terminator) { return port.readStringUntil(terminator); }
  int availableForWrite() { return 0; }

  template <typename... Args>
  size_t print(const Args &...) { return 0; }
  template <typename... Args>
  size_t println(const Args &...) { return 0; }
  template <typename... Args>
  size_t printf(const Args &...) { return 0; }
  template <typename... Args>
  size_t write(const Args &...) { return 0; }

private:
  Port &port;
};

inline SilentSerialOutput<decltype(Serial)> &applicationSerial()
{
  static SilentSerialOutput<decltype(Serial)> quiet(Serial);
  return quiet;
}

#define Serial applicationSerial()
#endif

#endif
