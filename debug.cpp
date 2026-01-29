#include "debug.h"

#ifdef NDEBUG

#include <utility>

void logDebug(const char* cstring, std::source_location loc) {}
void logError(const char* cstring, std::source_location loc) { std::unreachable(); }

#else

#include "fmt.h"
#include "uart.h"

void logDebug(const char* cstring, std::source_location loc) {
  char buffer[256] = {};
  kit::formatString(buffer, "DEBUG %s[%u] - %s: %s\r\n", loc.file_name(), uint32_t(loc.line()), loc.function_name(),
                    cstring);
  Uart::syncPrint(Uart::CONSOLE, buffer);
}

void logError(const char* cstring, std::source_location loc) {
  char buffer[256] = {};
  kit::formatString(buffer, "ERROR %s[%u] - %s: %s\r\n", loc.file_name(), uint32_t(loc.line()), loc.function_name(),
                    cstring);
  Uart::syncPrint(Uart::CONSOLE, buffer);
  for (;;) {
  }
}

#endif
