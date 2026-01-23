#pragma once
#include "fmt.h"
#include "uart.h"
#include <source_location>

#ifdef NDEBUG
[[maybe_unused]] inline void logDebug(size_t, const char*) {}
[[maybe_unused]] inline void logError(const char* cstring, std::source_location loc = std::source_location::current()) {
}
#else
[[maybe_unused]] inline void logDebug(const char* cstring, std::source_location loc = std::source_location::current()) {
  char buffer[256] = {};
  kit::formatString(buffer, "DEBUG %s[%u] - %s: %s", loc.file_name(), uint32_t(loc.line()), loc.function_name(),
                    cstring);
  Uart::syncPrint(Uart::CONSOLE, buffer);
}

[[maybe_unused]] inline void logError(const char* cstring, std::source_location loc = std::source_location::current()) {
  char buffer[256] = {};
  kit::formatString(buffer, "ERROR %s[%u] - %s: %s", loc.file_name(), uint32_t(loc.line()), loc.function_name(),
                    cstring);
  Uart::syncPrint(Uart::CONSOLE, buffer);
  for (;;) {
  }
}
#endif
