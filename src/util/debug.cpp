#include "util/debug.h"

#ifdef NDEBUG

#include <utility>

void logDebug(const char* cstring, std::source_location loc) {}
void logError(const char* cstring, std::source_location loc) { std::unreachable(); }

#else

#include "util/ctfmt.h"

void logDebug(const char* cstring, std::source_location loc) {
  kit::syncPrintf("DEBUG %s[%u] - %s: %s\r\n", loc.file_name(), uint32_t(loc.line()), loc.function_name(), cstring);
}

void logError(const char* cstring, std::source_location loc) {
  kit::syncPrintf("ERROR %s[%u] - %s: %s\r\n", loc.file_name(), uint32_t(loc.line()), loc.function_name(), cstring);
  for (;;) {
  }
}

#endif
