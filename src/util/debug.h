#pragma once

#include <source_location>

void logDebug(const char* cstring, std::source_location loc = std::source_location::current());

[[noreturn]] void logError(const char* cstring, std::source_location loc = std::source_location::current());
