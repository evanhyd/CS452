#pragma once

#include <source_location>

void logDebug(const char* cstring, std::source_location loc = std::source_location::current());

[[noreturn]] void logError(const char* cstring, std::source_location loc = std::source_location::current());

// clang-format off
#define KIT_ASSERT(expr, ...) do { if (!(expr)) { logError(__VA_ARGS__); } } while(false)
// clang-format on
