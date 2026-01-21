#pragma once
#include <cstddef>
#include <cstdint>

#define assert(expression)                                                                                             \
  if (expression)                                                                                                      \
  asm volatile("udf #1")

extern "C" {
void* memset(void* s, int c, size_t n);
void* memcpy(void* dest, const void* src, size_t n);
}

namespace kit {
template <typename It, typename T> It find(It begin, It end, const T& value) {
  while (begin != end && *begin != value) {
    ++begin;
  }
  return begin;
}
} // namespace kit
