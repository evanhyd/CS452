#pragma once
#include <cstddef>
#include <cstdint>

extern "C" {
// Define our own memset to avoid SIMD instructions emitted from the compiler.
inline void *memset(void *s, int c, size_t n) {
  for (char *it = (char *)s; n > 0; --n) {
    *it++ = char(c);
  }
  return s;
}

// Define our own memcpy to avoid SIMD instructions emitted from the compiler.
inline void *memcpy(void *dest, const void *src, size_t n) {
  char *sit = (char *)src;
  char *cdest = (char *)dest;
  for (size_t i = 0; i < n; ++i) {
    *cdest++ = *sit++;
  }
  return dest;
}
}
