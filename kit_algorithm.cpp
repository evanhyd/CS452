#include "kit_algorithm.h"

extern "C" {
// Define our own memset to avoid SIMD instructions emitted from the compiler.
void* memset(void* s, int c, size_t n) {
  for (char* it = (char*)s; n > 0; --n) {
    *it++ = char(c);
  }
  return s;
}

// Define our own memcpy to avoid SIMD instructions emitted from the compiler.
void* memcpy(void* dest, const void* src, size_t n) {
  char* sit = (char*)src;
  char* cdest = (char*)dest;
  for (size_t i = 0; i < n; ++i) {
    *cdest++ = *sit++;
  }
  return dest;
}

int strncmp(const char* s1, const char* s2, size_t n) {
  for (size_t i = 0; i < n; ++i) {
    if (s1[i] != s2[i]) {
      return (unsigned char)s1[i] - (unsigned char)s2[i];
    }
    if (s1[i] == '\0') {
      return 0;
    }
  }
  return 0;
}
char* strncpy(char* dest, const char* src, size_t n) {
  size_t i = 0;
  for (; i < n && src[i] != '\0'; ++i) {
    dest[i] = src[i];
  }
  for (; i < n; ++i) {
    dest[i] = '\0';
  }
  return dest;
}
}
