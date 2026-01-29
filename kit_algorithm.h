#pragma once
#include <cstddef>
#include <cstdint>

#define KIT_ASSERT(expression)                                                                                         \
  if (!(expression))                                                                                                   \
  asm volatile("udf #1")

extern "C" {
void* memset(void* s, int c, size_t n);
void* memcpy(void* dest, const void* src, size_t n);
int strncmp(const char* s1, const char* s2, size_t n);
char* strncpy(char* dest, const char* src, size_t n);
}
