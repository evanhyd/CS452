#pragma once
#include <cstddef>
#include <cstdint>

extern "C" {
void* memset(void* s, int c, size_t n);
void* memcpy(void* dest, const void* src, size_t n);
int strncmp(const char* s1, const char* s2, size_t n);
char* strncpy(char* dest, const char* src, size_t n);
size_t strlen(const char* start);
}

namespace kit {
template <typename T> T min(const T& l, const T& r) { return l < r ? l : r; }

template <typename T> T max(const T& l, const T& r) { return l < r ? r : l; }

template <typename It, typename Unary> It find_if(It begin, It end, const Unary& good) {
  for (; begin != end; ++begin) {
    if (good(*begin)) {
      break;
    }
  }
  return begin;
}

template <typename It, typename T> bool contains(It begin, It end, const T& value) {
  for (; begin != end; ++begin) {
    if (*begin == value) {
      return true;
    }
  }
  return false;
}

template <typename It, typename T> void fill(It begin, It end, const T& value) {
  for (; begin != end; ++begin) {
    *begin = value;
  }
}
} // namespace kit
