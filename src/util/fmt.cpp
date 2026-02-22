#include "util/fmt.h"
#include "util/kit_algorithm.h"
#include <cstdarg>

namespace {
constexpr size_t SYSTEM_TIMER_CLOCK_FREQUENCY = 1000000;
}

namespace kit {

// Return true if the character is visible on the screen.
bool isPrintable(char c) {
  switch (c) {
  case ' ':
  case '!':
  case '"':
  case '#':
  case '$':
  case '%':
  case '&':
  case '\'':
  case '(':
  case ')':
  case '*':
  case '+':
  case ',':
  case '-':
  case '.':
  case '/':
  case '0':
  case '1':
  case '2':
  case '3':
  case '4':
  case '5':
  case '6':
  case '7':
  case '8':
  case '9':
  case ':':
  case ';':
  case '<':
  case '=':
  case '>':
  case '?':
  case '@':
  case 'A':
  case 'B':
  case 'C':
  case 'D':
  case 'E':
  case 'F':
  case 'G':
  case 'H':
  case 'I':
  case 'J':
  case 'K':
  case 'L':
  case 'M':
  case 'N':
  case 'O':
  case 'P':
  case 'Q':
  case 'R':
  case 'S':
  case 'T':
  case 'U':
  case 'V':
  case 'W':
  case 'X':
  case 'Y':
  case 'Z':
  case '[':
  case '\\':
  case ']':
  case '^':
  case '_':
  case '`':
  case 'a':
  case 'b':
  case 'c':
  case 'd':
  case 'e':
  case 'f':
  case 'g':
  case 'h':
  case 'i':
  case 'j':
  case 'k':
  case 'l':
  case 'm':
  case 'n':
  case 'o':
  case 'p':
  case 'q':
  case 'r':
  case 's':
  case 't':
  case 'u':
  case 'v':
  case 'w':
  case 'x':
  case 'y':
  case 'z':
  case '{':
  case '|':
  case '}':
  case '~':
    return true;
  default:
    return false;
  }
}

// Append a cstring to the end of the buffer.
// Return one pass the end of the buffer.
// Null terminated.
char* strAppend(char* bufferEnd, const char* cstring) {
  while (*cstring) {
    *bufferEnd = *cstring;
    ++bufferEnd;
    ++cstring;
  }
  *bufferEnd = '\0';
  return bufferEnd;
}

// Append a char to the end of the buffer.
// Return one pass the end of the buffer.
// Null terminated.
char* strAppend(char* bufferEnd, char c) {
  bufferEnd[0] = c;
  bufferEnd[1] = '\0';
  return bufferEnd + 1;
}

// String to uint64.
uint64_t strToU64(const char* begin, const char* end) {
  uint64_t number = 0;
  for (; begin != end; ++begin) {
    number = number * 10 + (*begin - '0');
  }
  return number;
}

// Unsigned int to string, with base.
// Null terminated.
void u32ToStrWithBase(uint32_t num, uint32_t base, char* buffer) {
  uint32_t n = 0;
  uint32_t d = 1;

  while ((num / d) >= base)
    d *= base;
  while (d != 0) {
    uint32_t dgt = num / d;
    num %= d;
    d /= base;
    if (n || dgt > 0 || d == 0) {
      *buffer++ = char(dgt + (dgt < 10 ? '0' : 'A' - 10));
      ++n;
    }
  }
  *buffer = 0;
}

// Unsigned int to string, with base.
// Null terminated.
void u64ToStrWithBase(uint64_t num, uint64_t base, char* buffer) {
  uint64_t n = 0;
  uint64_t d = 1;

  while ((num / d) >= base)
    d *= base;
  while (d != 0) {
    uint64_t dgt = num / d;
    num %= d;
    d /= base;
    if (n || dgt > 0 || d == 0) {
      *buffer++ = char(dgt + (dgt < 10 ? '0' : 'A' - 10));
      ++n;
    }
  }
  *buffer = 0;
}

// int32 to base 10 string.
// Null terminated.
void i32ToStr(int32_t num, char* buffer) {
  if (num < 0) {
    num = -num;
    *buffer++ = '-';
  }
  u32ToStrWithBase(uint32_t(num), 10, buffer);
}

// Uint32 to base 10 string.
// Null terminated.
void u32ToStr(uint32_t num, char* buffer) { u32ToStrWithBase(num, 10, buffer); }

// Uint64 to base 10 string.
// Null terminated.
void u64ToStr(uint64_t num, char* buffer) { u64ToStrWithBase(num, 10, buffer); }

// Format uint64 to string align to right. Buffer values before the number are filled with 0.
// Null terminated.
void u64ToStrRightAlign(uint64_t number, char* buffer, size_t len) {
  buffer[len] = '\0';
  for (; len-- > 0; number /= 10) {
    buffer[len] = char(number % 10 + '0');
  }
}

// Format ticks to HH:MM:SS.a time format.
// Null terminated.
void tickToTime(uint64_t tick, char* buffer) {
  uint64_t totalSeconds = tick / SYSTEM_TIMER_CLOCK_FREQUENCY;
  uint64_t hours = totalSeconds / 3600;
  uint64_t mins = totalSeconds / 60 - hours * 60;
  uint64_t seconds = totalSeconds % 60;
  uint64_t subseconds = (tick % SYSTEM_TIMER_CLOCK_FREQUENCY) / 100000;

  u64ToStrRightAlign(hours, buffer, 2);
  buffer[2] = ':';
  u64ToStrRightAlign(mins, buffer + 3, 2);
  buffer[5] = ':';
  u64ToStrRightAlign(seconds, buffer + 6, 2);
  buffer[8] = '.';
  u64ToStrRightAlign(subseconds, buffer + 9, 1);
}

// Extract the first token after end, and output the range to begin and end.
// Return true if a token is extracted, false if no more token to extract.
bool extractStr(const char** begin, const char** end, char delimiter) {
  const char* string = *end;
  if (string == NULL || *string == '\0') {
    return false;
  }

  *begin = NULL;
  *end = NULL;

  for (;; ++string) {
    if (*string == delimiter) {
      // Leading delimiter, discard it.
      if (*begin == NULL) {
        continue;
      }

      // Trailing delimiter, we found the token.
      *end = string;
      return true;
    } else {
      // First char in the token.
      if (*begin == NULL) {
        *begin = string;
      }

      // Reach the end, must extract the token.
      if (*string == '\0') {
        *end = string;
        return true;
      }
    }
  }
}

// Extract the first uint8 token after end, and output the range to begin and end.
// Return true if a token is extracted, false if no more token to extract.
// Warning: This does not check if the extracted number is well formed.
bool extractU8(const char** begin, const char** end, char delimiter, uint8_t* num) {
  if (!extractStr(begin, end, delimiter)) {
    return false;
  }
  *num = (uint8_t)strToU64(*begin, *end);
  return true;
}

// Extract the first uint16 token after end, and output the range to begin and end.
// Return true if a token is extracted, false if no more token to extract.
// Warning: This does not check if the extracted number is well formed.
bool extractU16(const char** begin, const char** end, char delimiter, uint16_t* num) {
  if (!extractStr(begin, end, delimiter)) {
    return false;
  }
  *num = (uint16_t)strToU64(*begin, *end);
  return true;
}

// Extract the first uint32 token after end, and output the range to begin and end.
// Return true if a token is extracted, false if no more token to extract.
// Warning: This does not check if the extracted number is well formed.
bool extractU32(const char** begin, const char** end, char delimiter, uint32_t* num) {
  if (!extractStr(begin, end, delimiter)) {
    return false;
  }
  *num = (uint32_t)strToU64(*begin, *end);
  return true;
}

// Extract the first uint64 token after end, and output the range to begin and end.
// Return true if a token is extracted, false if no more token to extract.
// Warning: This does not check if the extracted number is well formed.
bool extractU64(const char** begin, const char** end, char delimiter, uint64_t* num) {
  if (!extractStr(begin, end, delimiter)) {
    return false;
  }
  *num = strToU64(*begin, *end);
  return true;
}

} // namespace kit
