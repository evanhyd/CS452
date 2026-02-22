#pragma once
#include <cstddef>
#include <cstdint>

namespace kit {
// String meta info.
bool isPrintable(char c);
char* strAppend(char* bufferEnd, const char* cstring);
char* strAppend(char* bufferEnd, char c);

// String formatting.
uint64_t strToU64(const char* begin, const char* end);
void u32ToStrWithBase(unsigned int num, unsigned int base, char* buffer);
void u64ToStrWithBase(uint64_t num, uint64_t base, char* buffer);
void i32ToStr(int32_t num, char* buffer);
void u32ToStr(uint32_t num, char* buffer);
void u64ToStr(uint64_t num, char* buffer);
void u64ToStrRightAlign(uint64_t number, char* buffer, size_t len);
void tickToTime(uint64_t tick, char* buffer);

// String parsing.
bool extractStr(const char** begin, const char** end, char delimiter);
bool extractU8(const char** begin, const char** end, char delimiter, uint8_t* num);
bool extractU16(const char** begin, const char** end, char delimiter, uint16_t* num);
bool extractU32(const char** begin, const char** end, char delimiter, uint32_t* num);
bool extractU64(const char** begin, const char** end, char delimiter, uint64_t* num);

} // namespace kit
