#pragma once

#include <compare> // auto operator<=>
#include <cstddef>
#include <cstdint>

namespace timer {

inline constexpr uint32_t TICK_DURATION_US = 10000;

// A Time class that can represent the smallest tick precision of the system timer.
class Time {
  constexpr explicit Time(uint32_t microseconds) : microseconds_{microseconds} {}

public:
  constexpr Time() : microseconds_{0} {}

  constexpr static Time fromMicros(uint32_t micros) { return Time{micros}; }
  constexpr static Time fromMillis(uint32_t millis) { return Time{millis * 1000}; }
  constexpr static Time fromSecs(uint32_t seconds) { return Time{seconds * 1'000'000}; }

  constexpr uint32_t micros() const { return microseconds_; }
  constexpr uint32_t millis() const { return microseconds_ / 1000; }
  constexpr uint32_t secs() const { return microseconds_ / 1'000'000; }
  constexpr uint32_t ticks() const { return microseconds_ / TICK_DURATION_US; }

  constexpr friend Time operator+(const Time& a, const Time& b) {
    return Time::fromMicros(a.microseconds_ + b.microseconds_);
  }
  constexpr friend Time operator-(const Time& a, const Time& b) {
    return Time::fromMicros(a.microseconds_ - b.microseconds_);
  }

  constexpr Time& operator+=(const Time& other) { return *this = *this + other; }
  constexpr Time& operator-=(const Time& other) { return *this = *this - other; }

  constexpr auto operator<=>(const Time& other) const = default;

private:
  uint32_t microseconds_;
};

inline constexpr Time TICK_DURATION = Time::fromMicros(TICK_DURATION_US);

namespace literals {

consteval Time operator""_us(unsigned long long micros) { return Time::fromMicros(static_cast<uint32_t>(micros)); }
consteval Time operator""_ms(unsigned long long millis) { return Time::fromMillis(static_cast<uint32_t>(millis)); }
consteval Time operator""_s(unsigned long long seconds) { return Time::fromSecs(static_cast<uint32_t>(seconds)); }

} // namespace literals

inline class SystemTimer {
  static constexpr uintptr_t TIMER_BASE = 0xFE003000;

  struct Registers {
    volatile uint32_t CS;
    volatile uint32_t CLO;
    volatile uint32_t CHI;
    volatile uint32_t C0;
    volatile uint32_t C1;
    volatile uint32_t C2;
    volatile uint32_t C3;
  };
  static_assert(offsetof(Registers, CS) == 0x00);
  static_assert(offsetof(Registers, CLO) == 0x04);
  static_assert(offsetof(Registers, CHI) == 0x08);
  static_assert(offsetof(Registers, C0) == 0x0C);
  static_assert(offsetof(Registers, C1) == 0x10);
  static_assert(offsetof(Registers, C2) == 0x14);
  static_assert(offsetof(Registers, C3) == 0x18);

  static Registers& regs() { return *reinterpret_cast<Registers*>(TIMER_BASE); }

public:
  Time now() const { return Time::fromMicros(regs().CLO); }
  Time since(Time timestamp) const { return now() - timestamp; }
  void setChannel1(Time timestamp) {
    regs().C1 = timestamp.micros();
    lastChannel1SetTime = timestamp;
  }
  void setChannel1After(Time delay) { setChannel1(lastChannel1SetTime + delay); }
  void setChannel3(Time timestamp) {
    regs().C3 = timestamp.micros();
    lastChannel3SetTime = timestamp;
  }
  void setChannel3After(Time delay) { setChannel3(lastChannel3SetTime + delay); }
  void clearChannel1() const { regs().CS = 1 << 1; }
  void clearChannel3() const { regs().CS = 1 << 3; }

private:
  Time lastChannel1SetTime;
  Time lastChannel3SetTime;
} system_timer;

} // namespace timer
