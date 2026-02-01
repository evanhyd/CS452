#pragma once

#include <cstddef>
#include <cstdint>

namespace timer {

inline constexpr class SystemTimer {
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
  uint32_t now() const { return regs().CLO; }
  uint32_t since(uint32_t timestamp) const { return now() - timestamp; }
} system_timer;

} // namespace timer
