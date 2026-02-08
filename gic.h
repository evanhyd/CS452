#pragma once

#include <cstddef>
#include <cstdint>

#include "syscalls.h"

namespace gic {

enum class InterruptEventId : uint32_t {
  TIMER1 = 97,
  TIMER3 = 99,
};
#define CHECK(x)                                                                                                       \
  static_assert(static_cast<uint32_t>(InterruptEventId::x) == ::EventId::x,                                            \
                "InterruptEventId::" #x " should be the same as ::EventId::" #x)
CHECK(TIMER1);
CHECK(TIMER3);
#undef CHECK

inline constexpr class GicdManager {
  struct Registers {
    volatile uint32_t GICD_CTLR;
    volatile uint32_t GICD_TYPER;
    volatile uint32_t GICD_IIDR;
    uint32_t reserved0[29];
    volatile uint32_t GICD_IGROUPR[32];
    volatile uint32_t GICD_ISENABLER[32];
    volatile uint32_t GICD_ICENABLER[32];
    volatile uint32_t GICD_ISPENDR[32];
    volatile uint32_t GICD_ICPENDR[32];
    volatile uint32_t GICD_ISACTIVER[32];
    volatile uint32_t GICD_ICACTIVER[32];
    volatile uint32_t GICD_IPRIORITYR[255];
    uint32_t reserved1;
    volatile uint32_t GICD_ITARGETSR[255];
    uint32_t reserved2;
    volatile uint32_t GICD_ICFGR[64];
    uint32_t reserved3[64];
    volatile uint32_t GICD_NSACR[64];
    volatile uint32_t GICD_SGIR;
    uint32_t reserved4[3];
    volatile uint32_t GICD_CPENDSGIR[4];
    volatile uint32_t GICD_SPENDSGIR[4];
  };
#define CHECK(x, o) static_assert(offsetof(Registers, x) == o)
  CHECK(GICD_CTLR, 0x000);
  CHECK(GICD_TYPER, 0x004);
  CHECK(GICD_IIDR, 0x008);
  CHECK(GICD_IGROUPR, 0x080);
  CHECK(GICD_ISENABLER, 0x100);
  CHECK(GICD_ICENABLER, 0x180);
  CHECK(GICD_ISPENDR, 0x200);
  CHECK(GICD_ICPENDR, 0x280);
  CHECK(GICD_ISACTIVER, 0x300);
  CHECK(GICD_ICACTIVER, 0x380);
  CHECK(GICD_IPRIORITYR, 0x400);
  CHECK(GICD_ITARGETSR, 0x800);
  CHECK(GICD_ICFGR, 0xC00);
  CHECK(GICD_NSACR, 0xE00);
  CHECK(GICD_SGIR, 0xF00);
  CHECK(GICD_CPENDSGIR, 0xF10);
  CHECK(GICD_SPENDSGIR, 0xF20);
#undef CHECK

  static Registers& regs() {
    static constexpr uintptr_t GICD_BASE = 0xFF840000 + 0x1000;
    return *reinterpret_cast<Registers*>(GICD_BASE);
  }

public:
  void init() const { regs().GICD_CTLR = 1; }

  // Route the interrupt to CPU's IRQ handler.
  void routeInterrupt(InterruptEventId interruptId, uint32_t cpuId) const {
    static constexpr uint32_t interruptPerRegister = 4;
    uint32_t id = static_cast<uint32_t>(interruptId);
    uint32_t registerIndex = id / interruptPerRegister;
    uint32_t byteIndex = id % interruptPerRegister;
    regs().GICD_ITARGETSR[registerIndex] |= 1 << (8 * byteIndex + cpuId);
  }

  // Enable the interrupt.
  void enableInterrupt(InterruptEventId interruptId) const {
    uint32_t id = static_cast<uint32_t>(interruptId);
    uint32_t registerIndex = id / 32;
    uint32_t bitIndex = id % 32;
    regs().GICD_ISENABLER[registerIndex] = 1 << bitIndex;
  }

  // Disable the interrupt.
  void diableInterrupt(InterruptEventId interruptId) const {
    uint32_t id = static_cast<uint32_t>(interruptId);
    uint32_t registerIndex = id / 32;
    uint32_t bitIndex = id % 32;
    regs().GICD_ICENABLER[registerIndex] = 1 << bitIndex;
  }

} gicd_manager;

inline constexpr class GicdcManager {
  struct Registers {
    volatile uint32_t GICC_CTLR;   // RW 0x00000000 CPU Interface Control Register
    volatile uint32_t GICC_PMR;    // RW 0x00000000 Interrupt Priority Mask Register
    volatile uint32_t GICC_BPR;    // RW 0x0000000x a Binary Point Register
    volatile uint32_t GICC_IAR;    // RO 0x000003FF Interrupt Acknowledge Register
    volatile uint32_t GICC_EOIR;   // WO - End of Interrupt Register
    volatile uint32_t GICC_RPR;    // RO 0x000000FF Running Priority Register
    volatile uint32_t GICC_HPPIR;  // RO 0x000003FF Highest Priority Pending Interrupt Register
    volatile uint32_t GICC_ABPR;   // RW 0x0000000xa Aliased Binary Point Register
    volatile uint32_t GICC_AIAR;   // RO 0x000003FF Aliased Interrupt Acknowledge Register
    volatile uint32_t GICC_AEOIR;  // WO - Aliased End of Interrupt Register
    volatile uint32_t GICC_AHPPIR; // RO 0x000003FF Aliased Highest Priority Pending Interrupt Register
    volatile uint32_t reserved0[41];
    volatile uint32_t GICC_APRn[4];   // RW 0x00000000 Active Priorities Registers
    volatile uint8_t GICC_NSAPRn[13]; // RW 0x00000000 Non-secure Active Priorities Registers
    volatile uint8_t reserved1[15];
    volatile uint32_t GICC_IIDR; // RO IMPLEMENTATION DEFINED CPU Interface Identification Register
    volatile uint8_t reserved2[3840];
    volatile uint32_t GICC_DIR; // WO - Deactivate Interrupt Register
  };

#define CHECK(x, o) static_assert(offsetof(Registers, x) == o)
  CHECK(GICC_CTLR, 0x0000);
  CHECK(GICC_PMR, 0x0004);
  CHECK(GICC_BPR, 0x0008);
  CHECK(GICC_IAR, 0x000C);
  CHECK(GICC_EOIR, 0x0010);
  CHECK(GICC_RPR, 0x0014);
  CHECK(GICC_HPPIR, 0x0018);
  CHECK(GICC_ABPR, 0x001C);
  CHECK(GICC_AIAR, 0x0020);
  CHECK(GICC_AEOIR, 0x0024);
  CHECK(GICC_AHPPIR, 0x0028);
  CHECK(GICC_APRn, 0x00D0);
  CHECK(GICC_NSAPRn, 0x00E0);
  CHECK(GICC_IIDR, 0x00FC);
  CHECK(GICC_DIR, 0x1000);
#undef CHECK

  static Registers& regs() {
    static constexpr uintptr_t GICC_BASE = 0xFF840000 + 0x2000;
    return *reinterpret_cast<Registers*>(GICC_BASE);
  }

public:
  void init() const {
    regs().GICC_CTLR = 1;
    regs().GICC_PMR = 0xFF;
  }

  // Sets interrupt state to Active in GIC
  // Returns interruptID.
  InterruptEventId readAndActivateInterruptId() const { return static_cast<InterruptEventId>(regs().GICC_IAR); }

  // Marks interrupt as not active in GIC.
  void deactivateInterrupt(InterruptEventId interruptId) const {
    regs().GICC_EOIR = static_cast<uint32_t>(interruptId);
  }

} gicc_manager;

} // namespace gic
