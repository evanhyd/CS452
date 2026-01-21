#include "kit_algorithm.h"
#include "slab_allocator.h"
#include "task.h"
#include "uart.h"
#include <cstddef>
#include <cstdint>

// Set up linkers, BSS sections, and constructors.
extern "C" void setup_mmu(); // in mmu.S
using ConstructorType = void (*)();
extern ConstructorType __init_array_start, __init_array_end; // defined in linker script

void bar() {
  while (true) {
    Uart::syncPrint(Uart::CONSOLE, "yes");
  }
}

extern "C" {
int kmain() {
#if defined(MMU)
  setup_mmu();
#endif
  // Set up C++ constructors.
  for (ConstructorType* ctr = &__init_array_start; ctr < &__init_array_end; ++ctr) {
    (*ctr)();
  }

  // Set up UART.
  Uart::configAndEnable(Uart::CONSOLE);

  Create(Priority::MEDIUM, bar);

  // for (;;) {
  //   currtask = schedule();
  //   request = activate(currtask);
  //   handle(request);
  // }

  return 0;
}
}
