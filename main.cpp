#include "fmt.h"
#include "task.h"
#include "uart.h"

// Set up linkers, BSS sections, and constructors.
extern "C" void setup_mmu(); // in mmu.S
using ConstructorType = void (*)();
extern ConstructorType __init_array_start, __init_array_end; // defined in linker script
extern char* rodata;

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
  Uart::syncPrint(Uart::CONSOLE, "Kitty kernel version: " __DATE__ " / " __TIME__ "\n");

  // Schedule a bar task.
  int tid = Create(Priority::MEDIUM, bar);

  char buffer[128];
  kit::formatString(buffer, "Task ID %d\n", tid);
  Uart::syncPrint(Uart::CONSOLE, buffer);

  for (;;) {
    TaskDescriptor& task = TaskScheduler::scheduleNextTask();
    TaskScheduler::activate(task);
  }

  return 0;
}
}
