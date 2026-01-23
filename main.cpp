#include "fmt.h"
#include "task.h"
#include "task_handler.h"
#include "uart.h"

// Set up linkers, BSS sections, and constructors.
extern "C" void setup_mmu(); // in mmu.S
using ConstructorType = void (*)();
extern ConstructorType __init_array_start, __init_array_end; // defined in linker script
extern char* rodata;

void bar() {
  char buffer[32];
  for (unsigned int i = 0; i < 100000; ++i) {
    kit::formatString(buffer, "bar %u\n", i);
    Uart::syncPrint(Uart::CONSOLE, buffer);
    ::Yield();
  }
}

extern "C" void kmain() {
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
  int tid = syscall_handler::Create(Priority::MEDIUM, bar);

  char buffer[128];
  kit::formatString(buffer, "Task ID %d\n", tid);
  Uart::syncPrint(Uart::CONSOLE, buffer);

  TaskDescriptor* task = TaskScheduler::scheduleNextTask();
  TaskScheduler::activate(*task);
}
