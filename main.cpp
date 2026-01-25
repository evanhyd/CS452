#include "fmt.h"
#include "k1_tasks.h"
#include "task.h"
#include "task_handler.h"
#include "uart.h"

// Set up linkers, BSS sections, and constructors.
extern "C" void setup_mmu(); // in mmu.S
using ConstructorType = void (*)();
extern ConstructorType __init_array_start, __init_array_end; // defined in linker script
extern char* rodata;

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
  Uart::syncPrint(Uart::CONSOLE, "Kitty kernel version: " __DATE__ " / " __TIME__ "\r\n");

  // Schedule a bar task.
  [[maybe_unused]] int tid = syscall_handler::Create(Priority::MEDIUM, firstTask);

  TaskDescriptor* task = TaskScheduler::getNextScheduledTask();
  TaskScheduler::activateTask(*task);
}
