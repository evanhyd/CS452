#include "k2_tasks.h"
#include "kit_algorithm.h"
#include "name_server.h"
#include "syscall_handler.h"
#include "syscalls.h"
#include "task_manager.h"
#include "task_queue.h"
#include "uart.h"

// Set up linkers, BSS sections, and constructors.
extern "C" void setup_mmu(); // in mmu.S
using ConstructorType = void (*)();
extern ConstructorType __init_array_start, __init_array_end; // defined in linker script
extern char* rodata;

void test() { Uart::syncPrint(Uart::CONSOLE, "test creating\r\n"); }

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

  // Main entry.
  syscall_handler::Create(Priority::MEDIUM, k2::FirstUserTask);

  TaskDescriptor* task = TaskScheduler::getNextScheduledTask();
  TaskScheduler::activateTask(*task);
}
