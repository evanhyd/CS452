#include "gic.h"
#include "syscall_handler.h"
#include "task_manager.h"
#include "task_queue.h"
#include "timer.h"
#include "uart.h"

#ifdef __OPTIMIZE__
#define OPT "opt"
#else
#define OPT "noopt"
#endif

#if defined(ENABLE_ICACHE) && defined(ENABLE_DCACHE)
#define CACHE "bcache"
#elif defined(ENABLE_ICACHE)
#define CACHE "icache"
#elif defined(ENABLE_DCACHE)
#define CACHE "dcache"
#else
#define CACHE "nocache"
#endif

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
  Uart::syncPrint(Uart::CONSOLE, "Kitty kernel version: " __DATE__ " / " __TIME__ ", " OPT ", " CACHE "\r\n");

  // Route the interrupts to CPU 0.
  gic::gicd_manager.setInterruptPriorityThresholdForQemu();
  gic::gicd_manager.routeInterupt(gic::InterruptId::TIMER1, 0);
  gic::gicd_manager.routeInterupt(gic::InterruptId::TIMER3, 0);
  gic::gicd_manager.enableInterrupt(gic::InterruptId::TIMER1);

  // Main entry.
  using namespace timer::literals;
  timer::system_timer.setChannel1After(timer::TICK_DURATION);

  [[maybe_unused]] int idleTid = syscall_handler::Create(Priority::LOWEST, []() {
    while (true) {
      asm volatile("wfi");
    }
  });
  TaskDescriptor* task = TaskScheduler::getNextScheduledTask();
  TaskScheduler::activateTask(*task);
}
