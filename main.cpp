#include "gic.h"
#include "gpio.h"
#include "k4_tasks.h"
#include "mcp2515.h"
#include "spi.h"
#include "syscall_task_handler.h"
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
  Uart::configAndEnable();
  Uart::syncPrint("Kitty kernel version: " __DATE__ " / " __TIME__ ", " OPT ", " CACHE "\r\n");

  // Route the interrupts to CPU 0.
  gic::gicd_manager.init();
  gic::gicc_manager.init();
  gic::gicd_manager.routeInterrupt(gic::InterruptEventId::TIMER1, 0);
  gic::gicd_manager.routeInterrupt(gic::InterruptEventId::UART_IO, 0);
  gic::gicd_manager.routeInterrupt(gic::InterruptEventId::CAN_IO, 0);
  gic::gicd_manager.enableInterrupt(gic::InterruptEventId::TIMER1);
  gic::gicd_manager.enableInterrupt(gic::InterruptEventId::UART_IO);
  gic::gicd_manager.enableInterrupt(gic::InterruptEventId::CAN_IO);

  spi::init();
  gpio::init();
  mcp2515::init();

  // Main entry.
  using namespace timer::literals;
  timer::system_timer.setChannel1(timer::system_timer.now() + timer::TICK_DURATION);

  createIdleTask();
  syscall_handler::Create(2, k4::FirstUserTask);
  TaskDescriptor* task = TaskScheduler::getNextScheduledTask();
  TaskScheduler::activateTask(*task);
}
