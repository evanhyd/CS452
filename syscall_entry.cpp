#include "syscall_entry.h"

#include "debug.h"
#include "task_handler.h"
#include "uart.h"

#include <cstdint>

extern "C" [[noreturn]] void _reboot();

void syscallEntry(StackContext* userStack) {
  TaskScheduler::currentTask()->stackPointer = userStack;

  switch (userStack->esr_el1 & 0xFFFF) {
  case 1:
    userStack->x0 = static_cast<uint64_t>(
        syscall_handler::Create(static_cast<int>(userStack->x0), reinterpret_cast<void (*)()>(userStack->x1)));
    break;
  case 2:
    userStack->x0 = static_cast<uint64_t>(syscall_handler::MyTid());
    break;
  case 3:
    userStack->x0 = static_cast<uint64_t>(syscall_handler::MyParentTid());
    break;
  case 4:
    syscall_handler::Yield();
    break;
  case 5:
    syscall_handler::Exit();
    break;
  default:
    break;
  }

  if (TaskDescriptor* task = TaskScheduler::scheduleNextTask()) {
    TaskScheduler::activate(*task);
  } else {
    Uart::syncPrint(Uart::CONSOLE, "All tasks exited. Press any key to reboot...\n");
    Uart::syncRead(Uart::CONSOLE);
    _reboot();
  }
}

void placeholderEntry() { logError("hit placeholder in vectors"); }
