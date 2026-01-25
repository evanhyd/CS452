#include "syscall_entry.h"

#include "debug.h"
#include "task_handler.h"
#include "uart.h"

#include <cstdint>

extern "C" [[noreturn]] void _reboot();

void syscallEntry(StackContext* userStack) {
  TaskScheduler::getCurrentTask()->stackPointer = userStack;

  switch (userStack->esr_el1 & 0xFFFF) {
  case 1:
    userStack->x0 = static_cast<uint64_t>(
        syscall_handler::Create(static_cast<int>(userStack->x0), reinterpret_cast<void (*)()>(userStack->x1)));
    TaskScheduler::scheduleNextTask();
    break;
  case 2:
    userStack->x0 = static_cast<uint64_t>(syscall_handler::MyTid());
    TaskScheduler::scheduleNextTask();
    break;
  case 3:
    userStack->x0 = static_cast<uint64_t>(syscall_handler::MyParentTid());
    TaskScheduler::scheduleNextTask();
    break;
  case 4:
    syscall_handler::Yield();
    TaskScheduler::scheduleNextTask();
    break;
  case 5:
    syscall_handler::Exit();
    break;
  default:
    break;
  }

  if (TaskDescriptor* task = TaskScheduler::getNextScheduledTask()) {
    TaskScheduler::activateTask(*task);
  } else {
    Uart::syncPrint(Uart::CONSOLE, "All tasks exited. Press any key to reboot...\n");
    Uart::syncRead(Uart::CONSOLE);
    _reboot();
  }
}

void placeholderEntry() { logError("hit placeholder in vectors"); }
