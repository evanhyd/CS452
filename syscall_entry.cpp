#include "syscall_entry.h"

#include "debug.h"
#include "task_handler.h"
#include "uart.h"

#include <cstdint>

extern "C" [[noreturn]] void _reboot();

void syscallEntry(StackContext* userStack) {
  TaskScheduler::getCurrentTask()->stackPointer = userStack;

  switch (userStack->esr_el1 & 0xFFFF) {
  case 1: // Create
    userStack->x0 = static_cast<uint64_t>(
        syscall_handler::Create(static_cast<int>(userStack->x0), reinterpret_cast<void (*)()>(userStack->x1)));
    break;
  case 2: // MyTid
    userStack->x0 = static_cast<uint64_t>(syscall_handler::MyTid());
    break;
  case 3: // MyParentTid
    userStack->x0 = static_cast<uint64_t>(syscall_handler::MyParentTid());
    break;
  case 4: // Yield
    syscall_handler::Yield();
    break;
  case 5: // Exit
    syscall_handler::Exit();
    break;
  case 6: // Send
    break;
  case 7: // Receive
    break;
  case 8: // Reply
    userStack->x0 =
        syscall_handler::Reply(int(userStack->x0), reinterpret_cast<const char*>(userStack->x1), size_t(userStack->x2));
    break;
  default:
    break;
  }

  if (TaskDescriptor* task = TaskScheduler::getNextScheduledTask()) {
    TaskScheduler::activateTask(*task);
  } else {
    Uart::syncPrint(Uart::CONSOLE, "All tasks exited. Press any key to reboot...\r\n");
    Uart::syncRead(Uart::CONSOLE);
    _reboot();
  }
}

void placeholderEntry() { logError("hit placeholder in vectors"); }
