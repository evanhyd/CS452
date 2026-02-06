#include "syscall_entry.h"

#include "debug.h"
#include "fmt.h"
#include "irq_handler.h"
#include "syscall_comm_handler.h"
#include "syscall_handler.h"
#include "task_manager.h"
#include "uart.h"

extern "C" [[noreturn]] void _reboot();

void syscallEntry(StackContext* userStack) {
  auto currTask = TaskScheduler::getCurrentTask();
  currTask->stackPointer = userStack;

  switch (userStack->esr_el1 & 0xFFFF) {
  case 1: // Create
    currTask->setRetValue(
        syscall_handler::Create(static_cast<int>(userStack->x0), reinterpret_cast<void (*)()>(userStack->x1)));
    break;
  case 2: // MyTid
    currTask->setRetValue(syscall_handler::MyTid());
    break;
  case 3: // MyParentTid
    currTask->setRetValue(syscall_handler::MyParentTid());
    break;
  case 4: // Yield
    syscall_handler::Yield();
    break;
  case 5: // Exit
    syscall_handler::Exit();
    break;
  case 6: // Send
    currTask->setRetValue(syscall_handler::Send(
        static_cast<int>(userStack->x0), reinterpret_cast<const char*>(userStack->x1), static_cast<int>(userStack->x2),
        reinterpret_cast<char*>(userStack->x3), static_cast<int>(userStack->x4)));
    break;
  case 7: // Receive
    currTask->setRetValue(syscall_handler::Receive(reinterpret_cast<int*>(userStack->x0),
                                                   reinterpret_cast<char*>(userStack->x1),
                                                   static_cast<int>(userStack->x2)));
    break;
  case 8: // Reply
    currTask->setRetValue(syscall_handler::Reply(static_cast<int>(userStack->x0),
                                                 reinterpret_cast<const char*>(userStack->x1),
                                                 static_cast<int>(userStack->x2)));
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

void interruptEntry(StackContext* userStack) {
  auto currTask = TaskScheduler::getCurrentTask();
  currTask->stackPointer = userStack;

  irq_handler::interruptEntry();

  if (TaskDescriptor* task = TaskScheduler::getNextScheduledTask()) {
    TaskScheduler::activateTask(*task);
  } else {
    logError("No tasks to schedule after IRQ!\r\n");
  }
}

void placeholderEntry(int group, int entry) {
  char buf[64];
  kit::formatString(buf, "hit placeholder in vectors: %d, %d\r\n", group, entry);
  logError(buf);
}
