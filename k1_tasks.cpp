#include "k1_tasks.h"

#include "fmt.h"
#include "syscalls.h"
#include "task_queue.h"
#include "uart.h"

static void otherTask() {
  char buffer[64];
  kit::formatString(buffer, "MyTid: %d, MyParentTid: %d\r\n", ::MyTid(), ::MyParentTid());
  Uart::syncPrint(Uart::CONSOLE, buffer);
  ::Yield();
  Uart::syncPrint(Uart::CONSOLE, buffer);
}

void firstTask() {
  char buffer[64];

  int t1 = ::Create(Priority::LOW, otherTask);
  kit::formatString(buffer, "Created: %d\r\n", t1);
  Uart::syncPrint(Uart::CONSOLE, buffer);

  int t2 = ::Create(Priority::LOW, otherTask);
  kit::formatString(buffer, "Created: %d\r\n", t2);
  Uart::syncPrint(Uart::CONSOLE, buffer);

  int t3 = ::Create(Priority::HIGH, otherTask);
  kit::formatString(buffer, "Created: %d\r\n", t3);
  Uart::syncPrint(Uart::CONSOLE, buffer);

  int t4 = ::Create(Priority::HIGH, otherTask);
  kit::formatString(buffer, "Created: %d\r\n", t4);
  Uart::syncPrint(Uart::CONSOLE, buffer);

  Uart::syncPrint(Uart::CONSOLE, "FirstUserTask: exiting\r\n");
}
