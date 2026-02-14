#include "k1_tasks.h"
#include "fmt.h"
#include "syscalls.h"
#include "task_queue.h"
#include "uart.h"

static void otherTask() {
  char buffer[64];
  kit::formatString(buffer, "MyTid: %d, MyParentTid: %d\r\n", ::MyTid(), ::MyParentTid());
  Uart::syncPrint(buffer);
  ::Yield();
  Uart::syncPrint(buffer);
}

namespace k1 {
void firstTask() {
  char buffer[64];

  int t1 = ::Create(Priority::LOW, otherTask);
  kit::formatString(buffer, "Created: %d\r\n", t1);
  Uart::syncPrint(buffer);

  int t2 = ::Create(Priority::LOW, otherTask);
  kit::formatString(buffer, "Created: %d\r\n", t2);
  Uart::syncPrint(buffer);

  int t3 = ::Create(Priority::HIGH, otherTask);
  kit::formatString(buffer, "Created: %d\r\n", t3);
  Uart::syncPrint(buffer);

  int t4 = ::Create(Priority::HIGH, otherTask);
  kit::formatString(buffer, "Created: %d\r\n", t4);
  Uart::syncPrint(buffer);

  Uart::syncPrint("FirstUserTask: exiting\r\n");
}
} // namespace k1
