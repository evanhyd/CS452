#include "k4_tasks.h"

#include "event_tasks.h"
#include "kernel/syscalls.h"
#include "server_tasks/can_server.h"
#include "server_tasks/clock_server.h"
#include "server_tasks/io_server.h"
#include "server_tasks/name_server.h"
#include "ui_server.h"
#include "util/debug.h"

void k4::FirstUserTask() {
  if (name_server::createNameServerTask(1) < 0) {
    logError("failed to create name server task");
  }
  if (::Create(1, io_server::ioServerTask) < 0) {
    logError("failed to create IO server task");
  }
  if (::Create(1, can_server::canServerTask) < 0) {
    logError("failed to create CAN server task");
  }
  if (::Create(1, clock_server::clockServerTask) < 0) {
    logError("failed to create clock server task");
  }

  ::Create(2, ui_server::uiServerTask);
  ::Create(3, event_tasks::terminalInputTask);
  ::Create(3, event_tasks::eventListenerTask);
  ::Create(8, event_tasks::clockTask);
}
