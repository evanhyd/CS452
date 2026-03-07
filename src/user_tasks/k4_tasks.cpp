#include "k4_tasks.h"

#include "helper_tasks.h"
#include "marklin_dispatcher_server_task.h"
#include "train_track_server_task.h"
#include "ui_controller_task.h"
#include "ui_view_server_task.h"

#include "kernel/syscalls.h"
#include "system_tasks/can_server_task.h"
#include "system_tasks/clock_server_task.h"
#include "system_tasks/io_server_task.h"
#include "system_tasks/name_server_task.h"
#include "user_tasks/marklin_event_listener_task.h"
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

  ::Create(2, k4::uiViewServerTask);
  ::Create(2, k4::marklinDispatcherServerTask);
  ::Create(2, k4::trainTrackTask);

  ::Create(0, k4::clockTask);
  ::Create(0, k4::marklinEventListenerTask);
  ::Create(0, k4::uiControllerTask);
}
