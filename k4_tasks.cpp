#include "k4_tasks.h"
#include "can_server.h"
#include "ctfmt.h"
#include "debug.h"
#include "io_server.h"
#include "name_server.h"
#include "syscalls.h"
#include <cstdio>

void k4::FirstUserTask() {
  if (name_server::createNameServerTask(1) < 0) {
    logError("Failed to create name server task");
  }
  int ioServerTid = ::Create(1, io_server::ioServerTask);
  if (ioServerTid < 0) {
    logError("Failed to create ioServerTask");
  }
  int canServerTid = ::Create(1, can_server::canServerTask);
  if (canServerTid < 0) {
    logError("Failed to create canServerTask");
  }

  kit::syncPrintf("FirstUserTask: ioServerTid=%d, canServerTid=%d", ioServerTid, canServerTid);
}
