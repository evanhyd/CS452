#include "k4_tasks.h"
#include "debug.h"
#include "io_server.h"
#include "name_server.h"
#include "syscalls.h"

void k4::FirstUserTask() {
  name_server::createNameServerTask(1);
  ::Create(1, io_server::ioServerTask);
  int ioServerTid = ::WhoIs(io_server::IO_SERVER_NAME);
  if (ioServerTid < 0) {
    logError("FirstUserTask failed to find io server");
  }
  for (;;) {
    int ch = ::Getc(ioServerTid);
    if (ch < 0) {
      logError("FirstUserTask failed to get character from io server");
    }

    for (int i = 0; i < 50; ++i) {
      if (::Putc(ioServerTid, static_cast<unsigned char>(ch)) < 0) {
        logError("FirstUserTask failed to put character to io server");
      }
    }
  }
}
