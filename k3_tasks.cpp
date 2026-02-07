#include "k3_tasks.h"
#include "clock_server.h"
#include "debug.h"
#include "fmt.h"
#include "name_server.h"
#include "syscalls.h"
#include "uart.h"

namespace {

struct ReplyMessage {
  int delayTicks;
  int nDelays;
};

void clientTask() {
  const char dummy{};
  ReplyMessage reply;
  if (::Send(::MyParentTid(), &dummy, 0, reinterpret_cast<char*>(&reply), sizeof(ReplyMessage)) < 0) {
    logError("client task failed to request from FirstUserTask");
  }
  int clockServerTid = ::WhoIs(clock_server::CLOCK_SERVER_NAME);
  if (clockServerTid < 0) {
    logError("client task failed to find clock server");
  }
  int myTid = ::MyTid();
  for (int i = 0; i < reply.nDelays; ++i) {
    if (::Delay(clockServerTid, reply.delayTicks) < 0) {
      logError("client task failed to delay");
    }
    char buf[64];
    kit::formatString(buf, "tid: %d, delay interval: %d, delays completed: %d\r\n", myTid, reply.delayTicks, i + 1);
    Uart::syncPrint(Uart::CONSOLE, buf);
  }
}
} // namespace

void k3::FirstUserTask() {
  name_server::createNameServerTask(Priority::HIGH);
  ::Create(Priority::HIGH, clock_server::clockServerTask);
  // TODO: priorities
  int clients[] = {
      ::Create(Priority::LOW, clientTask),
      ::Create(Priority::MEDIUM, clientTask),
      ::Create(Priority::HIGH, clientTask),
      ::Create(Priority::HIGHEST, clientTask),
  };
  ReplyMessage replies[] = {
      {10, 20},
      {23, 9},
      {33, 6},
      {71, 3},
  };
  for (int i = 0; i < 4; ++i) {
    int tid;
    char devnull;
    if (::Receive(&tid, &devnull, 0) < 0) {
      logError("FirstUserTask failed to receive from client");
    }
  }
  for (int i = 0; i < 4; ++i) {
    if (::Reply(clients[i], reinterpret_cast<const char*>(&replies[i]), sizeof(ReplyMessage)) < 0) {
      logError("FirstUserTask failed to reply to client");
    }
  }
}
