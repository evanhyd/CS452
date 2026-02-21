#include "helper_tasks.h"

#include "message.h"

#include "kernel/syscalls.h"
#include "marklin/marklin_message.h"
#include "server_tasks/can_server.h"
#include "server_tasks/clock_server.h"
#include "server_tasks/io_server.h"
#include "util/debug.h"

namespace k4 {

static void notify(int mainTid, const Message& msg) {
  char devnull;
  ::Send(mainTid, reinterpret_cast<const char*>(&msg), sizeof(msg), &devnull, 0);
}

void keypressTask() {
  int ioServerTid = ::WhoIs(io_server::IO_SERVER_NAME);
  int mainTid = ::WhoIs(MAIN_SERVER_NAME);

  for (;;) {
    int c = ::Getc(ioServerTid);
    if (c < 0) {
      logError("Getc failed");
    }
    notify(mainTid, {.type = MessageType::KeyPress, .keyPress{static_cast<char>(c)}});
  }
}

void eventListenerTask() {
  int canServerTid = ::WhoIs(can_server::CAN_SERVER_NAME);
  int mainTid = ::WhoIs(MAIN_SERVER_NAME);

  for (;;) {
    marklin::MMessage message;
    ::ReceiveCAN(canServerTid, message);

    if (!message.response) {
      continue;
    }

    Message msgToMain;
    if (message.command == marklin::Command::FeedbackEvent && message.dlc == 8) {
      uint32_t id = static_cast<uint32_t>(message.data[2] << 8 | message.data[3]) - 1;
      msgToMain = {.type = MessageType::SensorEvent,
                   .sensorEvent{
                       .bank = char(id / 16 + 'A'),
                       .number = uint8_t(id % 16 + 1),
                       .oldOccupied = bool(message.data[4]),
                       .newOccupied = bool(message.data[5]),
                   }};
    } else {
      msgToMain = {.type = MessageType::CanResponse, .canResponse{message}};
    }
    notify(mainTid, msgToMain);
  }
}

void clockTask() {
  int clockTid = ::WhoIs(clock_server::CLOCK_SERVER_NAME);
  int mainTid = ::WhoIs(MAIN_SERVER_NAME);

  for (;;) {
    int ticks = ::Delay(clockTid, 10); // 10 ticks = 100 ms
    if (ticks < 0) {
      logError("Delay failed");
    }
    unsigned deciseconds = static_cast<unsigned>(ticks) / 10;
    notify(mainTid, {.type = MessageType::Timer, .timerUpdate{deciseconds}});
  }
}

void reverseWorkerTask() {
  int parentTid = ::MyParentTid();
  ReverseArgs args;
  Message request{.type = MessageType::StartReverse, .empty{}};
  ::Send(parentTid, reinterpret_cast<const char*>(&request), sizeof(request), reinterpret_cast<char*>(&args),
         sizeof(args));

  // wait, based on old speed
  int waitTicks = 50 + args.oldSpeed * 25;
  ::Delay(args.clockServerTid, waitTicks);

  notify(parentTid, {.type = MessageType::EndReverse, .endReverse{args.trainNo}});
}

} // namespace k4
