#include "helper_tasks.h"

#include "k4_tasks/send_util.h"
#include "message.h"

#include "kernel/syscalls.h"
#include "marklin/marklin_message.h"
#include "server_tasks/can_server.h"
#include "server_tasks/clock_server.h"
#include "util/debug.h"

namespace k4 {

void eventListenerTask() {
  int canServerTid = ::WhoIs(can_server::CAN_SERVER_NAME);
  int dispatcherTid = ::WhoIs(DISPATCHER_SERVER_NAME);
  int trackTid = ::WhoIs(TRACK_SERVER_NAME);

  for (;;) {
    marklin::MMessage message;
    ::ReceiveCAN(canServerTid, message);

    if (!message.response) {
      continue;
    }

    if (message.command == marklin::Command::FeedbackEvent && message.dlc == 8) {
      // Sensor event -> Track Server
      uint32_t id = static_cast<uint32_t>(message.data[2] << 8 | message.data[3]) - 1;
      TrackMsg tm{.type = TrackMsgType::SensorEvent,
                  .sensorEvent{
                      .bank = char(id / 16 + 'A'),
                      .number = uint8_t(id % 16 + 1),
                      .oldOccupied = bool(message.data[4]),
                      .newOccupied = bool(message.data[5]),
                  }};
      notify(trackTid, tm);
    } else {
      // CAN response -> Dispatcher Server
      DispatcherMsg dm{.type = DispatcherMsgType::CanResponse, .mmsg = message};
      notify(dispatcherTid, dm);
    }
  }
}

void clockTask() {
  int clockTid = ::WhoIs(clock_server::CLOCK_SERVER_NAME);
  int dispatcherTid = ::WhoIs(DISPATCHER_SERVER_NAME);
  int trainTid = ::WhoIs(TRAIN_SERVER_NAME);
  int trackTid = ::WhoIs(TRACK_SERVER_NAME);
  int uiTid = ::WhoIs(UI_SERVER_NAME);

  for (;;) {
    int ticks = ::Delay(clockTid, 10); // 10 ticks = 100 ms
    if (ticks < 0) {
      logError("clockTask: Delay failed");
    }
    unsigned deciseconds = static_cast<unsigned>(ticks) / 10;
    notify(dispatcherTid, DispatcherMsg{.type = DispatcherMsgType::TimerTick, .time{deciseconds}});
    notify(trainTid, TrainMsg{.type = TrainMsgType::TimerTick, .time{deciseconds}});
    notify(trackTid, TrackMsg{.type = TrackMsgType::TimerTick, .time{deciseconds}});
    notify(uiTid, UIMsg{.type = UIMsgType::DrawTime, .time{deciseconds}});
  }
}

} // namespace k4
