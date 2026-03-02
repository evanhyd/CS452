#include "helper_tasks.h"

#include "k4_tasks/send_util.h"
#include "message.h"

#include "kernel/syscalls.h"
#include "marklin/marklin_message.h"
#include "server_tasks/can_server.h"
#include "server_tasks/clock_server.h"
#include "util/debug.h"

namespace k4 {

static constexpr unsigned DRAW_TIME_PERIOD = 10;
static constexpr unsigned DRAW_IDLE_PERIOD = 100;

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

  unsigned drawTimeTick = 0;
  unsigned drawIdleTick = 0;

  for (;;) {
    int res = ::Delay(clockTid, 1);
    if (res < 0) {
      logError("clockTask: Delay failed");
    }
    unsigned ticks = static_cast<unsigned>(res);
    notify(dispatcherTid, DispatcherMsg{.type = DispatcherMsgType::TimerTick, .time{ticks}});
    notify(trainTid, TrainMsg{.type = TrainMsgType::TimerTick, .time{ticks}});
    notify(trackTid, TrackMsg{.type = TrackMsgType::TimerTick, .time{ticks}});

    if (++drawTimeTick == DRAW_TIME_PERIOD) {
      drawTimeTick = 0;
      notify(uiTid, UIMsg{.type = UIMsgType::DrawSystemTime, .time{ticks}});
    }

    if (++drawIdleTick == DRAW_IDLE_PERIOD) {
      drawIdleTick = 0;
      notify(uiTid, UIMsg{.type = k4::UIMsgType::DrawIdleTime, .empty{}});
    }
  }
}

} // namespace k4
