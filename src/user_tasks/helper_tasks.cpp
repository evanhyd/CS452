#include "helper_tasks.h"
#include "kernel/syscalls.h"
#include "message.h"
#include "send_util.h"
#include "system_tasks/clock_server_task.h"
#include "util/debug.h"

namespace k4 {

static constexpr unsigned DRAW_TIME_PERIOD = 10;
static constexpr unsigned DRAW_IDLE_PERIOD = 100;

void clockTask() {
  int clockTid = ::WhoIs(clock_server::CLOCK_SERVER_NAME);
  int dispatcherTid = ::WhoIs(MARKLIN_DISPATCHER_SERVER_NAME);
  int trainTrackTid = ::WhoIs(TRAIN_TRACK_SERVER_NAME);
  int uiTid = ::WhoIs(UI_VIEW_SERVER_NAME);

  unsigned drawTimeTick = 0;
  unsigned drawIdleTick = 0;

  for (;;) {
    int res = ::Delay(clockTid, 1);
    if (res < 0) {
      logError("clockTask: Delay failed");
    }
    unsigned ticks = static_cast<unsigned>(res);
    notify(dispatcherTid, DispatcherMsg{.type = DispatcherMsgType::TimerTick, .time{ticks}});
    notify(trainTrackTid, TrainTrackMsg{.type = TrainTrackMsgType::TimerTick, .time{ticks}});

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
