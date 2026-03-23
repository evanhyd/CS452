#include "train_track_server_task.h"

#include "train_track_util.h"

#include "message.h"

#include "kernel/syscalls.h"
#include "user_tasks/train_track_handler.h"
#include "user_tasks/train_track_server_context.h"
#include "util/debug.h"

namespace k4 {

namespace {} // namespace

void trainTrackTask() {
  if (::RegisterAs(TRAIN_TRACK_SERVER_NAME) < 0) {
    logError("failed to register");
  }

  TrainTrackServerContext context{};
  context.dispatcherTid = ::WhoIs(MARKLIN_DISPATCHER_SERVER_NAME);
  KIT_ASSERT(context.dispatcherTid >= 0, "failed to find dispatcher");

  context.uiTid = ::WhoIs(UI_VIEW_SERVER_NAME);
  KIT_ASSERT(context.uiTid >= 0, "failed to find UI server");

  resetContext(context);

  for (;;) {
    TrainTrackMsg msg;
    int senderTid;
    ::Receive(&senderTid, reinterpret_cast<char*>(&msg), sizeof(msg));
    ::Reply(senderTid, "", 0);

    switch (msg.type) {
    case TrainTrackMsgType::SetSpeedCmd:
      setSpeedCmdHandler(context, msg.setSpeedCmd.trainId, msg.setSpeedCmd.speedLevel);
      break;
    case TrainTrackMsgType::WanderCmd:
      wanderCmdHandler(context, msg.wanderCmd.trainId, msg.wanderCmd.speedLevel);
      break;
    case TrainTrackMsgType::ReverseCmd:
      reverseCmdHandler(context, msg.reverseCmd.trainId);
      break;
    case TrainTrackMsgType::SetSwitchCmd:
      setSwitchCmdHandler(context, msg.setSwitchCmd.switchId, msg.setSwitchCmd.state);
      break;
    case TrainTrackMsgType::SetTrackCmd:
      setTrackCmdHandler(context, msg.setTrackCmd.trackId);
      break;
    case TrainTrackMsgType::SensorEvent:
      sensorEventHandler(context, msg.sensorEvent);
      break;
    case TrainTrackMsgType::GotoCmd:
      gotoCmdHandler(context, msg.gotoCmd.trainId, msg.gotoCmd.speedLevel, msg.gotoCmd.location,
                     msg.gotoCmd.offsetMm * 1000);
      break;
    case TrainTrackMsgType::TimerTick:
      timerTickHandler(context, msg.time.ticks);
      break;
    default:
      logError("invalid train track message");
      break;
    }
  }
}
} // namespace k4
