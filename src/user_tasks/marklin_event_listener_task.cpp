#include "marklin_event_listener_task.h"
#include "marklin/marklin_event.h"
#include "send_util.h"

#include "kernel/syscalls.h"
#include "marklin/marklin_message.h"
#include "system_tasks/can_server_task.h"
#include "user_tasks/message.h"

namespace k4 {

// Monitor and notify the marklin dispatcher server if receives any marklin message.
void marklinEventListenerTask() {
  int canServerTid = ::WhoIs(can_server::CAN_SERVER_NAME);
  int dispatcherTid = ::WhoIs(MARKLIN_DISPATCHER_SERVER_NAME);
  int trainTrackTid = ::WhoIs(TRAIN_TRACK_SERVER_NAME);

  for (;;) {
    marklin::MMessage msg;
    ::ReceiveCAN(canServerTid, msg);

    if (!msg.response) {
      continue;
    }

    if (msg.command == marklin::Command::FeedbackEvent && msg.dlc == 8) {
      notify(trainTrackTid, TrainTrackMsg{
                                .type = TrainTrackMsgType::SensorEvent,
                                .sensorEvent = marklin::SensorTriggeredEvent{msg},
                            });
    } else {
      // CAN response -> Dispatcher Server
      DispatcherMsg dm{.type = DispatcherMsgType::ReceiveMsg, .mmsg = msg};
      notify(dispatcherTid, dm);
    }
  }
}
} // namespace k4
