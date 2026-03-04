#include "train_track_server_task.h"

#include "message.h"

#include "kernel/syscalls.h"
#include "marklin/marklin_message.h"
#include "marklin/marklin_pathfinding.h"
#include "marklin/marklin_train_track.h"
#include "send_util.h"
#include "util/debug.h"
#include "util/history.h"

#include <array>
#include <cstdint>

namespace k4 {

namespace {

constexpr int VELOCITY = 3; // mm per tick

// Dispatcher Operations
void sendToDispatcher(int dispatcherTid, const marklin::MMessage& mmsg) {
  notify(dispatcherTid, DispatcherMsg{.type = DispatcherMsgType::SendMsg, .mmsg = mmsg});
}

// UI Operations
void sendSwitchToUI(int uiTid, uint8_t switchId, marklin::SwitchState state) {
  notify(uiTid, UIMsg{.type = UIMsgType::UpdateSwitch, .switchUpdate{.switchId = switchId, .state = state}});
}

void sendAllSwitchesToUI(int uiTid, marklin::TrainTrackState& ttState) {
  for (uint8_t i = 1; i <= 18; ++i) {
    sendSwitchToUI(uiTid, i, ttState.getSwitchState(i));
  }
  for (uint8_t i = 153; i <= 156; ++i) {
    sendSwitchToUI(uiTid, i, ttState.getSwitchState(i));
  }
}

void sendSensorsToUI(int uiTid, const History<SensorHistoryEntry, SENSOR_HISTORY_SIZE>& history) {
  UIMsg ui;
  ui.type = UIMsgType::RedrawSensors;
  unsigned idx = 0;
  for (const auto& e : history) {
    ui.sensors.entries[idx++] = e;
  }
  ui.sensors.count = idx;
  notify(uiTid, ui);
}

// Utils
void initTrain(int dispatcherTid, marklin::TrainTrackState& ttState, uint8_t trainId) {
  if (marklin::TrainState& t = ttState.trains[trainId]; !t.touched) {
    t.touched = true;
    sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainFunctionState(trainId, marklin::TrainFunction::F0, 1));
    sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainDirection(trainId, marklin::TrainDirection::Forward));
  }
}

} // namespace

void trainTrackTask() {
  if (::RegisterAs(TRAIN_TRACK_SERVER_NAME) < 0) {
    logError("failed to register");
  }
  int dispatcherTid = ::WhoIs(MARKLIN_DISPATCHER_SERVER_NAME);
  if (dispatcherTid < 0) {
    logError("failed to find dispatcher");
  }
  int uiTid = ::WhoIs(UI_VIEW_SERVER_NAME);
  if (uiTid < 0) {
    logError("failed to find UI server");
  }

  unsigned currentTicks = 0;
  marklin::TrainTrackState ttState{};
  History<SensorHistoryEntry, SENSOR_HISTORY_SIZE> sensorHistory;

  sendAllSwitchesToUI(uiTid, ttState);

  for (;;) {
    // Receive a train track message.
    TrainTrackMsg msg;
    int senderTid;
    ::Receive(&senderTid, reinterpret_cast<char*>(&msg), sizeof(msg));
    ::Reply(senderTid, "", 0);

    switch (msg.type) {
    case TrainTrackMsgType::SetSpeedCmd: {
      // Check argument ranges.
      uint8_t trainId = msg.setSpeedCmd.trainId;
      uint8_t speed = msg.setSpeedCmd.speed;
      if (speed > 14) {
        logError("invalid train speed level");
      }
      if (trainId == 0 || trainId > marklin::NUM_TRAINS) {
        logError("invalid train id");
      }

      // Dispatch commands.
      initTrain(dispatcherTid, ttState, trainId);
      marklin::TrainState& t = ttState.trains[trainId];
      t.speed = speed;
      if (t.state == marklin::TrainState::State::Idle) {
        // If reversing, will set speed after reverse countdown finishes.
        sendToDispatcher(dispatcherTid,
                         marklin::MMessage::setTrainSpeed(trainId, marklin::convertSpeedLevelToSpeed(speed)));
      }
      notifyStatusToUI(uiTid, "Set train %u to speed %u.", trainId, speed);
      break;
    }
    case TrainTrackMsgType::ReverseCmd: {
      uint8_t trainId = msg.reverseCmd.trainId;
      if (trainId == 0 || trainId > marklin::NUM_TRAINS) {
        logError("invalid train id");
      }

      initTrain(dispatcherTid, ttState, trainId);
      marklin::TrainState& t = ttState.trains[trainId];
      if (t.state == marklin::TrainState::State::Reversing) {
        // already reversing, cancel existing reverse command
        t.state = marklin::TrainState::State::Idle;
        notifyStatusToUI(uiTid, "Cancelled reversing train %u.", trainId);
        sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainSpeed(trainId, t.speed));
        break;
      }

      // stop the train
      sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainSpeed(trainId, 0));

      // begin reverse countdown
      t.state = marklin::TrainState::State::Reversing;
      t.reverseCountdownTicks = 50 + static_cast<unsigned>(t.speed) * 25;

      notifyStatusToUI(uiTid, "Reversing train %u in %u ticks.", trainId, t.reverseCountdownTicks);
      break;
    }
    case TrainTrackMsgType::SetSwitchCmd: {
      ttState.setSwitchState(msg.setSwitchCmd.switchId, msg.setSwitchCmd.state);
      sendToDispatcher(dispatcherTid,
                       marklin::MMessage::setSwitchState(msg.setSwitchCmd.switchId, msg.setSwitchCmd.state));
      sendSwitchToUI(uiTid, msg.setSwitchCmd.switchId, msg.setSwitchCmd.state);
      break;
    }
    case TrainTrackMsgType::SensorEvent: {
      if (msg.sensorEvent.state != marklin::SensorState::Occupied) {
        break;
      }

      size_t sensorIndex = marklin::getSensorIndex(msg.sensorEvent.id);
      marklin::TrackNode* currentNode = &ttState.getCurrentTrack()[sensorIndex];

      if (!sensorHistory.empty()) {
        auto& lastEntry = sensorHistory.back();
        if (lastEntry.hasPrediction && lastEntry.predictedId == msg.sensorEvent.id) {
          lastEntry.timeErrorTicks = static_cast<int>(currentTicks) - static_cast<int>(lastEntry.predictedTicks);
          lastEntry.distErrorMm = lastEntry.timeErrorTicks * VELOCITY;
        }
      }

      SensorHistoryEntry newEntry{};
      newEntry.event = msg.sensorEvent;
      newEntry.ticks = currentTicks;
      newEntry.hasPrediction = false;
      newEntry.timeErrorTicks = 0;
      newEntry.distErrorMm = 0;

      int outDist = 0;
      marklin::TrackNode* nextSensor =
          marklin::getNextSensor(ttState, currentNode->edge[marklin::TrackDirection::Ahead].dest, outDist);
      if (nextSensor != nullptr && nextSensor->type == marklin::TrackNode::Type::Sensor) {
        newEntry.predictedId = nextSensor->id;
        outDist += currentNode->edge[marklin::TrackDirection::Ahead].dist;
        newEntry.predictedTicks = currentTicks + (static_cast<unsigned>(outDist) / VELOCITY);
        newEntry.hasPrediction = true;
      }

      sensorHistory.push(newEntry);
      sendSensorsToUI(uiTid, sensorHistory);
      break;
    }
    case TrainTrackMsgType::TimerTick: {
      currentTicks = msg.time.ticks;
      for (unsigned i = 1; i <= marklin::NUM_TRAINS; ++i) {
        marklin::TrainState& t = ttState.trains[i];
        if (t.state != marklin::TrainState::State::Reversing) {
          continue;
        }
        if (--t.reverseCountdownTicks == 0) {
          uint8_t trainId = static_cast<uint8_t>(i);
          auto reverseDir = t.forward ? marklin::TrainDirection::Backward : marklin::TrainDirection::Forward;
          sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainDirection(trainId, reverseDir));
          sendToDispatcher(dispatcherTid,
                           marklin::MMessage::setTrainSpeed(trainId, marklin::convertSpeedLevelToSpeed(t.speed)));
          t.forward = !t.forward;
          t.state = marklin::TrainState::State::Idle;
        }
      }
      break;
    }
    }
  }
}
} // namespace k4
