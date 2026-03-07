#include "train_track_server_task.h"

#include "message.h"

#include "kernel/syscalls.h"
#include "marklin/marklin_message.h"
#include "marklin/marklin_pathfinding.h"
#include "marklin/marklin_train_track.h"
#include "send_util.h"
#include "util/debug.h"
#include "util/history.h"
#include "util/kit_algorithm.h"
#include "util/ring_buffer.h"

#include <array>
#include <cstdint>

namespace k4 {

namespace {

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
  if (marklin::TrainState& t = ttState.getTrainStateRef(trainId); !t.touched) {
    t.touched = true;
    sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainFunctionState(trainId, marklin::TrainFunction::F0, 1));
    sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainDirection(trainId, marklin::TrainDirection::Forward));
    // hardcoded start: between B9/B10 and A5/A6, facing towards A5, so last sensor is B9
    t.lastSensorNode = &ttState.getCurrentTrack()[marklin::sensorToId({'B', 9})];
  }
  ttState.theTrain = trainId;
}

} // namespace

bool isNodeInLoop(const marklin::TrackNode* node) {
  if (node == nullptr || node->type != marklin::TrackNode::Type::Sensor) {
    return false;
  }
  static constexpr uint32_t loopIds[] = {
      marklin::sensorToId({'D', 1}),  marklin::sensorToId({'D', 2}),  marklin::sensorToId({'E', 3}),
      marklin::sensorToId({'E', 4}),  marklin::sensorToId({'E', 5}),  marklin::sensorToId({'E', 6}),
      marklin::sensorToId({'D', 5}),  marklin::sensorToId({'D', 6}),  marklin::sensorToId({'E', 9}),
      marklin::sensorToId({'E', 10}), marklin::sensorToId({'E', 13}), marklin::sensorToId({'E', 14}),
      marklin::sensorToId({'D', 15}), marklin::sensorToId({'D', 16}), marklin::sensorToId({'B', 13}),
      marklin::sensorToId({'B', 14}),
  };
  for (uint32_t id : loopIds) {
    if (node->id == id)
      return true;
  }
  return false;
}

const marklin::TrackNode* findNodeByName(marklin::TrainTrackState& ttState, const char* name) {
  const auto& track = ttState.getCurrentTrack();
  for (size_t i = 0; i < marklin::NUM_TRACK; ++i) {
    if (track[i].name == nullptr)
      continue;
    const char* a = track[i].name;
    const char* b = name;
    while (*a && *b && *a == *b) {
      ++a;
      ++b;
    }
    if (*a == '\0' && *b == '\0')
      return &track[i];
  }
  return nullptr;
}

template <typename GoalFn>
const marklin::TrackNode* bfsAndSetSwitches(const marklin::TrackNode* startNode, GoalFn goalFn,
                                            marklin::TrainTrackState& ttState, int dispatcherTid, int uiTid) {
  bool visited[marklin::NUM_TRACK]{};
  const marklin::TrackEdge* pred[marklin::NUM_TRACK]{};
  RingBuffer<const marklin::TrackNode*, 256> bfsQueue;

  visited[startNode->id] = true;
  bfsQueue.push(startNode);

  const marklin::TrackNode* targetNode = nullptr;

  while (!bfsQueue.empty()) {
    const marklin::TrackNode* node = bfsQueue.pop();
    if (goalFn(node)) {
      targetNode = node;
      break;
    }

    auto tryEdge = [&](const marklin::TrackEdge& edge) {
      const marklin::TrackNode* dest = edge.dest;
      if (dest != nullptr && dest->id < marklin::NUM_TRACK && !visited[dest->id]) {
        visited[dest->id] = true;
        pred[dest->id] = &edge;
        bfsQueue.push(dest);
      }
    };

    switch (node->type) {
    case marklin::TrackNode::Type::Branch:
      tryEdge(node->edge[marklin::TrackDirection::Straight]);
      tryEdge(node->edge[marklin::TrackDirection::Curved]);
      break;
    case marklin::TrackNode::Type::Sensor:
    case marklin::TrackNode::Type::Merge:
    case marklin::TrackNode::Type::Enter:
      tryEdge(node->edge[marklin::TrackDirection::Ahead]);
      break;
    default:
      break;
    }
  }

  if (targetNode == nullptr) {
    return nullptr;
  }

  const marklin::TrackNode* cur = targetNode;
  while (cur != startNode && pred[cur->id] != nullptr) {
    const marklin::TrackEdge* edge = pred[cur->id];
    const marklin::TrackNode* src = edge->src;
    if (src->type == marklin::TrackNode::Type::Branch) {
      marklin::SwitchState sw;
      if (edge == &src->edge[marklin::TrackDirection::Straight]) {
        sw = marklin::SwitchState::Straight;
      } else {
        sw = marklin::SwitchState::Curved;
      }
      uint8_t id = static_cast<uint8_t>(src->id);
      ttState.setSwitchState(id, sw);
      sendToDispatcher(dispatcherTid, marklin::MMessage::setSwitchState(id, sw));
      sendSwitchToUI(uiTid, id, sw);
    }
    cur = src;
  }

  return targetNode;
}

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

  unsigned lastTrainUIRefreshTicks = 0;

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
      marklin::TrainState& t = ttState.getTrainStateRef(trainId);
      t.speedLevel = speed;
      if (t.motionState == marklin::TrainState::MotionState::Idle) {
        // If reversing, will set speed after reverse countdown finishes.
        sendToDispatcher(dispatcherTid,
                         marklin::MMessage::setTrainSpeed(trainId, marklin::convertSpeedLevelToCANSpeed(speed)));
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
      marklin::TrainState& t = ttState.getTrainStateRef(trainId);
      if (t.motionState == marklin::TrainState::MotionState::Reversing) {
        // already reversing, cancel existing reverse command
        t.motionState = marklin::TrainState::MotionState::Idle;
        notifyStatusToUI(uiTid, "Cancelled reversing train %u.", trainId);
        sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainSpeed(trainId, t.speedLevel));
        break;
      }

      // stop the train
      sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainSpeed(trainId, 0));

      // begin reverse countdown
      t.motionState = marklin::TrainState::MotionState::Reversing;
      t.reverseCountdownTicks = 50 + static_cast<unsigned>(t.speedLevel) * 25;

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
    case TrainTrackMsgType::SetTrackCmd: {
      ttState.setCurrentTrack(msg.setTrackCmd.trackId);
      break;
    }
    case TrainTrackMsgType::SensorEvent: {
      if (msg.sensorEvent.state != marklin::SensorState::Occupied) {
        break;
      }

      size_t sensorIndex = marklin::getSensorIndex(msg.sensorEvent.id);
      marklin::TrackNode* currentNode = &ttState.getCurrentTrack()[sensorIndex];

      auto velocity = ttState.getTrainStateRef(ttState.theTrain).estimatedSpeed;
      if (!sensorHistory.empty()) {
        auto& lastEntry = sensorHistory.back();
        if (lastEntry.hasPrediction && lastEntry.predictedId == msg.sensorEvent.id) {
          lastEntry.timeErrorTicks = static_cast<int>(currentTicks) - static_cast<int>(lastEntry.predictedTicks);
          lastEntry.distErrorMm = lastEntry.timeErrorTicks * static_cast<int>(velocity) / 1000;
        }
      }

      SensorHistoryEntry newEntry{};
      newEntry.event = msg.sensorEvent;
      newEntry.ticks = currentTicks;
      newEntry.hasPrediction = false;
      newEntry.timeErrorTicks = 0;
      newEntry.distErrorMm = 0;
      {
        unsigned outDist = 0;
        const marklin::TrackNode* nextSensor =
            marklin::getNextSensor(ttState, currentNode->edge[marklin::TrackDirection::Ahead].dest, outDist);
        if (ttState.theTrain != 0) {
          if (nextSensor != nullptr && nextSensor->type == marklin::TrackNode::Type::Sensor) {
            newEntry.predictedId = nextSensor->id;
            outDist += currentNode->edge[marklin::TrackDirection::Ahead].dist;
            newEntry.predictedTicks = currentTicks + (static_cast<unsigned>(outDist) * 1000 / velocity);
            newEntry.hasPrediction = true;
          }
        }
      }

      sensorHistory.push(newEntry);
      sendSensorsToUI(uiTid, sensorHistory);

      // Update the train state.

      // TOOD: remove me
      currentNode->trainOwnerId = ttState.theTrain;
      if (currentNode->trainOwnerId == 0) {
        break;
      }
      marklin::TrainState& trainState = ttState.getTrainStateRef(currentNode->trainOwnerId);

      // Update the train estimated speed.
      uint32_t dT = kit::max(1u, currentTicks - trainState.lastSensorTicks);
      uint32_t dS = [&] -> uint32_t {
        unsigned outDist = 0;
        const marklin::TrackNode* ptr = trainState.lastSensorNode;
        static constexpr unsigned MAX_HOPS = 20;
        for (unsigned hops = 0; ptr && hops < MAX_HOPS; ++hops) {
          outDist += ptr->edge[marklin::TrackDirection::Ahead].dist;
          ptr = marklin::getNextSensor(ttState, ptr->edge[marklin::TrackDirection::Ahead].dest, outDist);
          if (ptr == currentNode) {
            return outDist * 1000; // mm -> um
          }
        }
        notifyStatusToUI(uiTid, "Missing edge between two adjacent track nodes");
        return 0;
      }();
      if (dS != 0) {
        uint32_t v = dS / dT;
        trainState.estimatedSpeed = (trainState.estimatedSpeed * (marklin::TrainState::EWMA_DENOMINATOR - 1) + v) /
                                    marklin::TrainState::EWMA_DENOMINATOR;
      }

      // Update the train last triggered sensor node.
      trainState.lastSensorNode = currentNode;
      trainState.lastSensorTicks = currentTicks;
      trainState.positionOffset = 0;
      break;
    }
    case TrainTrackMsgType::LoopCmd: {
      uint8_t trainId = msg.loopCmd.trainId;
      if (trainId == 0 || trainId > marklin::NUM_TRAINS) {
        notifyStatusToUI(uiTid, "Invalid train id for loop.");
        break;
      }
      initTrain(dispatcherTid, ttState, trainId);
      marklin::TrainState& t = ttState.getTrainStateRef(trainId);
      const marklin::TrackNode* startNode = t.lastSensorNode;
      if (startNode == nullptr) {
        notifyStatusToUI(uiTid, "Train %u has no known location.", trainId);
        break;
      }

      const marklin::TrackNode* targetNode = bfsAndSetSwitches(startNode, isNodeInLoop, ttState, dispatcherTid, uiTid);
      if (targetNode == nullptr) {
        notifyStatusToUI(uiTid, "No path found to the loop from %s.", startNode->name);
      } else {
        notifyStatusToUI(uiTid, "Path set from %s to loop node %s.", startNode->name, targetNode->name);
      }
      break;
    }
    case TrainTrackMsgType::GotoCmd: {
      uint8_t trainId = msg.gotoCmd.trainId;
      if (trainId == 0 || trainId > marklin::NUM_TRAINS) {
        notifyStatusToUI(uiTid, "Invalid train id for goto.");
        break;
      }
      initTrain(dispatcherTid, ttState, trainId);
      marklin::TrainState& t = ttState.getTrainStateRef(trainId);
      const marklin::TrackNode* startNode = t.lastSensorNode;
      if (startNode == nullptr) {
        notifyStatusToUI(uiTid, "Train %u has no known location.", trainId);
        break;
      }

      const marklin::TrackNode* destNode = findNodeByName(ttState, msg.gotoCmd.location);
      if (destNode == nullptr) {
        notifyStatusToUI(uiTid, "Unknown track node '%s'.", msg.gotoCmd.location);
        break;
      }

      uint32_t destId = destNode->id;
      auto isGoal = [destId](const marklin::TrackNode* node) { return node->id == destId; };
      const marklin::TrackNode* targetNode = bfsAndSetSwitches(startNode, isGoal, ttState, dispatcherTid, uiTid);
      if (targetNode == nullptr) {
        notifyStatusToUI(uiTid, "No path found from %s to %s.", startNode->name, destNode->name);
      } else {
        notifyStatusToUI(uiTid, "Path set from %s to %s.", startNode->name, targetNode->name);
      }
      break;
    }
    case TrainTrackMsgType::TimerTick: {
      currentTicks = msg.time.ticks;
      for (uint8_t trainId = 1; trainId <= marklin::NUM_TRAINS; ++trainId) {
        marklin::TrainState& t = ttState.getTrainStateRef(trainId);

        // Reverse direction
        if (t.motionState == marklin::TrainState::MotionState::Reversing && --t.reverseCountdownTicks == 0) {
          auto reverseDir = t.forward ? marklin::TrainDirection::Backward : marklin::TrainDirection::Forward;
          sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainDirection(trainId, reverseDir));
          sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainSpeed(
                                              trainId, marklin::convertSpeedLevelToCANSpeed(t.speedLevel)));
          t.forward = !t.forward;
          t.motionState = marklin::TrainState::MotionState::Idle;
        }

        // Update the train position
        if (t.lastSensorNode) {
          t.positionOffset += t.estimatedSpeed;
        }
      }

      // Send state of "theTrain" to UI
      if (ttState.theTrain != 0 && currentTicks - lastTrainUIRefreshTicks >= 10) {
        lastTrainUIRefreshTicks = currentTicks;
        marklin::TrainState& t = ttState.getTrainStateRef(ttState.theTrain);
        notify(uiTid, UIMsg{.type = UIMsgType::TrainState, .trainState{t}});
      }
      break;
    }
    }
  }
}
} // namespace k4
