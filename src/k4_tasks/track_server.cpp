#include "track_server.h"

#include "message.h"
#include "send_util.h"

#include "kernel/syscalls.h"
#include "marklin/marklin_message.h"
#include "marklin/marklin_train_track.h"
#include "util/debug.h"
#include "util/history.h"

#include <array>
#include <cstdint>

namespace k4 {

namespace {

constexpr size_t getSwitchIndex(uint8_t id) {
  if (id >= 1 && id <= 18) {
    return id - 1;
  }
  if (id >= 153 && id <= 156) {
    return 18 + id - 153;
  }
  return NUM_SWITCHES; // invalid
}

void sendSwitchToUI(int uiTid, uint8_t switchNo, marklin::SwitchState state) {
  notify(uiTid, UIMsg{.type = UIMsgType::UpdateSwitch, .switchUpdate{.switchNo = switchNo, .state = state}});
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

constexpr int VELOCITY = 3; // mm per tick

struct TrackState {
  std::array<marklin::SwitchState, NUM_SWITCHES> switches{};
  History<SensorHistoryEntry, SENSOR_HISTORY_SIZE> sensorHistory;
  marklin::TrackSet track{};
  int dispatcherTid = -1;
  int uiTid = -1;
  unsigned currentTicks = 0;

  void initSwitches() {
    for (auto& sw : switches) {
      sw = marklin::SwitchState::Straight;
    }
  }

  void initTrack() { marklin::initTrackA(track); }

  void sendAllSwitchesToUI() const {
    auto notify = [this](uint8_t id) {
      size_t idx = getSwitchIndex(id);
      if (idx < NUM_SWITCHES) {
        sendSwitchToUI(uiTid, id, switches[idx]);
      }
    };
    for (uint8_t i = 1; i <= 18; ++i) {
      notify(i);
    }
    for (uint8_t i = 153; i <= 156; ++i) {
      notify(i);
    }
  }

  marklin::SwitchState getSwitchState(uint8_t id) const {
    size_t idx = getSwitchIndex(id);
    if (idx < NUM_SWITCHES) {
      return switches[idx];
    }
    return marklin::SwitchState::Curved;
  }

  void setSwitchState(uint8_t id, marklin::SwitchState switchState) {
    size_t idx = getSwitchIndex(id);
    if (idx < NUM_SWITCHES) {
      switches[idx] = switchState;
      notify(dispatcherTid, DispatcherMsg{.type = DispatcherMsgType::QueueCommand,
                                          .mmsg = marklin::MMessage::setSwitchState(id, switchState, true)});
    }
  }

  marklin::TrackNode* getNextSensor(marklin::TrackNode* current, int& outDist) {
    outDist = 0;
    marklin::TrackNode* node = current;
    while (node != nullptr && node->type != marklin::TrackNode::Type::Sensor) {
      if (node->type == marklin::TrackNode::Type::Exit) {
        return nullptr;
      }
      size_t dir = marklin::TrackDirection::Ahead;
      if (node->type == marklin::TrackNode::Type::Branch) {
        auto sw = getSwitchState(static_cast<uint8_t>(node->num));
        dir =
            sw == marklin::SwitchState::Straight ? marklin::TrackDirection::Straight : marklin::TrackDirection::Curved;
      }
      outDist += node->edge[dir].dist;
      node = node->edge[dir].dest;
    }
    return node;
  }
};

} // namespace

// A server that manages the local track state, and propagate the changes to the remote by sending CAN message.
void trackServerTask() {
  if (::RegisterAs(TRACK_SERVER_NAME) < 0) {
    logError("track server: failed to register");
  }
  int dispatcherTid = ::WhoIs(DISPATCHER_SERVER_NAME);
  if (dispatcherTid < 0) {
    logError("track server: failed to find dispatcher");
  }
  int uiTid = ::WhoIs(UI_SERVER_NAME);
  if (uiTid < 0) {
    logError("track server: failed to find UI server");
  }

  TrackState state;
  state.dispatcherTid = dispatcherTid;
  state.uiTid = uiTid;
  state.initSwitches();
  state.initTrack();
  state.sendAllSwitchesToUI();

  TrackMsg msg;
  for (;;) {
    int senderTid;
    ::Receive(&senderTid, reinterpret_cast<char*>(&msg), sizeof(msg));
    ::Reply(senderTid, "", 0);

    switch (msg.type) {
    case TrackMsgType::SetSwitch: {
      auto sw = msg.setSwitch.straight ? marklin::SwitchState::Straight : marklin::SwitchState::Curved;
      state.setSwitchState(msg.setSwitch.switchNo, sw);
      sendSwitchToUI(uiTid, msg.setSwitch.switchNo, sw);
      break;
    }
    case TrackMsgType::SensorEvent: {
      if (!msg.sensorEvent.newOccupied) {
        break;
      }

      size_t sensorIdx = static_cast<size_t>((msg.sensorEvent.bank - 'A') * 16 + msg.sensorEvent.number - 1);
      marklin::TrackNode* currentNode = &state.track[sensorIdx];

      if (!state.sensorHistory.empty()) {
        auto& lastEntry = state.sensorHistory.back();
        if (lastEntry.hasPrediction && lastEntry.predictedBank == msg.sensorEvent.bank &&
            lastEntry.predictedNumber == msg.sensorEvent.number) {
          lastEntry.timeErrorTicks = static_cast<int>(state.currentTicks) - static_cast<int>(lastEntry.predictedTicks);
          lastEntry.distErrorMm = lastEntry.timeErrorTicks * VELOCITY;
        }
      }

      SensorHistoryEntry newEntry{};
      newEntry.event = msg.sensorEvent;
      newEntry.ticks = state.currentTicks;
      newEntry.hasPrediction = false;
      newEntry.timeErrorTicks = 0;
      newEntry.distErrorMm = 0;

      int outDist = 0;
      marklin::TrackNode* nextSensor =
          state.getNextSensor(currentNode->edge[marklin::TrackDirection::Ahead].dest, outDist);
      if (nextSensor != nullptr && nextSensor->type == marklin::TrackNode::Type::Sensor) {
        int nextSensorNum = nextSensor->num;
        newEntry.predictedBank = 'A' + static_cast<char>(nextSensorNum / 16);
        newEntry.predictedNumber = static_cast<uint8_t>(nextSensorNum % 16 + 1);
        outDist += currentNode->edge[marklin::TrackDirection::Ahead].dist;
        newEntry.predictedTicks = state.currentTicks + (static_cast<unsigned>(outDist) / VELOCITY);
        newEntry.hasPrediction = true;
      }

      state.sensorHistory.push(newEntry);
      sendSensorsToUI(uiTid, state.sensorHistory);
      break;
    }
    case TrackMsgType::TimerTick: {
      state.currentTicks = msg.time.ticks;
      break;
    }
    }
  }
}

} // namespace k4
