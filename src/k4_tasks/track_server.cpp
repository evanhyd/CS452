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

struct TrackState {
  std::array<marklin::SwitchState, NUM_SWITCHES> switches{};
  History<SensorHistoryEntry, SENSOR_HISTORY_SIZE> sensorHistory;
  int dispatcherTid = -1;
  int uiTid = -1;
  unsigned currentTicks = 0;

  void initSwitches() {
    for (auto& sw : switches) {
      sw = marklin::SwitchState::Straight;
    }
  }

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
      state.sensorHistory.push({msg.sensorEvent, state.currentTicks});
      sendSensorsToUI(uiTid, state.sensorHistory);
      break;
    }
    case TrackMsgType::TimerTick: {
      state.currentTicks = msg.time.deciseconds;
      break;
    }
    }
  }
}

} // namespace k4
