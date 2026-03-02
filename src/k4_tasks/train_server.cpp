#include "train_server.h"

#include "message.h"
#include "send_util.h"

#include "kernel/syscalls.h"
#include "marklin/marklin_message.h"
#include "marklin/marklin_train_track.h"
#include "util/debug.h"

#include <array>
#include <cstdint>

namespace k4 {

namespace {

constexpr uint16_t convSpeed(unsigned speed) { return static_cast<uint16_t>(speed > 0 ? 1 + (speed - 1) * 77 : 0); }

void sendToDispatcher(int dispatcherTid, const marklin::MMessage& mmsg) {
  notify(dispatcherTid, DispatcherMsg{.type = DispatcherMsgType::QueueCommand, .mmsg = mmsg});
}

struct TrainState {
  uint8_t speed;
  bool forward;
  enum class State : uint8_t { Idle, Reversing } state;
  bool touched;
  unsigned reverseCountdownTicks;
};

struct TrainServerState {
  std::array<TrainState, MAX_TRAINS + 1> trains{};
  int dispatcherTid = -1;
  int uiTid = -1;

  void init() {
    for (unsigned i = 0; i <= MAX_TRAINS; ++i) {
      auto& t = trains[i];
      t.speed = 0;
      t.forward = true;
      t.state = TrainState::State::Idle;
      t.touched = false;
      t.reverseCountdownTicks = 0;
    }
  }

  void initTrain(uint8_t trainNo) {
    auto& t = trains[trainNo];
    if (!t.touched) {
      t.touched = true;
      sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainFunctionState(trainNo, marklin::TrainFunction::F0, 1));
      sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainDirection(trainNo, marklin::TrainDirection::Forward));
    }
  }
};

} // namespace

void trainServerTask() {
  if (::RegisterAs(TRAIN_SERVER_NAME) < 0) {
    logError("train server: failed to register");
  }
  int dispatcherTid = ::WhoIs(DISPATCHER_SERVER_NAME);
  if (dispatcherTid < 0) {
    logError("train server: failed to find dispatcher");
  }
  int uiTid = ::WhoIs(UI_SERVER_NAME);
  if (uiTid < 0) {
    logError("train server: failed to find UI server");
  }

  TrainServerState state;
  state.dispatcherTid = dispatcherTid;
  state.uiTid = uiTid;
  state.init();

  TrainMsg msg;
  for (;;) {
    int senderTid;
    ::Receive(&senderTid, reinterpret_cast<char*>(&msg), sizeof(msg));
    ::Reply(senderTid, "", 0);

    switch (msg.type) {
    case TrainMsgType::SetSpeed: {
      uint8_t trainNo = msg.setSpeed.trainNo;
      uint8_t speed = msg.setSpeed.speed;
      if (speed > 14 || trainNo == 0 || trainNo > MAX_TRAINS) {
        break;
      }
      state.initTrain(trainNo);
      auto& t = state.trains[trainNo];
      t.speed = speed;
      if (t.state == TrainState::State::Idle) {
        // if reversing, will set speed after reverse countdown finishes
        sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainSpeed(trainNo, convSpeed(speed)));
      }
      notifyStatusToUI(uiTid, "Set train %u to speed %u.", trainNo, speed);
      break;
    }
    case TrainMsgType::Reverse: {
      uint8_t trainNo = msg.reverse.trainNo;
      if (trainNo == 0 || trainNo > MAX_TRAINS) {
        break;
      }
      auto& t = state.trains[trainNo];
      state.initTrain(trainNo);

      if (t.state == TrainState::State::Reversing) {
        // already reversing, cancel existing reverse command
        t.state = TrainState::State::Idle;
        notifyStatusToUI(uiTid, "Cancelled reversing train %u.", trainNo);
        sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainSpeed(trainNo, t.speed));
        break;
      }

      // stop the train
      sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainSpeed(trainNo, 0));

      // begin reverse countdown
      t.state = TrainState::State::Reversing;
      t.reverseCountdownTicks = 50 + static_cast<unsigned>(t.speed) * 25;

      notifyStatusToUI(uiTid, "Reversing train %u in %u ticks.", trainNo, t.reverseCountdownTicks);
      break;
    }
    case TrainMsgType::TimerTick: {
      for (unsigned i = 1; i <= MAX_TRAINS; ++i) {
        auto& t = state.trains[i];
        if (t.state != TrainState::State::Reversing) {
          continue;
        }
        if (--t.reverseCountdownTicks == 0) {
          uint8_t trainNo = static_cast<uint8_t>(i);
          auto reverseDir = t.forward ? marklin::TrainDirection::Backward : marklin::TrainDirection::Forward;
          sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainDirection(trainNo, reverseDir));
          sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainSpeed(trainNo, convSpeed(t.speed)));
          t.forward = !t.forward;
          t.state = TrainState::State::Idle;
        }
      }
      break;
    }
    }
  }
}

} // namespace k4
