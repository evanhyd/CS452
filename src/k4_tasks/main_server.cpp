#include "main_server.h"

#include "command.h"
#include "k4_tasks/helper_tasks.h"
#include "message.h"

#include "kernel/syscalls.h"
#include "marklin/marklin_message.h"
#include "marklin/marklin_train_track.h"
#include "server_tasks/can_server.h"
#include "server_tasks/clock_server.h"
#include "server_tasks/io_server.h"
#include "util/console.h"
#include "util/ctfmt.h"
#include "util/debug.h"

namespace k4 {

namespace {

constexpr unsigned ROW_TIME = 1;
constexpr unsigned ROW_SWITCHES = 3;
constexpr unsigned ROW_SENSORS = 3;
constexpr unsigned COL_SENSORS = 30;
constexpr unsigned ROW_CMD_HISTORY = 20;
constexpr unsigned ROW_STATUS = 39;
constexpr unsigned ROW_PROMPT = 40;

} // namespace

void ServerState::sendCAN(const marklin::MMessage& msg) { ::TransmitCAN(canServerTid, msg); }

void ServerState::initTrain(uint8_t trainNo) {
  // lazily "init" trains
  auto& t = trains[trainNo];
  if (!t.touched) {
    t.touched = true;
    // light on
    sendCAN(marklin::MMessage::setTrainFunctionState(trainNo, marklin::TrainFunction::F0, 1));
    // forward
    sendCAN(marklin::MMessage::setTrainDirection(trainNo, marklin::TrainDirection::Forward));
  }
}

void mainServerTask() {
  if (::RegisterAs(MAIN_SERVER_NAME) < 0) {
    logError("failed to register itself to name server");
  }
  int ioServerTid = ::WhoIs(io_server::IO_SERVER_NAME);
  if (ioServerTid < 0) {
    logError("failed to find IO server");
  }
  int canServerTid = ::WhoIs(can_server::CAN_SERVER_NAME);
  if (canServerTid < 0) {
    logError("failed to find CAN server");
  }
  int clockServerTid = ::WhoIs(clock_server::CLOCK_SERVER_NAME);
  if (clockServerTid < 0) {
    logError("failed to find clock server");
  }

  ServerState state{
      .trains{},
      .toReverse{},
      .status{.msg = "Ready.", .dirty = true},
      .canServerTid = canServerTid,
  };

  for (unsigned i = 0; i <= MAX_TRAINS; ++i) {
    auto& train = state.trains[i];
    train.speed = 0;
    train.forward = true;
    train.state = TrainState::State::Idle;
    train.touched = false;
  }

  Console console{ioServerTid};
  console.clearScreen();
  console.hideCursor();

  cmd::CommandBuffer cmdBuf;
  const auto renderPrompt = [&]() {
    console.moveCursor(ROW_PROMPT, 1);
    console.printf("> %s", cmdBuf);
    console.clearToEol();
  };
  renderPrompt();

  const auto renderStatus = [&]() {
    if (!state.status.dirty) {
      return;
    }
    console.moveCursor(ROW_STATUS, 1);
    console.puts(state.status.msg);
    console.clearToEol();
    state.status.dirty = false;
  };
  renderStatus();

  Message msg;
  for (;;) {
    int senderTid;
    ::Receive(&senderTid, reinterpret_cast<char*>(&msg), sizeof(msg));
    switch (msg.type) {
    case MessageType::Timer: {
      console.moveCursor(1, 1);
      unsigned mins = msg.timerUpdate.deciseconds / 600;
      unsigned secs = msg.timerUpdate.deciseconds / 10 % 60;
      unsigned tenths = msg.timerUpdate.deciseconds % 10;
      console.printf("%02u:%02u.%u", mins, secs, tenths);
      ::Reply(senderTid, "", 0);
      break;
    }
    case MessageType::KeyPress: {
      char c = msg.keyPress.c;
      if (c == '\r' || c == '\n') {
        if (const cmd::Command* cmd = cmdBuf.parse()) {
          cmd->process(state);
          renderStatus();
        }
      } else if (c == '\b' || c == 127) {
        cmdBuf.backspace();
      } else if (cmd::CommandBuffer::isCmdChar(c)) {
        cmdBuf.push(c);
      }
      renderPrompt();
      ::Reply(senderTid, "", 0);
      break;
    }
    case MessageType::SensorEvent: {
      ::Reply(senderTid, "", 0);
      break;
    }
    case MessageType::CanResponse: {
      ::Reply(senderTid, "", 0);
      break;
    }
    case MessageType::StartReverse: {
      auto trainNo = state.toReverse.pop();
      auto& train = state.trains[trainNo];
      train.state = TrainState::State::Reversing;
      ReverseArgs args{
          .clockServerTid = clockServerTid,
          .trainNo = trainNo,
          .oldSpeed = train.speed,
      };
      ::Reply(senderTid, reinterpret_cast<const char*>(&args), sizeof(args));
      break;
    }
    case MessageType::EndReverse: {
      auto trainNo = msg.endReverse.trainNo;
      auto& train = state.trains[trainNo];
      auto reverseDir = train.forward ? marklin::TrainDirection::Backward : marklin::TrainDirection::Forward;
      state.sendCAN(marklin::MMessage::setTrainDirection(trainNo, reverseDir));
      state.sendCAN(marklin::MMessage::setTrainSpeed(trainNo, cmd::convSpeed(state.trains[trainNo].speed)));
      train.forward = !train.forward;
      train.state = TrainState::State::Idle;
      ::Reply(senderTid, "", 0);
      break;
    }
    default:
      logError("Received unknown message type");
    }
  }
}

} // namespace k4
