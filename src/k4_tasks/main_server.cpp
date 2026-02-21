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

#include <cstdint>
#include <ranges>

namespace k4 {

namespace {

constexpr unsigned ROW_TIME = 1;
constexpr unsigned ROW_SWITCHES = 3;
constexpr unsigned ROW_SENSORS = 3;
constexpr unsigned COL_SENSORS = 30;
constexpr unsigned ROW_CMD_HISTORY = 20;
constexpr unsigned ROW_STATUS = 38;
constexpr unsigned ROW_STATUS2 = 39;
constexpr unsigned ROW_PROMPT = 40;

constexpr unsigned ACK_TIMEOUT_TICKS = 10; // 1 second

} // namespace

bool ServerState::lastCommandAcked() const {
  if (cmdHistory.empty()) {
    return true;
  }
  const auto& last = cmdHistory.back();
  return last.ackAfter != NOT_ACKED || currentTicks - last.sentTicks >= ACK_TIMEOUT_TICKS;
}

void ServerState::sendCAN(const marklin::MMessage& msg) {
  if (canSendBuffer.empty() && lastCommandAcked()) {
    ::TransmitCAN(canServerTid, msg);
    cmdHistory.push({msg, currentTicks, NOT_ACKED});
  } else {
    if (!canSendBuffer.full()) {
      canSendBuffer.push(msg);
    }
  }
}

bool ServerState::tryFlushCanBuffer() {
  bool flushed = false;
  while (!canSendBuffer.empty() && lastCommandAcked()) {
    auto msg = canSendBuffer.pop();
    ::TransmitCAN(canServerTid, msg);
    cmdHistory.push({msg, currentTicks, NOT_ACKED});
    flushed = true;
  }
  return flushed;
}

bool ServerState::processCanResponse(const marklin::MMessage& response) {
  const auto match = [&](const CmdHistoryEntry& entry) {
    if (entry.ackAfter != NOT_ACKED || entry.msg.command != response.command || entry.msg.dlc != response.dlc) {
      return false;
    }
    for (unsigned i = 0; i < response.dlc; ++i) {
      if (entry.msg.data[i] != response.data[i]) {
        return false;
      }
    }
    return true;
  };
  for (auto& entry : cmdHistory | std::views::reverse) {
    if (match(entry)) {
      entry.ackAfter = currentTicks - entry.sentTicks;
      return true;
    }
  }
  return false;
}

void ServerState::initTrain(uint8_t trainNo) {
  auto& t = trains[trainNo];
  if (!t.touched) {
    t.touched = true;
    sendCAN(marklin::MMessage::setTrainFunctionState(trainNo, marklin::TrainFunction::F0, 1));
    sendCAN(marklin::MMessage::setTrainDirection(trainNo, marklin::TrainDirection::Forward));
  }
}

constexpr size_t getSwitchIndex(unsigned id) {
  if (id >= 1 && id <= 18) {
    return id - 1;
  }
  if (id >= 153 && id <= 156) {
    return 18 + id - 153;
  }
  return NUM_SWITCHES; // invalid
}

marklin::SwitchState ServerState::getSwitchState(unsigned id) const {
  size_t idx = getSwitchIndex(id);
  if (idx < NUM_SWITCHES) {
    return switches[idx];
  }
  return marklin::SwitchState::Curved;
}

void ServerState::setSwitchState(unsigned id, marklin::SwitchState switchState) {
  size_t idx = getSwitchIndex(id);
  if (idx < NUM_SWITCHES) {
    switches[idx] = switchState;
    sendCAN(marklin::MMessage::setSwitchState(static_cast<uint8_t>(id), switchState, true));
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
      .switches{},
      .sensorHistory{},
      .cmdHistory{},
      .canSendBuffer{},
      .status{.msg = "Ready.", .dirty = true},
      .canServerTid = canServerTid,
      .currentTicks = 0,
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

  state.sendCAN(marklin::MMessage::systemGoAll());
  for (auto& sw : state.switches) {
    sw = marklin::SwitchState::Straight;
  }
  // Send switch init commands
  for (uint8_t id = 1; id <= 18; ++id) {
    state.sendCAN(marklin::MMessage::setSwitchState(id, marklin::SwitchState::Straight, true));
  }
  for (uint8_t id = 153; id <= 156; ++id) {
    state.sendCAN(marklin::MMessage::setSwitchState(id, marklin::SwitchState::Straight, true));
  }

  cmd::CommandBuffer cmdBuf;

  bool promptDirty = true;
  bool switchesDirty = true;
  bool sensorsDirty = true;
  bool cmdHistoryDirty = true;

  const auto renderPrompt = [&]() {
    if (!promptDirty) {
      return;
    }
    promptDirty = false;
    console.moveCursor(ROW_PROMPT, 1);
    console.printf("> %s", cmdBuf);
    console.clearToEol();
  };

  const auto renderStatus = [&]() {
    if (!state.status.dirty) {
      return;
    }
    console.moveCursor(ROW_STATUS, 1);
    console.puts(state.status.msg);
    console.clearToEol();
    state.status.dirty = false;
  };

  const auto renderSwitch = [&](unsigned id, unsigned r, unsigned c) {
    console.moveCursor(ROW_SWITCHES + 1 + r, 1 + c * 10);
    console.printf("%03u: %c", id, state.getSwitchState(id) == marklin::SwitchState::Straight ? 'S' : 'C');
  };

  const auto renderSwitches = [&]() {
    if (!switchesDirty) {
      return;
    }
    switchesDirty = false;
    console.moveCursor(ROW_SWITCHES, 1);
    console.puts("Switches:");
    for (unsigned i = 1; i <= 11; ++i) {
      renderSwitch(i, i - 1, 0);
    }
    for (unsigned i = 12; i <= 18; ++i) {
      renderSwitch(i, i - 12, 1);
    }
    for (unsigned i = 153; i <= 156; ++i) {
      renderSwitch(i, 7 + i - 153, 1);
    }
  };

  const auto renderSensors = [&]() {
    if (!sensorsDirty) {
      return;
    }
    sensorsDirty = false;
    console.moveCursor(ROW_SENSORS, COL_SENSORS);
    console.puts("Sensors:");
    unsigned row = 0;
    for (const auto& entry : state.sensorHistory) {
      console.moveCursor(ROW_SENSORS + 1 + row, COL_SENSORS);
      console.putTimestamp(entry.ticks);
      console.printf(" %c%u: %s -> %s", entry.event.bank, entry.event.number, entry.event.oldOccupied ? "Occ" : "Free",
                     entry.event.newOccupied ? "Occ" : "Free");
      console.clearToEol();
      ++row;
    }
  };

  const auto renderCmdHistory = [&]() {
    if (!cmdHistoryDirty)
      return;
    cmdHistoryDirty = false;
    console.moveCursor(ROW_CMD_HISTORY, 1);
    console.puts("Command history:");
    unsigned row = 0;
    for (const auto& entry : state.cmdHistory) {
      console.moveCursor(ROW_CMD_HISTORY + 1 + row, 1);
      console.putTimestamp(entry.sentTicks);
      console.putc(' ');
      console.putByte(static_cast<uint8_t>(entry.msg.command));
      console.putc(' ');
      for (unsigned d = 0; d < 8; ++d) {
        if (d < entry.msg.dlc) {
          console.putByte(entry.msg.data[d]);
        } else {
          console.puts("..");
        }
      }
      if (entry.ackAfter != NOT_ACKED) {
        console.printf(" ack %u ticks", entry.ackAfter);
      }
      console.clearToEol();
      ++row;
    }
  };

  Message msg;
  for (;;) {

    renderPrompt();
    renderStatus();
    renderSwitches();
    renderSensors();
    renderCmdHistory();

    int senderTid;
    ::Receive(&senderTid, reinterpret_cast<char*>(&msg), sizeof(msg));
    switch (msg.type) {
    case MessageType::Timer: {
      state.currentTicks = msg.timerUpdate.deciseconds;
      console.moveCursor(ROW_TIME, 1);
      console.putTimestamp(msg.timerUpdate.deciseconds);
      if (state.tryFlushCanBuffer()) {
        cmdHistoryDirty = true;
      }
      ::Reply(senderTid, "", 0);
      break;
    }
    case MessageType::KeyPress: {
      char c = msg.keyPress.c;
      if (c == '\r' || c == '\n') {
        if (const cmd::Command* cmd = cmdBuf.parse()) {
          cmd->process(state);
          switchesDirty = true;
          cmdHistoryDirty = true;
        }
      } else if (c == '\b' || c == 127) {
        cmdBuf.backspace();
      } else if (cmd::CommandBuffer::isCmdChar(c)) {
        cmdBuf.push(c);
      }
      promptDirty = true;
      ::Reply(senderTid, "", 0);
      break;
    }
    case MessageType::SensorEvent: {
      state.sensorHistory.push({msg.sensorEvent, state.currentTicks});
      sensorsDirty = true;
      ::Reply(senderTid, "", 0);
      break;
    }
    case MessageType::CanResponse: {
      if (state.processCanResponse(msg.canResponse.msg)) {
        state.tryFlushCanBuffer();
        cmdHistoryDirty = true;
      }
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
      cmdHistoryDirty = true;
      ::Reply(senderTid, "", 0);
      break;
    }
    default:
      logError("Received unknown message type");
    }
  }
}

} // namespace k4
