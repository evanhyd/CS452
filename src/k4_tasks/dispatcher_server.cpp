#include "dispatcher_server.h"

#include "k4_tasks/send_util.h"
#include "message.h"

#include "kernel/syscalls.h"
#include "marklin/marklin_message.h"
#include "server_tasks/can_server.h"
#include "util/ctfmt.h"
#include "util/debug.h"
#include "util/history.h"
#include "util/ring_buffer.h"

#include <cstdint>
#include <ranges>

namespace k4 {

namespace {

constexpr unsigned ACK_TIMEOUT_TICKS = 10; // 1 second

struct DispatcherState {
  History<CmdHistoryEntry, CMD_HISTORY_SIZE> cmdHistory;
  RingBuffer<marklin::MMessage, 256> canSendBuffer;
  unsigned currentTicks = 0;
  int canServerTid = -1;

  bool lastCommandAcked() const {
    if (cmdHistory.empty()) {
      return true;
    }
    const auto& last = cmdHistory.back();
    return last.ackAfter != NOT_ACKED || currentTicks - last.sentTicks >= ACK_TIMEOUT_TICKS;
  }

  void sendCAN(const marklin::MMessage& msg) {
    if (canSendBuffer.empty() && lastCommandAcked()) {
      ::TransmitCAN(canServerTid, msg);
      cmdHistory.push({msg, currentTicks, NOT_ACKED});
    } else {
      if (!canSendBuffer.full()) {
        canSendBuffer.push(msg);
      }
    }
  }

  bool tryFlushCanBuffer() {
    bool flushed = false;
    while (!canSendBuffer.empty() && lastCommandAcked()) {
      auto msg = canSendBuffer.pop();
      ::TransmitCAN(canServerTid, msg);
      cmdHistory.push({msg, currentTicks, NOT_ACKED});
      flushed = true;
    }
    return flushed;
  }

  bool processCanResponse(const marklin::MMessage& response) {
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

  void notifyCmdHistoryToUI(int uiServerTid) {
    UIMsg uiMsg;
    uiMsg.type = UIMsgType::RedrawCmdHistory;
    unsigned idx = 0;
    for (const auto& e : cmdHistory) {
      if (idx >= CMD_HISTORY_SIZE)
        break;
      uiMsg.cmdHistory.entries[idx++] = e;
    }
    uiMsg.cmdHistory.count = idx;
    notify(uiServerTid, uiMsg);
  }
};

} // namespace

void dispatcherServerTask() {
  if (::RegisterAs(DISPATCHER_SERVER_NAME) < 0) {
    logError("dispatcher: failed to register");
  }
  int canServerTid = ::WhoIs(can_server::CAN_SERVER_NAME);
  if (canServerTid < 0) {
    logError("dispatcher: failed to find CAN server");
  }
  int uiServerTid = ::WhoIs(UI_SERVER_NAME);
  if (uiServerTid < 0) {
    logError("dispatcher: failed to find UI server");
  }

  DispatcherState state;
  state.canServerTid = canServerTid;

  notify(uiServerTid, UIMsg{.type = UIMsgType::ClearScreen, .empty{}});
  notifyStatusToUI(uiServerTid, "Ready.");

  state.sendCAN(marklin::MMessage::systemGoAll());
  for (uint8_t id = 1; id <= 18; ++id) {
    state.sendCAN(marklin::MMessage::setSwitchState(id, marklin::SwitchState::Straight, true));
  }
  for (uint8_t id = 153; id <= 156; ++id) {
    state.sendCAN(marklin::MMessage::setSwitchState(id, marklin::SwitchState::Straight, true));
  }
  state.notifyCmdHistoryToUI(uiServerTid);

  DispatcherMsg msg;
  for (;;) {
    int senderTid;
    ::Receive(&senderTid, reinterpret_cast<char*>(&msg), sizeof(msg));
    ::Reply(senderTid, "", 0);

    switch (msg.type) {
    case DispatcherMsgType::QueueCommand: {
      state.sendCAN(msg.mmsg);
      state.notifyCmdHistoryToUI(uiServerTid);
      break;
    }
    case DispatcherMsgType::CanResponse: {
      if (state.processCanResponse(msg.mmsg)) {
        state.tryFlushCanBuffer();
      }
      state.notifyCmdHistoryToUI(uiServerTid);
      break;
    }
    case DispatcherMsgType::TimerTick: {
      state.currentTicks = msg.time.deciseconds;
      if (state.tryFlushCanBuffer()) {
        state.notifyCmdHistoryToUI(uiServerTid);
      }
      break;
    }
    }
  }
}

} // namespace k4
