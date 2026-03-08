#include "marklin_dispatcher_server_task.h"

#include "marklin/marklin_train_track.h"
#include "message.h"
#include "send_util.h"

#include "kernel/syscalls.h"
#include "marklin/marklin_message.h"
#include "system_tasks/can_server_task.h"
#include "util/ctfmt.h"
#include "util/debug.h"
#include "util/history.h"
#include "util/ring_buffer.h"

#include <ranges>

namespace k4 {

namespace {

constexpr unsigned ACK_TIMEOUT_TICKS = 100; // 1 second

// Responsible for queueing the commands,
// tracking the acknowledge status,
// and calculating the round-trip delay.
struct DispatcherState {
  History<CmdHistoryEntry, CMD_HISTORY_SIZE> cmdHistory;
  RingBuffer<marklin::MMessage, 256> cmdBuffer;
  unsigned currentTicks = 0;
  int canServerTid = -1;

  // Return true if the last sent command is acknowledged or timed out.
  bool isLastCommandAcked() const {
    if (cmdHistory.empty()) {
      return true;
    }
    const auto& last = cmdHistory.back();
    return last.ackAfter != NOT_ACKED || currentTicks - last.sentTicks >= ACK_TIMEOUT_TICKS;
  }

  // Request to send a marklin message.
  // If the queue is not empty, then append to the queue buffer.
  void sendCommand(const marklin::MMessage& msg) {
    if (cmdBuffer.empty() && isLastCommandAcked()) {
      ::TransmitCAN(canServerTid, msg);
      cmdHistory.push({msg, currentTicks, NOT_ACKED});
    } else {
      if (!cmdBuffer.full()) {
        cmdBuffer.pushBack(msg);
      }
    }
  }

  // Try send as many marklin messages as possible until the first unacknowledged message.
  // Returns true if it sends at least one message.
  bool tryFlushCommandBuffer() {
    bool flushed = false;
    while (!cmdBuffer.empty() && isLastCommandAcked()) {
      auto msg = cmdBuffer.popFront();
      ::TransmitCAN(canServerTid, msg);
      cmdHistory.push({msg, currentTicks, NOT_ACKED});
      flushed = true;
    }
    return flushed;
  }

  // Mark and calculate the latency of any sent command that matches with the response message.
  // Returns true if there's a match.
  bool processResponse(const marklin::MMessage& response) {
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

  // Notify the UI server to redraw the command history.
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

// A server sends/receives marklin message,
// and manages the marklin event queues, history, etc.
void marklinDispatcherServerTask() {
  if (::RegisterAs(MARKLIN_DISPATCHER_SERVER_NAME) < 0) {
    logError("failed to register");
  }
  int canServerTid = ::WhoIs(can_server::CAN_SERVER_NAME);
  if (canServerTid < 0) {
    logError("failed to find CAN server");
  }
  int uiServerTid = ::WhoIs(UI_VIEW_SERVER_NAME);
  if (uiServerTid < 0) {
    logError("failed to find UI server");
  }

  DispatcherState state;
  state.canServerTid = canServerTid;
  notifyStatusToUI(uiServerTid, "Ready.");

  // Enable the system and set all the switches to straight.
  state.sendCommand(marklin::MMessage::systemGoAll());
  state.sendCommand(marklin::MMessage::systemHaltAll());
  for (marklin::SwitchId id = 1; id <= 18; ++id) {
    state.sendCommand(marklin::MMessage::setSwitchState(id, marklin::SwitchState::Straight));
  }
  for (marklin::SwitchId id = 153; id <= 156; ++id) {
    state.sendCommand(marklin::MMessage::setSwitchState(id, marklin::SwitchState::Straight));
  }
  state.notifyCmdHistoryToUI(uiServerTid);

  DispatcherMsg msg;
  for (;;) {
    int senderTid;
    ::Receive(&senderTid, reinterpret_cast<char*>(&msg), sizeof(msg));
    ::Reply(senderTid, "", 0);

    switch (msg.type) {
    case DispatcherMsgType::SendMsg: {
      state.sendCommand(msg.mmsg);
      state.notifyCmdHistoryToUI(uiServerTid);
      break;
    }
    case DispatcherMsgType::ReceiveMsg: {
      if (state.processResponse(msg.mmsg)) {
        state.tryFlushCommandBuffer();
      }
      state.notifyCmdHistoryToUI(uiServerTid);
      break;
    }
    case DispatcherMsgType::TimerTick: {
      state.currentTicks = msg.time.ticks;
      if (state.tryFlushCommandBuffer()) {
        state.notifyCmdHistoryToUI(uiServerTid);
      }
      break;
    }
    }
  }
}

} // namespace k4
