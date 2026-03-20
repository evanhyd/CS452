#pragma once

#include "message.h"

#include "kernel/syscalls.h"
#include "util/ctfmt.h"

namespace k4 {
// Send a message to the server. Expect empty response.
inline void notify(int tid, const auto& msg) {
  char devnull;
  ::Send(tid, reinterpret_cast<const char*>(&msg), sizeof(msg), &devnull, 0);
}

// Send a status message to the UI-view server.
template <typename... Args> void notifyStatusToUI(int uiTid, kit::FormatSpec<Args...> fmt, const Args&... args) {
  UIMsg ui{.type = UIMsgType::LogStatus, .status{.msg = {}}};
  kit::formatString(ui.status.msg.data(), fmt, args...);
  notify(uiTid, ui);
}

// Send a "SendMsg" message to the dispatcher server.
inline void sendToDispatcher(int dispatcherTid, const marklin::MMessage& mmsg) {
  notify(dispatcherTid, DispatcherMsg{.type = DispatcherMsgType::SendMsg, .mmsg = mmsg});
}

} // namespace k4
