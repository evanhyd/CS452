#pragma once

#include "message.h"

#include "kernel/syscalls.h"
#include "util/ctfmt.h"

namespace k4 {

void notify(int tid, const auto& msg) {
  char devnull;
  ::Send(tid, reinterpret_cast<const char*>(&msg), sizeof(msg), &devnull, 0);
}

template <typename... Args> void notifyStatusToUI(int uiTid, kit::FormatSpec<Args...> fmt, const Args&... args) {
  UIMsg ui{.type = UIMsgType::LogStatus, .status{.msg = ""}};
  kit::formatString(ui.status.msg, fmt, args...);
  notify(uiTid, ui);
}

} // namespace k4
