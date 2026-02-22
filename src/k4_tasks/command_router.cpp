#include "command_router.h"

#include "command.h"
#include "message.h"
#include "send_util.h"

#include "kernel/syscalls.h"
#include "server_tasks/io_server.h"
#include "util/ctfmt.h"
#include "util/debug.h"

extern "C" [[noreturn]] void _reboot();

namespace k4 {

// Responsible for accepting and parsing user's terminal input.
// It sanitizes the command and forward it to the UI server and the dispatcher server.
void commandRouterTask() {
  int ioServerTid = ::WhoIs(io_server::IO_SERVER_NAME);
  if (ioServerTid < 0) {
    logError("command router: failed to find IO server");
  }
  int trainTid = ::WhoIs(TRAIN_SERVER_NAME);
  if (trainTid < 0) {
    logError("command router: failed to find train server");
  }
  int trackTid = ::WhoIs(TRACK_SERVER_NAME);
  if (trackTid < 0) {
    logError("command router: failed to find track server");
  }
  int uiTid = ::WhoIs(UI_SERVER_NAME);
  if (uiTid < 0) {
    logError("command router: failed to find UI server");
  }

  cmd::CommandBuffer cmdBuf;
  notify(uiTid, UIMsg{.type = UIMsgType::PromptClear, .empty{}});

  for (;;) {
    int c = ::Getc(ioServerTid);
    if (c < 0) {
      logError("command router: Getc failed");
    }

    char ch = static_cast<char>(c);

    if (ch == '\r' || ch == '\n') {
      cmd::ParsedCommand parsed = cmdBuf.parse();
      notify(uiTid, UIMsg{.type = UIMsgType::PromptClear, .empty{}});

      switch (parsed.tag) {
      case cmd::CmdTag::None:
        break;

      case cmd::CmdTag::SetSpeed: {
        if (parsed.setSpeed.speed > 14) {
          notifyStatusToUI(uiTid, "Invalid speed %u. Must be between 0 and 14.", parsed.setSpeed.speed);
          break;
        }
        if (parsed.setSpeed.trainNo == 0 || parsed.setSpeed.trainNo > MAX_TRAINS) {
          notifyStatusToUI(uiTid, "Invalid train number %u. Must be between 1 and %u.", parsed.setSpeed.trainNo,
                           MAX_TRAINS);
          break;
        }
        TrainMsg tm{.type = TrainMsgType::SetSpeed,
                    .setSpeed{.trainNo = static_cast<uint8_t>(parsed.setSpeed.trainNo),
                              .speed = static_cast<uint8_t>(parsed.setSpeed.speed)}};
        notify(trainTid, tm);
        break;
      }
      case cmd::CmdTag::Reverse: {
        if (parsed.reverse.trainNo == 0 || parsed.reverse.trainNo > MAX_TRAINS) {
          notifyStatusToUI(uiTid, "Invalid train number %u. Must be between 1 and %u.", parsed.reverse.trainNo,
                           MAX_TRAINS);
          break;
        }
        TrainMsg tm{.type = TrainMsgType::Reverse, .reverse{.trainNo = static_cast<uint8_t>(parsed.reverse.trainNo)}};
        notify(trainTid, tm);
        break;
      }
      case cmd::CmdTag::SetSwitch: {
        TrackMsg tm{.type = TrackMsgType::SetSwitch,
                    .setSwitch{.switchNo = static_cast<uint8_t>(parsed.setSwitch.switchNo),
                               .straight = parsed.setSwitch.straight}};
        notify(trackTid, tm);
        notifyStatusToUI(uiTid, "Set switch %u to %c.", parsed.setSwitch.switchNo,
                         parsed.setSwitch.straight ? 'S' : 'C');
        break;
      }
      case cmd::CmdTag::Quit: {
        _reboot();
        break;
      }
      case cmd::CmdTag::Invalid: {
        notifyStatusToUI(uiTid, "Usage: %s", parsed.invalid.usage);
        break;
      }
      }
    } else if (ch == '\b' || ch == 127) {
      cmdBuf.backspace();
      UIMsg ui{.type = UIMsgType::PromptDelete, .promptDelete{.index = static_cast<unsigned>(cmdBuf.length())}};
      notify(uiTid, ui);
    } else if (cmd::CommandBuffer::isCmdChar(ch)) {
      cmdBuf.push(ch);
      UIMsg ui{.type = UIMsgType::PromptInsert,
               .promptInsert{.index = static_cast<unsigned>(cmdBuf.length() - 1), .ch = ch}};
      notify(uiTid, ui);
    }
  }
}

} // namespace k4
