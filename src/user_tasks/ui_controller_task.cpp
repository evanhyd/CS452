#include "ui_controller_task.h"

#include "command.h"
#include "marklin/marklin_train_track.h"
#include "message.h"
#include "send_util.h"

#include "kernel/syscalls.h"
#include "system_tasks/io_server_task.h"
#include "util/ctfmt.h"
#include "util/debug.h"

extern "C" [[noreturn]] void _reboot();

namespace k4 {

// Responsible for accepting and parsing user's terminal input.
// It sanitizes the command and forward it to the UI server and the dispatcher server.
void uiControllerTask() {
  int ioServerTid = ::WhoIs(io_server::IO_SERVER_NAME);
  if (ioServerTid < 0) {
    logError("failed to find IO server");
  }
  int trainTrackTid = ::WhoIs(TRAIN_TRACK_SERVER_NAME);
  if (trainTrackTid < 0) {
    logError("failed to find train track server");
  }
  int uiTid = ::WhoIs(UI_VIEW_SERVER_NAME);
  if (uiTid < 0) {
    logError("failed to find UI server");
  }

  cmd::CommandBuffer cmdBuf;
  notify(uiTid, UIMsg{.type = UIMsgType::PromptClear, .empty{}});

  for (;;) {
    int c = ::Getc(ioServerTid);
    if (c < 0) {
      logError("Getc failed");
    }

    char ch = static_cast<char>(c);

    if (ch == '\r' || ch == '\n') {
      cmd::ParsedCommand parsed = cmdBuf.parse();
      notify(uiTid, UIMsg{.type = UIMsgType::PromptClear, .empty{}});

      switch (parsed.tag) {
      case cmd::CmdTag::None: {
        notifyStatusToUI(uiTid, "Invalid command.");
        break;
      }
      case cmd::CmdTag::SetSpeed: {
        if (!marklin::isValidSpeedLevel(parsed.setSpeed.speedLevel)) {
          notifyStatusToUI(uiTid, "Invalid speed %u. Must be between 0 and 14.", parsed.setSpeed.speedLevel);
          break;
        }
        if (!marklin::isValidTrainId(parsed.setSpeed.trainId)) {
          notifyStatusToUI(uiTid, "Invalid train number %u. Must be between 1 and %u.", parsed.setSpeed.trainId,
                           marklin::NUM_TRAINS);
          break;
        }
        TrainTrackMsg tm{.type = TrainTrackMsgType::SetSpeedCmd,
                         .setSpeedCmd{.trainId = parsed.setSpeed.trainId, .speedLevel = parsed.setSpeed.speedLevel}};
        notify(trainTrackTid, tm);
        break;
      }
      case cmd::CmdTag::Reverse: {
        if (!marklin::isValidTrainId(parsed.reverse.trainId)) {
          notifyStatusToUI(uiTid, "Invalid train number %u. Must be between 1 and %u.", parsed.reverse.trainId,
                           marklin::NUM_TRAINS);
          break;
        }
        TrainTrackMsg tm{.type = TrainTrackMsgType::ReverseCmd, .reverseCmd{.trainId = parsed.reverse.trainId}};
        notify(trainTrackTid, tm);
        break;
      }
      case cmd::CmdTag::SetSwitch: {
        if (!marklin::isValidSwitchId(parsed.setSwitch.switchId)) {
          notifyStatusToUI(uiTid, "Invalid switch id %u.", parsed.setSwitch.switchId);
          break;
        }
        TrainTrackMsg tm{.type = TrainTrackMsgType::SetSwitchCmd,
                         .setSwitchCmd{.switchId = parsed.setSwitch.switchId, .state = parsed.setSwitch.state}};
        notify(trainTrackTid, tm);
        notifyStatusToUI(uiTid, "Set switch %u to %c.", parsed.setSwitch.switchId,
                         parsed.setSwitch.state == marklin::SwitchState::Straight ? 'S' : 'C');
        break;
      }
      case cmd::CmdTag::SetTrack: {
        TrainTrackMsg tm{.type = TrainTrackMsgType::SetTrackCmd, .setTrackCmd{.trackId = parsed.setTrack.trackId}};
        notify(trainTrackTid, tm);
        notifyStatusToUI(uiTid, "Set track to %c.", parsed.setTrack.trackId == 0 ? 'A' : 'B');
        break;
      }
      case cmd::CmdTag::Quit: {
        _reboot();
        break;
      }
      case cmd::CmdTag::Goto: {
        if (!marklin::isValidTrainId(parsed.gotoData.trainId)) {
          notifyStatusToUI(uiTid, "Invalid train number %u. Must be between 1 and %u.", parsed.gotoData.trainId,
                           marklin::NUM_TRAINS);
          break;
        }
        TrainTrackMsg tm{.type = TrainTrackMsgType::GotoCmd, .gotoCmd{}};
        tm.gotoCmd.trainId = parsed.gotoData.trainId;
        for (int i = 0; i < 16; ++i) {
          tm.gotoCmd.location[i] = parsed.gotoData.location[i];
        }
        notify(trainTrackTid, tm);
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
