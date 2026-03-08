#include "ui_view_server_task.h"

#include "marklin/marklin_train_track.h"
#include "message.h"

#include "kernel/syscalls.h"
#include "util/console.h"
#include "util/debug.h"

namespace k4 {

namespace {

constexpr unsigned ROW_SYSTEM_TIME = 1;
constexpr unsigned ROW_IDLE_TIME = 1;
constexpr unsigned COL_IDLE_TIME = 30;
constexpr unsigned ROW_SWITCHES = 3;
constexpr unsigned ROW_SENSORS = 3;
constexpr unsigned COL_SENSORS = 30;
constexpr unsigned ROW_TRAINS = 3;
constexpr unsigned COL_TRAINS = 50;
constexpr unsigned ROW_CMD_HISTORY = 20;
constexpr unsigned ROW_STATUS = 40;
constexpr unsigned ROW_PROMPT = 42;

constexpr unsigned CMD_HISTORY_DEBOUNCE_TICKS = 10;

void renderSwitch(Console& console, unsigned id, marklin::SwitchState state) {
  unsigned row, col;
  if (id >= 1 && id <= 11) {
    row = ROW_SWITCHES + id;
    col = 1;
  } else if (id >= 12 && id <= 18) {
    row = ROW_SWITCHES + id - 12 + 1;
    col = 11;
  } else if (id >= 153 && id <= 156) {
    row = ROW_SWITCHES + id - 153 + 8;
    col = 11;
  } else {
    logError("invalid switch id");
  }
  console.moveCursor(row, col);
  console.printf("%03u: %c", id, state == marklin::SwitchState::Straight ? 'S' : 'C');
}

} // namespace

void uiViewServerTask() {
  if (::RegisterAs(UI_VIEW_SERVER_NAME) < 0) {
    logError("failed to register");
  }
  int ioServerTid = ::WhoIs(io_server::IO_SERVER_NAME);
  if (ioServerTid < 0) {
    logError("failed to find IO server");
  }

  Console console{ioServerTid};

  console.clearScreen();
  console.hideCursor();

  console.moveCursor(ROW_SWITCHES, 1);
  console.puts("Switches:");
  console.moveCursor(ROW_SENSORS, COL_SENSORS);
  console.puts("Sensors:");
  console.moveCursor(ROW_TRAINS, COL_TRAINS);
  console.puts("Trains:");
  console.moveCursor(ROW_CMD_HISTORY, 1);
  console.puts("Command history:");
  console.moveCursor(ROW_PROMPT, 1);
  console.puts("> ");

  bool cmdHistoryDirty = false;
  unsigned cmdHistoryDrawnTicks = 0;
  UIMsg::CmdHistoryData cmdHistoryToDraw;

  auto maybeDrawCmdHistory = [&](unsigned currentTicks) {
    if (cmdHistoryDirty && currentTicks - cmdHistoryDrawnTicks >= CMD_HISTORY_DEBOUNCE_TICKS) {
      cmdHistoryDrawnTicks = currentTicks;
      cmdHistoryDirty = false;
      console.moveCursor(ROW_CMD_HISTORY, 1);
      console.puts("Command history:");
      for (unsigned row = 0; row < cmdHistoryToDraw.count; ++row) {
        const auto& entry = cmdHistoryToDraw.entries[row];
        console.moveCursor(ROW_CMD_HISTORY + 1 + row, 1);
        console.putTicks(entry.sentTicks);
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
      }
    }
  };

  UIMsg msg;
  for (;;) {
    int senderTid;
    ::Receive(&senderTid, reinterpret_cast<char*>(&msg), sizeof(msg));
    ::Reply(senderTid, "", 0);

    switch (msg.type) {
    case UIMsgType::PromptInsert: {
      console.moveCursor(ROW_PROMPT, 3 + msg.promptInsert.index);
      console.putc(msg.promptInsert.ch);
      break;
    }
    case UIMsgType::PromptDelete: {
      console.moveCursor(ROW_PROMPT, 3 + msg.promptDelete.index);
      console.putc(' ');
      break;
    }
    case UIMsgType::PromptClear: {
      console.moveCursor(ROW_PROMPT, 3);
      console.clearToEol();
      break;
    }
    case UIMsgType::LogStatus: {
      console.moveCursor(ROW_STATUS, 1);
      console.puts(msg.status.msg);
      console.clearToEol();
      break;
    }
    case UIMsgType::DrawSystemTime: {
      console.moveCursor(ROW_SYSTEM_TIME, 1);
      console.putTimestamp(msg.time.ticks);
      maybeDrawCmdHistory(msg.time.ticks);
      break;
    }
    case k4::UIMsgType::DrawIdleTime: {
      uint64_t idlePerMille = ::GetIdleTime();
      uint64_t idlePercent = idlePerMille / 10;
      uint64_t idleFraction = idlePerMille % 10;

      console.moveCursor(ROW_IDLE_TIME, COL_IDLE_TIME);
      console.printf("Idle: %02u.%u%%", idlePercent, idleFraction);
      console.clearToEol();
      break;
    }
    case UIMsgType::UpdateSwitch: {
      renderSwitch(console, msg.switchUpdate.switchId, msg.switchUpdate.state);
      break;
    }
    case UIMsgType::RedrawSensors: {
      console.moveCursor(ROW_SENSORS, COL_SENSORS);
      console.puts("Sensors:");
      for (unsigned row = 0; row < msg.sensorHistory.count; ++row) {
        const auto& entry = msg.sensorHistory.entries[row];
        console.moveCursor(ROW_SENSORS + 1 + row, COL_SENSORS);
        console.putTicks(entry.ticks);
        auto [sensorBank, sensorNumber] = marklin::trackNodeIdToSensor(entry.event.id);
        console.printf(" %c%u ", sensorBank, sensorNumber); // need the extra space in the end
        // if (entry.hasPrediction) {
        //   auto [predictedBank, predictedNumber] = marklin::trackNodeIdToSensor(entry.predictedId);
        //   console.printf(" -> Exp: %c%u @ ", predictedBank, predictedNumber);
        //   console.putTicks(entry.predictedTicks);
        //   if (entry.timeErrorTicks != 0 || entry.distErrorMm != 0) {
        //     console.printf(" [Err: %dt, %dmm]", entry.timeErrorTicks, entry.distErrorMm);
        //   }
        // }
      }
      break;
    }
    case UIMsgType::RedrawCmdHistory: {
      cmdHistoryToDraw = msg.cmdHistory;
      cmdHistoryDirty = true;
      break;
    }
    case UIMsgType::TrainStates: {
      console.moveCursor(ROW_TRAINS, COL_TRAINS);
      console.puts("Trains:");
      for (unsigned row = 0; row < msg.trainStates.count; ++row) {
        const auto& entry = msg.trainStates.entries[row];
        console.moveCursor(ROW_TRAINS + 1 + row, COL_TRAINS);
        marklin::TrainId trainId = entry.trainId;
        marklin::Sensor lastSensor =
            (entry.lastTrackNode ? marklin::trackNodeIdToSensor(entry.lastTrackNode->id) : marklin::Sensor{'?', 0});
        marklin::Sensor estSensor =
            (entry.estimatedTrackNode ? marklin::trackNodeIdToSensor(entry.estimatedTrackNode->id)
                                      : marklin::Sensor{'?', 0});
        console.printf("Train %d, Last Sensor %c%d, Est Sensor %c%d, Est Offset %d um, Est Speed %d um/t", trainId,
                       lastSensor.bank, lastSensor.number, estSensor.bank, estSensor.number, entry.estimatedOffset,
                       entry.estimatedSpeed);
        console.clearToEol();
      }
      break;
    }
    }
  }
}

} // namespace k4
