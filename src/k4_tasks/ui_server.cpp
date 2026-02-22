#include "ui_server.h"

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
constexpr unsigned ROW_CMD_HISTORY = 20;
constexpr unsigned ROW_STATUS = 38;
constexpr unsigned ROW_PROMPT = 40;

constexpr int DRAW_IDLE_PERIOD = 100; // draw idle percentage every 1s.

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
    logError("ui server: invalid switch id");
  }
  console.moveCursor(row, col);
  console.printf("%03u: %c", id, state == marklin::SwitchState::Straight ? 'S' : 'C');
}

} // namespace

void uiServerTask() {
  if (::RegisterAs(UI_SERVER_NAME) < 0) {
    logError("ui server: failed to register");
  }
  int ioServerTid = ::WhoIs(io_server::IO_SERVER_NAME);
  if (ioServerTid < 0) {
    logError("ui server: failed to find IO server");
  }

  int drawIdleTick = 0;

  Console console{ioServerTid};

  UIMsg msg;
  for (;;) {
    int senderTid;
    ::Receive(&senderTid, reinterpret_cast<char*>(&msg), sizeof(msg));
    ::Reply(senderTid, "", 0);

    switch (msg.type) {
    case UIMsgType::ClearScreen: {
      console.clearScreen();
      console.hideCursor();
      // redraw static labels
      console.moveCursor(ROW_SWITCHES, 1);
      console.puts("Switches:");
      console.moveCursor(ROW_SENSORS, COL_SENSORS);
      console.puts("Sensors:");
      console.moveCursor(ROW_CMD_HISTORY, 1);
      console.puts("Command history:");
      console.moveCursor(ROW_PROMPT, 1);
      console.puts("> ");
      break;
    }
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
      console.putTimestamp(msg.time.deciseconds);
      break;
    }
    case k4::UIMsgType::DrawIdleTime: {
      ++drawIdleTick;
      if (drawIdleTick == DRAW_IDLE_PERIOD) {
        drawIdleTick = 0;
        uint64_t idlePerMille = ::GetIdleTime();
        uint64_t idlePercent = idlePerMille / 10;
        uint64_t idleFraction = idlePerMille % 10;

        console.moveCursor(ROW_IDLE_TIME, COL_IDLE_TIME);
        console.printf("Idle: %02u.%02u%%", idlePercent, idleFraction);
        console.clearToEol();
      }
      break;
    }
    case UIMsgType::UpdateSwitch: {
      renderSwitch(console, msg.switchUpdate.switchNo, msg.switchUpdate.state);
      break;
    }
    case UIMsgType::RedrawSensors: {
      console.moveCursor(ROW_SENSORS, COL_SENSORS);
      console.puts("Sensors:");
      for (unsigned row = 0; row < msg.sensors.count; ++row) {
        const auto& entry = msg.sensors.entries[row];
        console.moveCursor(ROW_SENSORS + 1 + row, COL_SENSORS);
        console.putTimestamp(entry.ticks);
        console.printf(" %c%u: %s", entry.event.bank, static_cast<unsigned>(entry.event.number),
                       entry.event.newOccupied ? "Occupied" : "Free");
        console.clearToEol();
      }
      break;
    }
    case UIMsgType::RedrawCmdHistory: {
      console.moveCursor(ROW_CMD_HISTORY, 1);
      console.puts("Command history:");
      for (unsigned row = 0; row < msg.cmdHistory.count; ++row) {
        const auto& entry = msg.cmdHistory.entries[row];
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
      }
      break;
    }
    }
  }
}

} // namespace k4
