#include "ui_view_server_task.h"

#include "marklin/marklin_def.h"
#include "marklin/marklin_train_track.h"
#include "message.h"

#include "kernel/syscalls.h"
#include "util/console.h"
#include "util/debug.h"
#include "util/history.h"
#include <cstdint>
#include <iterator>

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
constexpr unsigned STATUS_HISTORY_SIZE = 16;
constexpr unsigned ROW_PROMPT = ROW_STATUS + STATUS_HISTORY_SIZE + 2;

constexpr unsigned CMD_HISTORY_DEBOUNCE_TICKS = 50;

constexpr unsigned ROW_ART = 20;
constexpr unsigned COL_ART = 80;

constexpr const char* ART[] = {
    R"(/------------------A1>-------------------C13>--------------------------------E7>----\)",
    R"(X                        12        11                                                \)",
    R"(\------------------A2<-/   /-----/   /---C14<--------------------------------E8<--\   \)",
    R"(                      /   /     /   /                                              \   \)",
    R"(/-------------A13>---/   /     /   /---C11>----------B5>-------D3>-------E5>----\   \   \)",
    R"(X                     4 /     / 14               13                 10           \  |   |D8^)",
    R"(\-------------A14<-/   /     /   /-----C12<-----\   \B6<-------D4</   /--E6<--\   \ |   |D7v)",
    R"(                  /   /     /   /            E15^\   \           /   /E3^      \   \|   |)",
    R"(             A15v/   /     |   |              E16v\   \    =    /   /E4v     D5^\   |   |)",
    R"(            A16^/   /   A3^|   |                   \   \   |   /   /          D6v\      |)",
    R"(               |   |    A4v|   |                 E2^\   \  |  /   /D2^            \  9  |)",
    R"(               |   |       |   |                  E1v\   \ | /   /D1v              \    |)",
    R"(               |   |       |   |                      \   \|/   /                   |   |)",
    R"(               |   |       |   |                       \156 155/                    |   |)",
    R"(               |   |       |   |                       /153 154\                    |   |)",
    R"(               |   |       |   |                      /   /|\   \                   |   |)",
    R"(               |   |       |   |                  C2^/   / | \   \B13^             /    |)",
    R"(               |   |   B15^|   |                 C1v/   /  |  \   \B14v           /  8  |)",
    R"(            A11v\   \  B16v|   |                   /   /   |   \   \          E9^/      |)",
    R"(             A12^\   \     |   |               B3^/   /    =    \   \D15^   E10v/   |   |)",
    R"(                  \   \     \   \             B4v/   /           \   \D16v     /   /|   |)",
    R"(/---B7>--------A9<-\   \     \   \-----C9<------/   /B1>------D13<\   \--E13<-/   / |   |D10^)",
    R"(X                     1 \     \ 15               16                 17           /  |   |D9v)",
    R"(\---B8<--------A10>--\   \     \   \---C10>----------B2<------D14>-------E14>---/   /   /)",
    R"(                      \   \     \   \                                              /   /)",
    R"(/--B11>------------A7<-\   \     \   \--C5>---------C15>-------D11<--------E11>---/   /)",
    R"(X                         2 \     \             6                      7             /)",
    R"(\--B12<------------A8>---\   \     \----C6<---\   \-C16<-------D12>-/   /--E12<-----/)",
    R"(                          \   \                \   \               /   /)",
    R"(/--B9>-----------------A5>-\   \-----------C7>--\   \-------------/   /--C3>-------------\)",
    R"(X                             3                   18                5                    X)",
    R"(\--B10<----------------A6<-----------------C8<---------------------------C4<-------------/)",
};

struct ArtLoc {
  unsigned row;
  unsigned col;
  unsigned len;
};

constexpr ArtLoc SENSOR_LOCS[80] = {
    {0, 19, 2},  {2, 19, 2},  {9, 24, 2},  {10, 24, 2}, {29, 23, 2}, {31, 23, 2}, {25, 19, 2}, {27, 19, 2}, {21, 15, 2},
    {23, 15, 3}, {18, 12, 3}, {19, 13, 3}, {4, 14, 3},  {6, 14, 3},  {8, 13, 3},  {9, 12, 3},  {21, 53, 2}, {23, 53, 2},
    {19, 47, 2}, {20, 46, 2}, {4, 53, 2},  {6, 53, 2},  {21, 4, 2},  {23, 4, 2},  {29, 3, 2},  {31, 3, 3},  {25, 3, 3},
    {27, 3, 3},  {16, 66, 3}, {17, 67, 3}, {17, 23, 3}, {18, 23, 3}, {17, 49, 2}, {16, 50, 2}, {29, 73, 2}, {31, 73, 2},
    {25, 40, 2}, {27, 40, 2}, {29, 43, 2}, {31, 43, 2}, {21, 39, 2}, {23, 39, 3}, {4, 39, 3},  {6, 39, 3},  {0, 41, 3},
    {2, 41, 3},  {25, 52, 3}, {27, 52, 3}, {11, 66, 2}, {10, 67, 2}, {4, 63, 2},  {6, 63, 2},  {8, 77, 2},  {9, 78, 2},
    {6, 89, 2},  {5, 89, 2},  {22, 89, 2}, {21, 89, 3}, {25, 63, 3}, {27, 63, 3}, {21, 62, 3}, {23, 62, 3}, {19, 69, 3},
    {20, 70, 3}, {11, 50, 2}, {10, 49, 2}, {7, 70, 2},  {8, 69, 2},  {4, 73, 2},  {6, 73, 2},  {0, 77, 2},  {2, 77, 2},
    {18, 78, 2}, {19, 76, 3}, {25, 75, 3}, {27, 75, 3}, {21, 73, 3}, {23, 73, 3}, {7, 45, 3},  {8, 46, 3},
};
constexpr ArtLoc SWITCH_LOCS[18] = {
    {22, 22, 1}, {26, 26, 1}, {30, 30, 1}, {5, 22, 1}, {30, 68, 1}, {26, 48, 1}, {26, 71, 1}, {17, 85, 1}, {10, 85, 1},
    {5, 68, 2},  {1, 35, 2},  {1, 25, 2},  {5, 49, 2}, {5, 32, 2},  {22, 32, 2}, {22, 49, 2}, {22, 68, 2}, {30, 50, 2},
};
constexpr ArtLoc CENTER_SWITCH_LOCS[4] = {
    {14, 56, 3},
    {14, 60, 3},
    {13, 60, 3},
    {13, 56, 3},
};

const char* toString(marklin::KinematicsSystem::State state) {
  switch (state) {
  case marklin::KinematicsSystem::State::Lost:
    return "Lost";
  case marklin::KinematicsSystem::State::Tracked:
    return "Tracked";
  }
  return "Unknown";
}

const char* toString(marklin::NavigationSystem::State state) {
  switch (state) {
  case marklin::NavigationSystem::State::Manual:
    return "Manual";
  case marklin::NavigationSystem::State::FindingPath:
    return "FindingPath";
  case marklin::NavigationSystem::State::Routed:
    return "Routed";
  case marklin::NavigationSystem::State::Reversing:
    return "Reversing";
  }
  return "Unknown";
}

const char* toString(marklin::PathingState state) {
  switch (state) {
  case marklin::PathingState::Idling:
    return "Idling";
  case marklin::PathingState::Moving:
    return "Moving";
  case marklin::PathingState::Yielding:
    return "Yielding";
  case marklin::PathingState::Arriving:
    return "Arriving";
  case marklin::PathingState::Trespassing:
    return "Trespassing";
  case marklin::PathingState::Resuming:
    return "Resuming";
  }
  return "Unknown";
}

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

  for (unsigned i = 0; i < std::size(ART); ++i) {
    console.moveCursor(ROW_ART + i, COL_ART);
    console.puts(ART[i]);
  }

  static constexpr uint8_t NO_COLOR = static_cast<uint8_t>(-1);
  uint8_t sensorColors[80];
  uint8_t switchColors[18];
  uint8_t centerSwitchColors[4];
  for (unsigned i = 0; i < 80; ++i) {
    sensorColors[i] = NO_COLOR;
  }
  for (unsigned i = 0; i < 18; ++i) {
    switchColors[i] = NO_COLOR;
  }
  for (unsigned i = 0; i < 4; ++i) {
    centerSwitchColors[i] = NO_COLOR;
  }

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
        } else {
          for (unsigned i = 0; i < 20; ++i) {
            console.putc(' ');
          }
        }
      }
    }
  };

  struct MessageWithTimestamp {
    uint32_t ticks;
    std::array<char, 128> msg;
  };
  History<MessageWithTimestamp, STATUS_HISTORY_SIZE> statusHistory;
  uint32_t currentTicks = 0;

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
      statusHistory.push({
          .ticks = currentTicks,
          .msg = msg.status.msg,
      });
      console.moveCursor(ROW_STATUS + static_cast<unsigned>(statusHistory.capacity - statusHistory.size()) + 1, 1);
      for (const auto& entry : statusHistory) {
        console.putTimestamp(entry.ticks);
        console.puts(" ");
        unsigned len = console.puts(entry.msg.data()) + sizeof("00:00.0 ");
        for (unsigned i = len; i < COL_ART; ++i) {
          console.putc(' ');
        }
        console.nextLine();
      }
      break;
    }
    case UIMsgType::DrawSystemTime: {
      currentTicks = msg.time.ticks;
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
      for (unsigned i = 0; i < msg.trainStates.count; ++i) {
        const auto& entry = msg.trainStates.entries[i];
        unsigned baseRow = ROW_TRAINS + 1 + (i * 2);

        // Row 1: Kinematics and Predictions
        console.moveCursor(baseRow, COL_TRAINS);
        console.printf("Train %u [%s|%s|%s] Est %u um/t Off %u um/t | Last %s[%u mm] Est %s[%u mm]", entry.trainId,
                       toString(entry.train->kinematics.state), toString(entry.train->navigation.state),
                       toString(entry.train->navigation.oldPathingState), entry.train->kinematics.estimatedSpeed,
                       entry.train->kinematics.offlineSpeed,
                       (entry.train->kinematics.lastSensor ? entry.train->kinematics.lastSensor->name : "N/A"),
                       entry.train->kinematics.lastSensorOffset / 1000,
                       (entry.train->kinematics.estimatedNode ? entry.train->kinematics.estimatedNode->name : "N/A"),
                       entry.train->kinematics.estimatedNodeOffset / 1000);
        if (entry.train->prediction.lastTimeErrorTicks != 0 || entry.train->prediction.lastDistErrorUm != 0) {
          console.printf(" [Err: %dt %dmm]", entry.train->prediction.lastTimeErrorTicks,
                         entry.train->prediction.lastDistErrorUm / 1000);
        }
        if (entry.train->prediction.sensor) {
          console.printf(" | Exp: %s", entry.train->prediction.sensor->name);
          if (entry.train->prediction.predictedTicks != 0) {
            console.puts(" @ ");
            console.putTicks(entry.train->prediction.predictedTicks);
          }
        }
        console.clearToEol();

        // Row 2: Locks
        console.moveCursor(baseRow + 1, COL_TRAINS);
        console.printf("  Path[%u|%u]: ", entry.lockCount, entry.nodeCount);
        if (entry.nodeCount == 0) {
          console.puts("None");
        } else {
          for (unsigned j = 0; j < entry.nodeCount; ++j) {
            console.printf("%s ", entry.nodes[j]->name);
            if (j + 1 == entry.lockCount) {
              console.printf("### ");
            }
          }
        }
        console.clearToEol();
      }

      // Track art
      static constexpr const char* COLORS[2 * marklin::NUM_TRAIN_IN_LAB] = {
          // bright green, green
          "\033[92m",
          "\033[32m",
          // bright red, red
          "\033[91m",
          "\033[31m",
          // bright yellow, yellow
          "\033[93m",
          "\033[33m",
          // bright blue, blue
          "\033[94m",
          "\033[34m",
          // bright magenta, magenta
          "\033[95m",
          "\033[35m",
          // bright cyan, cyan
          "\033[96m",
          "\033[36m",
      };
      static constexpr const char* RESET_COLOR = "\033[0m";

      uint8_t desiredSensorColors[80];
      uint8_t desiredSwitchColors[18];
      uint8_t desiredCenterSwitchColors[4];
      for (unsigned i = 0; i < 80; ++i) {
        desiredSensorColors[i] = NO_COLOR;
      }
      for (unsigned i = 0; i < 18; ++i) {
        desiredSwitchColors[i] = NO_COLOR;
      }
      for (unsigned i = 0; i < 4; ++i) {
        desiredCenterSwitchColors[i] = NO_COLOR;
      }

      for (unsigned i = 0; i < msg.trainStates.count; ++i) {
        const auto& entry = msg.trainStates.entries[i];
        for (unsigned j = 0; j < entry.nodeCount; ++j) {
          uint8_t colorIndex = static_cast<uint8_t>(2 * i + (j >= entry.lockCount));
          const auto& node = *entry.nodes[j];
          if (node.type == marklin::TrackNode::Type::Sensor) {
            desiredSensorColors[node.num - 1] = colorIndex;
          } else if (node.type == marklin::TrackNode::Type::Branch || node.type == marklin::TrackNode::Type::Merge) {
            if (node.num >= 153) {
              desiredCenterSwitchColors[node.num - 153] = colorIndex;
            } else {
              desiredSwitchColors[node.num - 1] = colorIndex;
            }
          }
        }
      }

      auto drawLoc = [&](const ArtLoc& loc, uint8_t colorIndex) {
        console.moveCursor(ROW_ART + loc.row, COL_ART + loc.col);
        if (colorIndex == NO_COLOR) {
          console.puts(RESET_COLOR);
        } else {
          console.puts(COLORS[colorIndex]);
        }
        for (unsigned k = 0; k < loc.len; ++k) {
          console.putc(ART[loc.row][loc.col + k]);
        }
      };

      for (unsigned i = 0; i < 80; ++i) {
        if (sensorColors[i] != desiredSensorColors[i]) {
          sensorColors[i] = desiredSensorColors[i];
          drawLoc(SENSOR_LOCS[i], sensorColors[i]);
        }
      }
      for (unsigned i = 0; i < 18; ++i) {
        if (switchColors[i] != desiredSwitchColors[i]) {
          switchColors[i] = desiredSwitchColors[i];
          drawLoc(SWITCH_LOCS[i], switchColors[i]);
        }
      }
      for (unsigned i = 0; i < 4; ++i) {
        if (centerSwitchColors[i] != desiredCenterSwitchColors[i]) {
          centerSwitchColors[i] = desiredCenterSwitchColors[i];
          drawLoc(CENTER_SWITCH_LOCS[i], centerSwitchColors[i]);
        }
      }

      console.puts(RESET_COLOR);
      break;
    }
    }
  }
}

} // namespace k4
