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
    {0, 19, 3},  {2, 19, 3},  {9, 24, 3},  {10, 24, 3}, {29, 23, 3}, {31, 23, 3}, {25, 19, 3}, {27, 19, 3}, {21, 15, 3},
    {23, 15, 4}, {18, 12, 4}, {19, 13, 4}, {4, 14, 4},  {6, 14, 4},  {8, 13, 4},  {9, 12, 4},  {21, 53, 3}, {23, 53, 3},
    {19, 47, 3}, {20, 46, 3}, {4, 53, 3},  {6, 53, 3},  {21, 4, 3},  {23, 4, 3},  {29, 3, 3},  {31, 3, 4},  {25, 3, 4},
    {27, 3, 4},  {16, 66, 4}, {17, 67, 4}, {17, 23, 4}, {18, 23, 4}, {17, 49, 3}, {16, 50, 3}, {29, 73, 3}, {31, 73, 3},
    {25, 40, 3}, {27, 40, 3}, {29, 43, 3}, {31, 43, 3}, {21, 39, 3}, {23, 39, 4}, {4, 39, 4},  {6, 39, 4},  {0, 41, 4},
    {2, 41, 4},  {25, 52, 4}, {27, 52, 4}, {11, 66, 3}, {10, 67, 3}, {4, 63, 3},  {6, 63, 3},  {8, 77, 3},  {9, 78, 3},
    {6, 89, 3},  {5, 89, 3},  {22, 89, 3}, {21, 89, 4}, {25, 63, 4}, {27, 63, 4}, {21, 62, 4}, {23, 62, 4}, {19, 69, 4},
    {20, 70, 4}, {11, 50, 3}, {10, 49, 3}, {7, 70, 3},  {8, 69, 3},  {4, 73, 3},  {6, 73, 3},  {0, 77, 3},  {2, 77, 3},
    {18, 78, 3}, {19, 76, 4}, {25, 75, 4}, {27, 75, 4}, {21, 73, 4}, {23, 73, 4}, {7, 45, 4},  {8, 46, 4},
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

constexpr const char* COLORS[2 * marklin::NUM_TRAIN_IN_LAB] = {
    // black on bright green, bright green
    "\033[30;102m",
    "\033[92m",
    // black on bright red, bright red
    "\033[30;101m",
    "\033[91m",
    // black on bright yellow, bright yellow
    "\033[30;103m",
    "\033[93m",
    // black on bright blue, bright blue
    "\033[30;104m",
    "\033[94m",
    // black on bright magenta, bright magenta
    "\033[30;105m",
    "\033[95m",
    // black on bright cyan, bright cyan
    "\033[30;106m",
    "\033[96m",
};
constexpr const char* RESET_COLOR = "\033[0m";
constexpr const char* BOLD = "\033[1m";
constexpr const char* FG_BLACK = "\033[30m";
constexpr uint8_t NO_TRAIN = static_cast<uint8_t>(-1);

constexpr const char* BRIGHT_FG[marklin::NUM_TRAIN_IN_LAB] = {
    "\033[92m", "\033[91m", "\033[93m", "\033[94m", "\033[95m", "\033[96m",
};
constexpr const char* REGULAR_FG[marklin::NUM_TRAIN_IN_LAB] = {
    "\033[32m", "\033[31m", "\033[33m", "\033[34m", "\033[35m", "\033[36m",
};
constexpr const char* BRIGHT_BG[marklin::NUM_TRAIN_IN_LAB] = {
    "\033[102m", "\033[101m", "\033[103m", "\033[104m", "\033[105m", "\033[106m",
};
constexpr const char* REGULAR_BG[marklin::NUM_TRAIN_IN_LAB] = {
    "\033[42m", "\033[41m", "\033[43m", "\033[44m", "\033[45m", "\033[46m",
};

struct NodeRenderStyle {
  uint8_t lockTrain;
  uint8_t pathTrain;
  uint8_t estimatedTrain;
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
  console.puts(RESET_COLOR);
  console.moveCursor(ROW_SWITCHES, 1);
  console.puts(BOLD);
  console.puts("Switches:");
  console.moveCursor(ROW_SENSORS, COL_SENSORS);
  console.puts("Sensors:");
  console.moveCursor(ROW_TRAINS, COL_TRAINS);
  console.puts("Trains:");
  console.moveCursor(ROW_CMD_HISTORY, 1);
  console.puts("Command history:");
  console.moveCursor(ROW_PROMPT, 1);
  console.puts("> ");
  console.puts(RESET_COLOR);

  for (unsigned i = 0; i < std::size(ART); ++i) {
    console.moveCursor(ROW_ART + i, COL_ART);
    console.puts(ART[i]);
  }

  NodeRenderStyle sensorStyles[80];
  NodeRenderStyle switchStyles[18];
  NodeRenderStyle centerSwitchStyles[4];
  for (unsigned i = 0; i < 80; ++i) {
    sensorStyles[i] = NodeRenderStyle{.lockTrain = NO_TRAIN, .pathTrain = NO_TRAIN, .estimatedTrain = NO_TRAIN};
  }
  for (unsigned i = 0; i < 18; ++i) {
    switchStyles[i] = NodeRenderStyle{.lockTrain = NO_TRAIN, .pathTrain = NO_TRAIN, .estimatedTrain = NO_TRAIN};
  }
  for (unsigned i = 0; i < 4; ++i) {
    centerSwitchStyles[i] = NodeRenderStyle{.lockTrain = NO_TRAIN, .pathTrain = NO_TRAIN, .estimatedTrain = NO_TRAIN};
  }

  bool cmdHistoryDirty = false;
  unsigned cmdHistoryDrawnTicks = 0;
  UIMsg::CmdHistoryData cmdHistoryToDraw;

  auto maybeDrawCmdHistory = [&](unsigned currentTicks) {
    if (cmdHistoryDirty && currentTicks - cmdHistoryDrawnTicks >= CMD_HISTORY_DEBOUNCE_TICKS) {
      cmdHistoryDrawnTicks = currentTicks;
      cmdHistoryDirty = false;
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
          console.puts(COLORS[2 * i]);
          for (unsigned j = 0; j < entry.nodeCount; ++j) {
            console.printf("%s ", entry.nodes[j]->name);
            if (j + 1 == entry.lockCount) {
              console.puts(COLORS[2 * i + 1]);
            }
          }
          console.puts(RESET_COLOR);
        }
        console.clearToEol();
      }

      // Track art
      NodeRenderStyle desiredSensorStyles[80];
      NodeRenderStyle desiredSwitchStyles[18];
      NodeRenderStyle desiredCenterSwitchStyles[4];
      for (unsigned i = 0; i < 80; ++i) {
        desiredSensorStyles[i] =
            NodeRenderStyle{.lockTrain = NO_TRAIN, .pathTrain = NO_TRAIN, .estimatedTrain = NO_TRAIN};
      }
      for (unsigned i = 0; i < 18; ++i) {
        desiredSwitchStyles[i] =
            NodeRenderStyle{.lockTrain = NO_TRAIN, .pathTrain = NO_TRAIN, .estimatedTrain = NO_TRAIN};
      }
      for (unsigned i = 0; i < 4; ++i) {
        desiredCenterSwitchStyles[i] =
            NodeRenderStyle{.lockTrain = NO_TRAIN, .pathTrain = NO_TRAIN, .estimatedTrain = NO_TRAIN};
      }

      for (unsigned i = 0; i < msg.trainStates.count; ++i) {
        const auto& entry = msg.trainStates.entries[i];
        for (unsigned j = 0; j < entry.nodeCount; ++j) {
          bool isLocked = j < entry.lockCount;
          const auto& node = *entry.nodes[j];
          if (node.type == marklin::TrackNode::Type::Sensor) {
            auto& style = desiredSensorStyles[node.num];
            if (isLocked) {
              style.lockTrain = static_cast<uint8_t>(i);
            } else {
              style.pathTrain = static_cast<uint8_t>(i);
            }
          } else if (node.type == marklin::TrackNode::Type::Branch || node.type == marklin::TrackNode::Type::Merge) {
            if (node.num >= 153) {
              auto& style = desiredCenterSwitchStyles[node.num - 153];
              if (isLocked) {
                style.lockTrain = static_cast<uint8_t>(i);
              } else {
                style.pathTrain = static_cast<uint8_t>(i);
              }
            } else {
              auto& style = desiredSwitchStyles[node.num - 1];
              if (isLocked) {
                style.lockTrain = static_cast<uint8_t>(i);
              } else {
                style.pathTrain = static_cast<uint8_t>(i);
              }
            }
          }
        }

        const marklin::TrackNode* estimatedNode = entry.train->kinematics.estimatedNode;
        if (!estimatedNode) {
          continue;
        }
        if (estimatedNode->type == marklin::TrackNode::Type::Sensor) {
          desiredSensorStyles[estimatedNode->num].estimatedTrain = static_cast<uint8_t>(i);
        } else if (estimatedNode->type == marklin::TrackNode::Type::Branch ||
                   estimatedNode->type == marklin::TrackNode::Type::Merge) {
          if (estimatedNode->num >= 153) {
            desiredCenterSwitchStyles[estimatedNode->num - 153].estimatedTrain = static_cast<uint8_t>(i);
          } else {
            desiredSwitchStyles[estimatedNode->num - 1].estimatedTrain = static_cast<uint8_t>(i);
          }
        }
      }

      auto drawLoc = [&](const ArtLoc& loc, const NodeRenderStyle& style) {
        console.moveCursor(ROW_ART + loc.row, COL_ART + loc.col);
        console.puts(RESET_COLOR);
        if (style.estimatedTrain != NO_TRAIN) {
          console.puts(REGULAR_BG[style.estimatedTrain]);
        } else if (style.lockTrain != NO_TRAIN) {
          console.puts(BRIGHT_BG[style.lockTrain]);
        }

        if (style.lockTrain != NO_TRAIN) {
          if (style.pathTrain != NO_TRAIN && style.pathTrain != style.lockTrain) {
            console.puts(REGULAR_FG[style.pathTrain]);
          } else {
            console.puts(FG_BLACK);
          }
        } else if (style.pathTrain != NO_TRAIN) {
          console.puts(BRIGHT_FG[style.pathTrain]);
        }
        for (unsigned k = 0; k < loc.len; ++k) {
          console.putc(ART[loc.row][loc.col + k]);
        }
      };

      auto styleChanged = [](const NodeRenderStyle& a, const NodeRenderStyle& b) {
        return a.lockTrain != b.lockTrain || a.pathTrain != b.pathTrain || a.estimatedTrain != b.estimatedTrain;
      };

      for (unsigned i = 0; i < 80; ++i) {
        if (styleChanged(sensorStyles[i], desiredSensorStyles[i])) {
          sensorStyles[i] = desiredSensorStyles[i];
          drawLoc(SENSOR_LOCS[i], sensorStyles[i]);
        }
      }
      for (unsigned i = 0; i < 18; ++i) {
        if (styleChanged(switchStyles[i], desiredSwitchStyles[i])) {
          switchStyles[i] = desiredSwitchStyles[i];
          drawLoc(SWITCH_LOCS[i], switchStyles[i]);
        }
      }
      for (unsigned i = 0; i < 4; ++i) {
        if (styleChanged(centerSwitchStyles[i], desiredCenterSwitchStyles[i])) {
          centerSwitchStyles[i] = desiredCenterSwitchStyles[i];
          drawLoc(CENTER_SWITCH_LOCS[i], centerSwitchStyles[i]);
        }
      }

      console.puts(RESET_COLOR);
      break;
    }
    }
  }
}

} // namespace k4
