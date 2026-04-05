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
    R"(   /-------------C3<---------------------------C7<-----------------A5<----------------B9<---\)",
    R"(   X                  < 5                18 >                 3 >                           X)",
    R"(   \-------------C4>--/ v /-------------\ v \--C8>-----------\ v \-A6>----------------B10>--/)",
    R"(                     /   /               \   \                \   \)",
    R"(        /-----E11<--/ ^ /-D11>-------C15<-\ ^ \---C5<----\     \   \---A7>------------B11<--\)",
    R"(       /             7 >                   < 6            \     \ 2 >                       X)",
    R"(      /   /---E12>--------D12<-------C16>---------C6>--\   \     \ v \-A8<------------B12>--/)",
    R"(     /   /                                              \   \     \   \)",
    R"(    /   /   /---E13>-------D13>------B1<----------C9>----\ ^ \     \   \---A9>--------B7<---\)",
    R"( D9^|   |  /           17 >             < 16             < 15 \     \ 1 >                   X)",
    R"(D10v|   | /   /-E14<--\ v \D14<------B2>/ v /-----C10<-----\   \     \ v \-A10<-------B8>---/)",
    R"(    |   |/   /     D15v\   \           /   /B3v             \   \     \   \)",
    R"(    |   |   /E9v    D16^\   \    =    /   /B4^               |   |     \   \A11^)",
    R"(    | ^  > /E10^         \   \   |   /   /                   |   |B15v  \   \A12v)",
    R"(    |  8  /           B13v\   \  |  /   /C1^                 |   |B16^   |   |)",
    R"(    |    /             B14^\   \ | /   /C2v                  |   |       |   |)",
    R"(    |   |                   \   \|/   /                      |   |       |   |)",
    R"(    |   |                 SC \154 153/ SC                    |   |       |   |)",
    R"(    |   |                 SC /155 156\ SC                    |   |       |   |)",
    R"(    |   |                   /   /|\   \                      |   |       |   |)",
    R"(    |    \              D1v/   / | \   \E1v                  |   |       |   |)",
    R"(    |  9  \            D2^/   /  |  \   \E2^                 |   |A3v    |   |)",
    R"(    | v  > \D5v          /   /   |   \   \                   |   |A4^   /   /A15^)",
    R"(    |   |   \D6^     E3v/   /    =    \   \E15v              |   |     /   /A16v)",
    R"(    |   |\   \      E4^/   /           \   \E16^            /   /     /   /)",
    R"( D7^|   | \   \--E5<--/ ^ /D3<-------B5<\ ^ \-----C11<-----/   /     / ^ /-A13<-------------\)",
    R"( D8v|   |  \           10 >             < 13             < 14 /     / 4 >                   X)",
    R"(    \   \   \----E6>-------D4>-------B6>----------C12>---/ v /     /   /---A14>-------------/)",
    R"(     \   \                                              /   /     /   /)",
    R"(      \   \--E7<--------------------------------C13<---/ ^ /-----/ ^ /-A1<------------------\)",
    R"(       \                                                11 >      12 >                      X)",
    R"(        \----E8>--------------------------------C14>-------------------A2>------------------/)",
};

struct ArtLoc {
  unsigned row;
  unsigned col;
  unsigned len;
};

constexpr ArtLoc SENSOR_LOCS[80] = {
    {29, 71, 3}, {31, 71, 3}, {21, 66, 3}, {22, 66, 3}, {0, 67, 3},  {2, 67, 3},  {4, 71, 3},  {6, 71, 3},  {8, 75, 3},
    {10, 75, 4}, {12, 76, 4}, {13, 77, 4}, {25, 75, 4}, {27, 75, 4}, {22, 77, 4}, {23, 76, 4}, {8, 37, 3},  {10, 37, 3},
    {11, 44, 3}, {12, 43, 3}, {25, 37, 3}, {27, 37, 3}, {8, 86, 3},  {10, 86, 3}, {0, 86, 3},  {2, 86, 4},  {4, 86, 4},
    {6, 86, 4},  {14, 22, 4}, {15, 23, 4}, {13, 66, 4}, {14, 66, 4}, {14, 41, 3}, {15, 40, 3}, {0, 17, 3},  {2, 17, 3},
    {4, 50, 3},  {6, 50, 3},  {0, 47, 3},  {2, 47, 3},  {8, 50, 3},  {10, 50, 4}, {25, 50, 4}, {27, 50, 4}, {29, 48, 4},
    {31, 48, 4}, {4, 37, 4},  {6, 37, 4},  {20, 24, 3}, {21, 23, 3}, {25, 27, 3}, {27, 27, 3}, {22, 12, 3}, {23, 13, 3},
    {25, 1, 3},  {26, 1, 3},  {9, 1, 3},   {10, 0, 4},  {4, 26, 4},  {6, 26, 4},  {8, 27, 4},  {10, 27, 4}, {11, 19, 4},
    {12, 20, 4}, {20, 40, 3}, {21, 41, 3}, {23, 21, 3}, {24, 20, 3}, {25, 17, 3}, {27, 17, 3}, {29, 13, 3}, {31, 13, 3},
    {12, 13, 3}, {13, 12, 4}, {4, 14, 4},  {6, 14, 4},  {8, 16, 4},  {10, 16, 4}, {23, 43, 4}, {24, 44, 4},
};
constexpr ArtLoc SWITCH_LOCS[18] = {
    {9, 70, 1},  {5, 66, 1},  {1, 62, 1},  {26, 70, 1}, {1, 24, 1},  {5, 45, 1}, {5, 21, 1}, {14, 7, 1}, {21, 7, 1},
    {26, 23, 2}, {30, 56, 2}, {30, 66, 2}, {26, 42, 2}, {26, 59, 2}, {9, 59, 2}, {9, 42, 2}, {9, 23, 2}, {1, 41, 2},
};
constexpr ArtLoc CENTER_SWITCH_LOCS[4] = {
    {17, 34, 3},
    {17, 30, 3},
    {18, 30, 3},
    {18, 34, 3},
};
constexpr ArtLoc SWITCH_STATE_LOCS[18][2] = {
    {{9, 72, 1}, {10, 71, 1}},  {{5, 68, 1}, {6, 67, 1}},   {{1, 64, 1}, {2, 63, 1}},   {{26, 72, 1}, {25, 71, 1}},
    {{2, 24, 1}, {1, 22, 1}},   {{4, 44, 1}, {5, 43, 1}},   {{4, 22, 1}, {5, 23, 1}},   {{13, 9, 1}, {13, 6, 1}},
    {{22, 9, 1}, {22, 6, 1}},   {{25, 24, 1}, {26, 26, 1}}, {{29, 57, 1}, {30, 59, 1}}, {{29, 67, 1}, {30, 69, 1}},
    {{25, 42, 1}, {26, 40, 1}}, {{26, 57, 1}, {27, 59, 1}}, {{9, 57, 1}, {8, 59, 1}},   {{10, 42, 1}, {9, 40, 1}},
    {{10, 24, 1}, {9, 26, 1}},  {{2, 42, 1}, {1, 44, 1}},
};
constexpr ArtLoc CENTER_SWITCH_STATE_LOCS[4][2] = {
    {{17, 40, 1}, {17, 39, 1}},
    {{17, 27, 1}, {17, 26, 1}},
    {{18, 27, 1}, {18, 26, 1}},
    {{18, 40, 1}, {18, 39, 1}},
};

constexpr const char* COLORS[2 * marklin::NUM_TRAIN_IN_LAB] = {
    // green
    "\033[30;102m",
    "\033[40;92m",
    // red
    "\033[30;101m",
    "\033[40;91m",
    // yellow
    "\033[30;103m",
    "\033[40;93m",
    // blue
    "\033[30;104m",
    "\033[40;94m",
    // magenta
    "\033[30;105m",
    "\033[40;95m",
    // cyan
    "\033[30;106m",
    "\033[40;96m",
};
constexpr const char* RESET_COLOR = "\033[0m";
constexpr const char* BOLD = "\033[1m";
constexpr const char* FG_BLACK = "\033[30m";
constexpr const char* FG_BRIGHT_BLACK = "\033[90m";
constexpr const char* FG_BRIGHT_WHITE = "\033[1;97m";
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
constexpr const char* REGULAR_BG_BOLD[marklin::NUM_TRAIN_IN_LAB] = {
    "\033[1;42m", "\033[1;41m", "\033[1;43m", "\033[1;44m", "\033[1;45m", "\033[1;46m",
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

auto getSwitchStateArtLoc(unsigned int id) -> const ArtLoc (&)[2] {
  if (id >= 1 && id <= 18) {
    return SWITCH_STATE_LOCS[id - 1];
  }
  if (id >= 153 && id <= 156) {
    return CENTER_SWITCH_STATE_LOCS[id - 153];
  }
  logError("invalid switch id");
}

void drawSwitchStateMarker(Console& console, const ArtLoc& loc, bool active) {
  console.moveCursor(ROW_ART + loc.row, COL_ART + loc.col);
  console.puts(active ? FG_BRIGHT_WHITE : FG_BRIGHT_BLACK);
  console.putc(ART[loc.row][loc.col]);
  console.puts(RESET_COLOR);
}

void renderSwitchStateMarkers(Console& console, unsigned id, marklin::SwitchState state, bool isKnown) {
  const auto& locs = getSwitchStateArtLoc(id);
  drawSwitchStateMarker(console, locs[0], isKnown && state == marklin::SwitchState::Curved);
  drawSwitchStateMarker(console, locs[1], isKnown && state == marklin::SwitchState::Straight);
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
  renderSwitchStateMarkers(console, id, state, true);
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
  for (marklin::SwitchId id = 1; id <= 18; ++id) {
    renderSwitchStateMarkers(console, id, marklin::SwitchState::Straight, false);
  }
  for (marklin::SwitchId id = 153; id <= 156; ++id) {
    renderSwitchStateMarkers(console, id, marklin::SwitchState::Straight, false);
  }
  console.puts(RESET_COLOR);

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
          console.puts(REGULAR_BG_BOLD[style.estimatedTrain]);
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
