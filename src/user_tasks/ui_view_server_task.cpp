#include "ui_view_server_task.h"

#include "marklin/marklin_def.h"
#include "marklin/marklin_train_track.h"
#include "message.h"
#include "track_art.h"

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

constexpr const char* COLORS[2 * marklin::NUM_TRAIN_IN_LAB] = {
    // green
    "\033[30;102m",
    "\033[40;92m",
    // red
    "\033[30;101m",
    "\033[40;91m",
    // blue
    "\033[30;104m",
    "\033[40;94m",
    // magenta
    "\033[30;105m",
    "\033[40;95m",
    // cyan
    "\033[30;106m",
    "\033[40;96m",
    // yellow
    "\033[30;103m",
    "\033[40;93m",
};
constexpr const char* RESET_COLOR = "\033[0m";
constexpr const char* BOLD = "\033[1m";
constexpr const char* FG_BLACK = "\033[30m";
constexpr const char* FG_BRIGHT_BLACK = "\033[90m";
constexpr const char* FG_BRIGHT_WHITE_BOLD = "\033[1;97m";
constexpr const char* FG_BRIGHT_WHITE = "\033[97m";
constexpr uint64_t ALL_PACMAN_DOTS_MASK = (uint64_t{1} << PACMAN_DOT_COUNT) - 1;
constexpr uint8_t NO_TRAIN = static_cast<uint8_t>(-1);

constexpr const char* BRIGHT_FG[marklin::NUM_TRAIN_IN_LAB] = {
    "\033[92m", "\033[91m", "\033[94m", "\033[95m", "\033[96m", "\033[93m",
};
constexpr const char* REGULAR_FG[marklin::NUM_TRAIN_IN_LAB] = {
    "\033[32m", "\033[31m", "\033[34m", "\033[35m", "\033[36m", "\033[33m",
};
constexpr const char* BRIGHT_BG[marklin::NUM_TRAIN_IN_LAB] = {
    "\033[102m", "\033[101m", "\033[104m", "\033[105m", "\033[106m", "\033[103m",
};
constexpr const char* REGULAR_BG_BOLD[marklin::NUM_TRAIN_IN_LAB] = {
    "\033[1;42m", "\033[1;41m", "\033[1;44m", "\033[1;45m", "\033[1;46m", "\033[1;43m",
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
  console.puts(active ? FG_BRIGHT_WHITE_BOLD : FG_BRIGHT_BLACK);
  console.putc(ART[loc.row][loc.col]);
  console.puts(RESET_COLOR);
}

void renderSwitchStateMarkers(Console& console, unsigned id, marklin::SwitchState state, bool isKnown) {
  const auto& locs = getSwitchStateArtLoc(id);
  drawSwitchStateMarker(console, locs[0], isKnown && state == marklin::SwitchState::Curved);
  drawSwitchStateMarker(console, locs[1], isKnown && state == marklin::SwitchState::Straight);
}

void drawPacmanDot(Console& console, unsigned dotIndex, bool active) {
  const auto& loc = DOT_LOCS[dotIndex];
  console.moveCursor(ROW_ART + loc.row, COL_ART + loc.col);
  if (active) {
    console.puts("\033[93mo");
    console.puts(RESET_COLOR);
  } else {
    console.putc(' ');
  }
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
  console.enableMouse();
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
  for (unsigned i = 0; i < PACMAN_DOT_COUNT; ++i) {
    drawPacmanDot(console, i, true);
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
  uint64_t pacmanDotsMask = ALL_PACMAN_DOTS_MASK;

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
    case UIMsgType::RedrawPacmanDots: {
      for (unsigned i = 0; i < PACMAN_DOT_COUNT; ++i) {
        uint64_t bit = uint64_t{1} << i;
        bool oldActive = (pacmanDotsMask & bit) != 0;
        bool newActive = (msg.pacmanDots.activeMask & bit) != 0;
        if (oldActive != newActive) {
          drawPacmanDot(console, i, newActive);
        }
      }
      pacmanDotsMask = msg.pacmanDots.activeMask;
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
            if (j == entry.lockCount) {
              console.puts(COLORS[2 * i + 1]);
            }
            if (j > 0) {
              console.putc(' ');
            }
            console.printf("%s", entry.nodes[j]->name);
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
