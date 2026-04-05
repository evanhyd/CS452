#include "pacman_server_task.h"

#include "kernel/syscalls.h"
#include "message.h"
#include "send_util.h"
#include "system_tasks/clock_server_task.h"
#include "util/debug.h"
#include "util/kit_algorithm.h" // strncpy

#include <array>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace k4 {
namespace {

constexpr int PACMAN_TICK_PERIOD_TICKS = 50;
constexpr marklin::Distance GHOST_CATCH_DISTANCE_UM = 400'000;
constexpr marklin::SpeedLevel GHOST_SPEED_LEVEL = 5;
constexpr unsigned AMBUSH_LOOKAHEAD_NODES = 2;

constexpr marklin::TrainId DEFAULT_HUMAN_TRAIN_ID = 13;
constexpr marklin::TrainId DEFAULT_CHASER_TRAIN_ID = 14;
constexpr marklin::TrainId DEFAULT_AMBUSHER_TRAIN_ID = 15;

constexpr size_t DOT_SENSOR_COUNT = PACMAN_DOT_COUNT;

constexpr marklin::Distance absDist(marklin::Distance a, marklin::Distance b) { return a > b ? a - b : b - a; }

marklin::TrackNode* projectLookAheadNode(marklin::TrainTrackState& trackState, marklin::TrackNodeId sourceNodeId,
                                         marklin::TrainDirection direction, unsigned nodesAhead) {
  if (sourceNodeId == INVALID_TRACK_NODE_ID) {
    return nullptr;
  }

  marklin::TrackNode* node = &trackState.getTrackNodeById(sourceNodeId);
  if (direction == marklin::TrainDirection::Backward && node->reverse != nullptr) {
    node = node->reverse;
  }

  for (unsigned step = 0; step < nodesAhead && node != nullptr; ++step) {
    if (node->type == marklin::TrackNode::Type::Exit || node->type == marklin::TrackNode::Type::None) {
      break;
    }
    node = node->edges[marklin::TrackDirection::Straight].dest;
  }

  return node;
}

marklin::Distance shortestPathDistance(marklin::TrainTrackState& trackState, marklin::TrackNodeId sourceNodeId,
                                       marklin::TrackNodeId destNodeId) {
  static constexpr marklin::Distance INF = std::numeric_limits<marklin::Distance>::max() / 4;
  if (sourceNodeId == INVALID_TRACK_NODE_ID || destNodeId == INVALID_TRACK_NODE_ID) {
    return INF;
  }

  std::array<marklin::Distance, marklin::NUM_TRACK_NODES> minDistance{};
  std::array<bool, marklin::NUM_TRACK_NODES> visited{};
  for (marklin::Distance& dist : minDistance) {
    dist = INF;
  }

  const size_t sourceIdx = static_cast<size_t>(sourceNodeId);
  const size_t destIdx = static_cast<size_t>(destNodeId);
  minDistance[sourceIdx] = 0;

  for (size_t iter = 0; iter < marklin::NUM_TRACK_NODES; ++iter) {
    size_t bestNodeIdx = marklin::NUM_TRACK_NODES;
    marklin::Distance bestDistance = INF;

    for (size_t nodeIdx = 0; nodeIdx < marklin::NUM_TRACK_NODES; ++nodeIdx) {
      if (!visited[nodeIdx] && minDistance[nodeIdx] < bestDistance) {
        bestDistance = minDistance[nodeIdx];
        bestNodeIdx = nodeIdx;
      }
    }

    if (bestNodeIdx == marklin::NUM_TRACK_NODES) {
      break;
    }
    if (bestNodeIdx == destIdx) {
      return minDistance[bestNodeIdx];
    }

    visited[bestNodeIdx] = true;
    marklin::TrackNode& node = trackState.getTrackNodeById(static_cast<marklin::TrackNodeId>(bestNodeIdx));
    size_t numEdges = marklin::outgoingEdgeCount(node);
    for (size_t i = 0; i < numEdges; ++i) {
      const marklin::TrackEdge& edge = node.edges[i];
      if (edge.dest == nullptr) {
        continue;
      }
      const size_t neighborIdx = static_cast<size_t>(edge.dest->id);
      const marklin::Distance candidateDistance = minDistance[bestNodeIdx] + edge.dist;
      if (candidateDistance < minDistance[neighborIdx]) {
        minDistance[neighborIdx] = candidateDistance;
      }
    }
  }

  return minDistance[destIdx];
}

struct GameState {
  marklin::TrainId humanTrainId = DEFAULT_HUMAN_TRAIN_ID;
  marklin::TrainId ghostChaserTrainId = DEFAULT_CHASER_TRAIN_ID;
  marklin::TrainId ghostAmbusherTrainId = DEFAULT_AMBUSHER_TRAIN_ID;

  std::bitset<DOT_SENSOR_COUNT> activeDots{};
  uint32_t score = 0;

  marklin::TrackId currentTrackId = 0;
  std::array<PacmanTrainStateEntry, marklin::MAX_TRAIN_ID + 1> trainById{};
  std::array<bool, marklin::MAX_TRAIN_ID + 1> trainPresent{};

  marklin::TrackNodeId lastHumanNodeForChaser = INVALID_TRACK_NODE_ID;
  marklin::TrackNodeId lastAmbusherTargetNode = INVALID_TRACK_NODE_ID;
  marklin::TrackNodeId lastHumanSensorForScoring = INVALID_TRACK_NODE_ID;

  bool hasSnapshot = false;
  bool isGameOver = false;
  bool isGameWon = false;
};

const PacmanTrainStateEntry* getTrainState(const GameState& state, marklin::TrainId trainId) {
  if (!marklin::isValidTrainId(trainId)) {
    return nullptr;
  }
  if (!state.trainPresent[trainId]) {
    return nullptr;
  }
  return &state.trainById[trainId];
}

void setTrainSpeed(int trainTrackTid, marklin::TrainId trainId, marklin::SpeedLevel speedLevel) {
  notify(trainTrackTid, TrainTrackMsg{.type = TrainTrackMsgType::SetSpeedCmd,
                                      .setSpeedCmd{.trainId = trainId, .speedLevel = speedLevel}});
}

void sendPacmanDotsToUI(int uiTid, const GameState& state) {
  notify(uiTid, UIMsg{.type = UIMsgType::RedrawPacmanDots,
                      .pacmanDots{.activeMask = static_cast<uint64_t>(state.activeDots.to_ullong())}});
}

void stopAllActiveTrains(int trainTrackTid, const GameState& state) {
  for (size_t idx = 1; idx < state.trainPresent.size(); ++idx) {
    if (!state.trainPresent[idx]) {
      continue;
    }
    setTrainSpeed(trainTrackTid, static_cast<marklin::TrainId>(idx), 0);
  }
}

void sendGotoCommand(int trainTrackTid, marklin::TrainId trainId, marklin::SpeedLevel speedLevel,
                     const char* nodeName) {
  TrainTrackMsg msg{.type = TrainTrackMsgType::GotoCmd, .gotoCmd{}};
  msg.gotoCmd.trainId = trainId;
  msg.gotoCmd.speedLevel = speedLevel;
  msg.gotoCmd.offsetMm = 0;
  strncpy(msg.gotoCmd.location, nodeName, sizeof(msg.gotoCmd.location));
  notify(trainTrackTid, msg);
}

marklin::Distance estimateSeparationUm(marklin::TrainTrackState& trackState, const PacmanTrainStateEntry& lhs,
                                       const PacmanTrainStateEntry& rhs) {
  constexpr marklin::Distance INF = std::numeric_limits<marklin::Distance>::max() / 4;
  if (!lhs.isTracked || !rhs.isTracked) {
    return INF;
  }
  if (lhs.estimatedNodeId == INVALID_TRACK_NODE_ID || rhs.estimatedNodeId == INVALID_TRACK_NODE_ID) {
    return INF;
  }
  if (lhs.estimatedNodeId == rhs.estimatedNodeId) {
    return absDist(lhs.estimatedNodeOffset, rhs.estimatedNodeOffset);
  }
  const marklin::Distance lhsToRhs = shortestPathDistance(trackState, lhs.estimatedNodeId, rhs.estimatedNodeId);
  const marklin::Distance rhsToLhs = shortestPathDistance(trackState, rhs.estimatedNodeId, lhs.estimatedNodeId);
  return kit::min(lhsToRhs, rhsToLhs);
}

bool maybeConsumeDot(GameState& state, int uiTid) {
  const PacmanTrainStateEntry* human = getTrainState(state, state.humanTrainId);
  if (human == nullptr || !human->isTracked || human->lastSensorId == INVALID_TRACK_NODE_ID) {
    return false;
  }
  if (human->lastSensorId == state.lastHumanSensorForScoring) {
    return false;
  }

  state.lastHumanSensorForScoring = human->lastSensorId;
  if (human->lastSensorId / 2 >= DOT_SENSOR_COUNT) {
    return false;
  }

  const size_t sensorIdx = static_cast<size_t>(human->lastSensorId / 2);
  if (state.activeDots.test(sensorIdx)) {
    state.activeDots.reset(sensorIdx);
    ++state.score;
    notifyStatusToUI(uiTid, "Pacman ate dot %u. Score: %u", human->lastSensorId, state.score);
    return true;
  }
  return false;
}

void maybeTriggerWin(GameState& state, int trainTrackTid, int uiTid) {
  if (state.isGameOver || state.isGameWon) {
    return;
  }
  if (!state.activeDots.none()) {
    return;
  }

  state.isGameWon = true;
  stopAllActiveTrains(trainTrackTid, state);
  notifyStatusToUI(uiTid, "Pacman WIN! Final score: %u", state.score);
}

void maybeTriggerLoss(GameState& state, int trainTrackTid, int uiTid, marklin::TrainTrackState& trackState) {
  if (state.isGameOver || state.isGameWon) {
    return;
  }

  const PacmanTrainStateEntry* human = getTrainState(state, state.humanTrainId);
  const PacmanTrainStateEntry* ghost1 = getTrainState(state, state.ghostChaserTrainId);
  const PacmanTrainStateEntry* ghost2 = getTrainState(state, state.ghostAmbusherTrainId);
  if (human == nullptr) {
    return;
  }

  marklin::Distance minDistance = std::numeric_limits<marklin::Distance>::max();
  if (ghost1 != nullptr) {
    marklin::Distance d = estimateSeparationUm(trackState, *human, *ghost1);
    if (d < minDistance) {
      minDistance = d;
    }
  }
  if (ghost2 != nullptr) {
    marklin::Distance d = estimateSeparationUm(trackState, *human, *ghost2);
    if (d < minDistance) {
      minDistance = d;
    }
  }

  if (minDistance <= GHOST_CATCH_DISTANCE_UM) {
    state.isGameOver = true;
    stopAllActiveTrains(trainTrackTid, state);
    notifyStatusToUI(uiTid, "Pacman GAME OVER! Score: %u", state.score);
  }
}

void maybeRouteGhosts(GameState& state, int trainTrackTid, marklin::TrainTrackState& trackState) {
  if (state.isGameOver || state.isGameWon) {
    return;
  }

  const PacmanTrainStateEntry* human = getTrainState(state, state.humanTrainId);
  if (human == nullptr || !human->isTracked || human->estimatedNodeId == INVALID_TRACK_NODE_ID) {
    return;
  }

  marklin::TrackNode& humanNode = trackState.getTrackNodeById(human->estimatedNodeId);

  if (state.lastHumanNodeForChaser != human->estimatedNodeId) {
    sendGotoCommand(trainTrackTid, state.ghostChaserTrainId, GHOST_SPEED_LEVEL, humanNode.name);
    state.lastHumanNodeForChaser = human->estimatedNodeId;
  }

  marklin::TrackNode* ambushTarget =
      projectLookAheadNode(trackState, human->estimatedNodeId, human->direction, AMBUSH_LOOKAHEAD_NODES);
  if (ambushTarget == nullptr) {
    ambushTarget = &humanNode;
  }

  if (state.lastAmbusherTargetNode != ambushTarget->id) {
    sendGotoCommand(trainTrackTid, state.ghostAmbusherTrainId, GHOST_SPEED_LEVEL, ambushTarget->name);
    state.lastAmbusherTargetNode = ambushTarget->id;
  }
}

void updateSnapshot(GameState& state, const PacmanMsg& msg, marklin::TrainTrackState& trackState) {
  for (bool& present : state.trainPresent) {
    present = false;
  }

  state.currentTrackId = msg.gameStateUpdate.trackId;
  if (marklin::isValidTrack(state.currentTrackId)) {
    trackState.setCurrentTrack(state.currentTrackId);
  }

  const unsigned count = msg.gameStateUpdate.count;
  for (unsigned i = 0; i < count && i < marklin::NUM_TRAIN_IN_LAB; ++i) {
    const PacmanTrainStateEntry& entry = msg.gameStateUpdate.entries[i];
    if (!marklin::isValidTrainId(entry.trainId)) {
      continue;
    }
    state.trainById[entry.trainId] = entry;
    state.trainPresent[entry.trainId] = true;
  }
  state.hasSnapshot = true;
}

void pacmanTickNotifierTask() {
  int clockTid = ::WhoIs(clock_server::CLOCK_SERVER_NAME);
  if (clockTid < 0) {
    logError("failed to find clock server");
  }
  int serverTid = ::MyParentTid();
  for (;;) {
    int ticks = ::Delay(clockTid, PACMAN_TICK_PERIOD_TICKS);
    if (ticks < 0) {
      continue;
    }

    PacmanMsg msg{.type = PacmanMsgType::TimerTick, .time{static_cast<uint32_t>(ticks)}};
    notify(serverTid, msg);
  }
}

} // namespace

void pacmanServerTask() {
  if (::RegisterAs(PACMAN_SERVER_NAME) < 0) {
    logError("failed to register");
  }

  int clockTid = ::WhoIs(clock_server::CLOCK_SERVER_NAME);
  if (clockTid < 0) {
    logError("failed to find clock server");
  }
  int trainTrackTid = ::WhoIs(TRAIN_TRACK_SERVER_NAME);
  if (trainTrackTid < 0) {
    logError("failed to find train track server");
  }
  int uiTid = ::WhoIs(UI_VIEW_SERVER_NAME);
  if (uiTid < 0) {
    logError("failed to find UI view server");
  }

  if (::Create(1, pacmanTickNotifierTask) < 0) {
    logError("failed to create pacman timer notifier");
  }

  GameState state{};
  state.activeDots.set();
  marklin::TrainTrackState trackState{};
  notifyStatusToUI(uiTid, "Pacman server online. Human=%u, Chaser=%u, Ambusher=%u", state.humanTrainId,
                   state.ghostChaserTrainId, state.ghostAmbusherTrainId);
  sendPacmanDotsToUI(uiTid, state);

  for (;;) {
    PacmanMsg msg{};
    int senderTid = -1;
    ::Receive(&senderTid, reinterpret_cast<char*>(&msg), sizeof(msg));
    ::Reply(senderTid, "", 0);

    switch (msg.type) {
    case PacmanMsgType::TimerTick: {
      if (!state.hasSnapshot) {
        break;
      }
      maybeRouteGhosts(state, trainTrackTid, trackState);
      maybeTriggerLoss(state, trainTrackTid, uiTid, trackState);
      break;
    }
    case PacmanMsgType::GameStateUpdate: {
      updateSnapshot(state, msg, trackState);
      if (maybeConsumeDot(state, uiTid)) {
        sendPacmanDotsToUI(uiTid, state);
      }
      maybeTriggerWin(state, trainTrackTid, uiTid);
      break;
    }
    default:
      logError("Pacman server got an invalid message type");
      break;
    }
  }
}
} // namespace k4
