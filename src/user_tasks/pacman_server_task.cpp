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
constexpr uint32_t GHOST_GOTO_DEBOUNCE_TICKS = 300;
constexpr uint32_t HUMAN_REVERSE_CMD_DEBOUNCE_TICKS = 50;
constexpr unsigned UPCOMING_SWITCH_LOOKAHEAD_STEPS = 12;

constexpr unsigned MAX_REGISTERED_GHOSTS = marklin::NUM_TRAIN_IN_LAB;

constexpr size_t DOT_SENSOR_COUNT = PACMAN_DOT_COUNT;

struct SwitchArrowMapping {
  marklin::SwitchId switchId;
  marklin::SwitchState leftState;
  marklin::SwitchState rightState;
};

constexpr std::array<SwitchArrowMapping, 22> SWITCH_ARROW_MAPPINGS = {{
    {1, marklin::SwitchState::Curved, marklin::SwitchState::Straight},
    {2, marklin::SwitchState::Curved, marklin::SwitchState::Straight},
    {3, marklin::SwitchState::Straight, marklin::SwitchState::Curved},
    {4, marklin::SwitchState::Straight, marklin::SwitchState::Curved},
    {5, marklin::SwitchState::Curved, marklin::SwitchState::Straight},
    {6, marklin::SwitchState::Straight, marklin::SwitchState::Curved},
    {7, marklin::SwitchState::Curved, marklin::SwitchState::Straight},
    {8, marklin::SwitchState::Straight, marklin::SwitchState::Curved},
    {9, marklin::SwitchState::Curved, marklin::SwitchState::Straight},
    {10, marklin::SwitchState::Curved, marklin::SwitchState::Straight},
    {11, marklin::SwitchState::Curved, marklin::SwitchState::Straight},
    {12, marklin::SwitchState::Curved, marklin::SwitchState::Straight},
    {13, marklin::SwitchState::Straight, marklin::SwitchState::Curved},
    {14, marklin::SwitchState::Straight, marklin::SwitchState::Curved},
    {15, marklin::SwitchState::Curved, marklin::SwitchState::Straight},
    {16, marklin::SwitchState::Curved, marklin::SwitchState::Straight},
    {17, marklin::SwitchState::Straight, marklin::SwitchState::Curved},
    {18, marklin::SwitchState::Straight, marklin::SwitchState::Curved},
    {153, marklin::SwitchState::Straight, marklin::SwitchState::Curved},
    {154, marklin::SwitchState::Curved, marklin::SwitchState::Straight},
    {155, marklin::SwitchState::Straight, marklin::SwitchState::Curved},
    {156, marklin::SwitchState::Curved, marklin::SwitchState::Straight},
}};

constexpr marklin::Distance absDist(marklin::Distance a, marklin::Distance b) { return a > b ? a - b : b - a; }

bool debounceElapsed(uint32_t lastTicks, uint32_t currentTicks, uint32_t minIntervalTicks) {
  return lastTicks == std::numeric_limits<uint32_t>::max() || currentTicks - lastTicks >= minIntervalTicks;
}

int signedHumanSpeedLevel(const PacmanTrainStateEntry& human) {
  int speed = static_cast<int>(human.offlineSpeedLevel);
  if (human.direction == marklin::TrainDirection::Backward) {
    speed = -speed;
  }
  return speed;
}

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
  marklin::TrainId humanTrainId = marklin::NO_TRAIN;
  marklin::TrainId ghostChaserTrainId = marklin::NO_TRAIN;
  marklin::TrainId ghostAmbusherTrainId = marklin::NO_TRAIN;

  std::array<marklin::TrainId, MAX_REGISTERED_GHOSTS> registeredGhosts{};
  std::array<marklin::SpeedLevel, marklin::MAX_TRAIN_ID + 1> ghostSpeedByTrain{};
  std::array<bool, marklin::MAX_TRAIN_ID + 1> ghostRegistered{};
  unsigned registeredGhostCount = 0;

  std::bitset<DOT_SENSOR_COUNT> activeDots{};
  uint32_t score = 0;

  marklin::TrackId currentTrackId = 0;
  uint32_t lastGameStateTicks = 0;
  std::array<PacmanTrainStateEntry, marklin::MAX_TRAIN_ID + 1> trainById{};
  std::array<bool, marklin::MAX_TRAIN_ID + 1> trainPresent{};

  marklin::TrackNodeId lastHumanNodeForChaser = INVALID_TRACK_NODE_ID;
  marklin::TrackNodeId lastAmbusherTargetNode = INVALID_TRACK_NODE_ID;
  marklin::TrackNodeId lastHumanSensorForScoring = INVALID_TRACK_NODE_ID;
  uint32_t lastChaserGotoTicks = std::numeric_limits<uint32_t>::max();
  uint32_t lastAmbusherGotoTicks = std::numeric_limits<uint32_t>::max();

  int humanDesiredSignedSpeedLevel = 0;
  bool humanDesiredSpeedInitialized = false;
  bool humanReversePending = false;
  marklin::TrainDirection humanLastDirection = marklin::TrainDirection::Forward;
  uint32_t lastHumanReverseCmdTicks = std::numeric_limits<uint32_t>::max();

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

bool isHumanRegistered(const GameState& state) { return marklin::isValidTrainId(state.humanTrainId); }

marklin::SpeedLevel ghostSpeedLevel(const GameState& state, marklin::TrainId ghostId) {
  if (!marklin::isValidTrainId(ghostId) || !state.ghostRegistered[ghostId]) {
    return GHOST_SPEED_LEVEL;
  }
  return state.ghostSpeedByTrain[ghostId];
}

void resetGhostRoutingTargets(GameState& state) {
  state.lastHumanNodeForChaser = INVALID_TRACK_NODE_ID;
  state.lastAmbusherTargetNode = INVALID_TRACK_NODE_ID;
  state.lastChaserGotoTicks = std::numeric_limits<uint32_t>::max();
  state.lastAmbusherGotoTicks = std::numeric_limits<uint32_t>::max();
}

void assignGhostRoles(GameState& state) {
  marklin::TrainId nextChaser = marklin::NO_TRAIN;
  marklin::TrainId nextAmbusher = marklin::NO_TRAIN;

  if (isHumanRegistered(state)) {
    if (state.registeredGhostCount >= 1) {
      nextChaser = state.registeredGhosts[0];
    }
    if (state.registeredGhostCount >= 2) {
      nextAmbusher = state.registeredGhosts[1];
    }
  }

  if (state.ghostChaserTrainId != nextChaser || state.ghostAmbusherTrainId != nextAmbusher) {
    state.ghostChaserTrainId = nextChaser;
    state.ghostAmbusherTrainId = nextAmbusher;
    resetGhostRoutingTargets(state);
  }
}

void unregisterGhostTrain(GameState& state, marklin::TrainId trainId) {
  if (!marklin::isValidTrainId(trainId) || !state.ghostRegistered[trainId]) {
    return;
  }

  size_t idx = state.registeredGhostCount;
  for (size_t i = 0; i < state.registeredGhostCount; ++i) {
    if (state.registeredGhosts[i] == trainId) {
      idx = i;
      break;
    }
  }

  if (idx < state.registeredGhostCount) {
    for (size_t i = idx + 1; i < state.registeredGhostCount; ++i) {
      state.registeredGhosts[i - 1] = state.registeredGhosts[i];
    }
    --state.registeredGhostCount;
  }

  state.ghostRegistered[trainId] = false;
  state.ghostSpeedByTrain[trainId] = 0;
}

void registerGhostTrain(GameState& state, marklin::TrainId trainId, marklin::SpeedLevel speedLevel) {
  if (!state.ghostRegistered[trainId]) {
    if (state.registeredGhostCount >= MAX_REGISTERED_GHOSTS) {
      return;
    }
    state.registeredGhosts[state.registeredGhostCount++] = trainId;
    state.ghostRegistered[trainId] = true;
  }

  state.ghostSpeedByTrain[trainId] = speedLevel;
  assignGhostRoles(state);
}

void registerHumanTrain(GameState& state, marklin::TrainId trainId) {
  if (state.ghostRegistered[trainId]) {
    unregisterGhostTrain(state, trainId);
  }

  state.humanTrainId = trainId;
  state.humanDesiredSpeedInitialized = false;
  state.humanReversePending = false;
  assignGhostRoles(state);
}

void setTrainSpeed(int trainTrackTid, marklin::TrainId trainId, marklin::SpeedLevel speedLevel) {
  notify(trainTrackTid, TrainTrackMsg{.type = TrainTrackMsgType::SetSpeedCmd,
                                      .setSpeedCmd{.trainId = trainId, .speedLevel = speedLevel}});
}

void reverseTrainDirection(int trainTrackTid, marklin::TrainId trainId) {
  notify(trainTrackTid, TrainTrackMsg{.type = TrainTrackMsgType::ReverseCmd, .reverseCmd{.trainId = trainId}});
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

void sendSwitchCommand(int trainTrackTid, marklin::SwitchId switchId, marklin::SwitchState state) {
  notify(trainTrackTid,
         TrainTrackMsg{.type = TrainTrackMsgType::SetSwitchCmd, .setSwitchCmd{.switchId = switchId, .state = state}});
}

marklin::SwitchState mappedSwitchState(marklin::SwitchId switchId, bool leftArrow) {
  for (const auto& mapping : SWITCH_ARROW_MAPPINGS) {
    if (mapping.switchId == switchId) {
      return leftArrow ? mapping.leftState : mapping.rightState;
    }
  }
  return leftArrow ? marklin::SwitchState::Curved : marklin::SwitchState::Straight;
}

marklin::SwitchId findUpcomingSwitch(const PacmanTrainStateEntry& human, marklin::TrainTrackState& trackState) {
  if (!human.isTracked || human.estimatedNodeId == INVALID_TRACK_NODE_ID) {
    return 0;
  }

  marklin::TrackNode* node = &trackState.getTrackNodeById(human.estimatedNodeId);
  if (human.direction == marklin::TrainDirection::Backward && node->reverse != nullptr) {
    node = node->reverse;
  }

  for (unsigned i = 0; i < UPCOMING_SWITCH_LOOKAHEAD_STEPS && node != nullptr; ++i) {
    if (node->type == marklin::TrackNode::Type::Branch && marklin::isValidSwitchId(node->num)) {
      return node->num;
    }

    marklin::Distance segmentDist = 0;
    node = marklin::getNextTrackNode(trackState, *node, segmentDist);
  }

  return 0;
}

void applyHumanDesiredSpeed(GameState& state, int trainTrackTid, int uiTid, bool currentBackward, uint32_t ticks) {
  const int desired = state.humanDesiredSignedSpeedLevel;
  const int desiredMagnitude = desired < 0 ? -desired : desired;
  const marklin::SpeedLevel desiredLevel = static_cast<marklin::SpeedLevel>(desiredMagnitude);
  const bool desiredBackward = desired < 0;

  setTrainSpeed(trainTrackTid, state.humanTrainId, desiredLevel);

  if (desired == 0) {
    state.humanReversePending = false;
    notifyStatusToUI(uiTid, "Pacman human speed -> 0");
    return;
  }

  if (currentBackward == desiredBackward) {
    state.humanReversePending = false;
    notifyStatusToUI(uiTid, "Pacman human speed -> %d", desired);
    return;
  }

  if (state.humanReversePending) {
    notifyStatusToUI(uiTid, "Pacman human speed -> %d (reversing)", desired);
    return;
  }

  if (!debounceElapsed(state.lastHumanReverseCmdTicks, ticks, HUMAN_REVERSE_CMD_DEBOUNCE_TICKS)) {
    notifyStatusToUI(uiTid, "Pacman human speed -> %d (reverse cooldown)", desired);
    return;
  }

  reverseTrainDirection(trainTrackTid, state.humanTrainId);
  state.humanReversePending = true;
  state.lastHumanReverseCmdTicks = ticks;
  notifyStatusToUI(uiTid, "Pacman human speed -> %d (reverse)", desired);
}

void handleHumanSpeedControl(GameState& state, int trainTrackTid, int uiTid, int delta) {
  if (!isHumanRegistered(state)) {
    notifyStatusToUI(uiTid, "Pacman control: set human train first (human <id>).");
    return;
  }

  const PacmanTrainStateEntry* human = getTrainState(state, state.humanTrainId);
  if (!state.humanDesiredSpeedInitialized) {
    state.humanDesiredSignedSpeedLevel = human ? signedHumanSpeedLevel(*human) : 0;
    state.humanDesiredSpeedInitialized = true;
  }

  int next = state.humanDesiredSignedSpeedLevel + delta;
  if (next < -static_cast<int>(marklin::MAX_SPEED_LEVEL)) {
    next = -static_cast<int>(marklin::MAX_SPEED_LEVEL);
  }
  if (next > static_cast<int>(marklin::MAX_SPEED_LEVEL)) {
    next = static_cast<int>(marklin::MAX_SPEED_LEVEL);
  }

  state.humanDesiredSignedSpeedLevel = next;
  applyHumanDesiredSpeed(state, trainTrackTid, uiTid, human && human->direction == marklin::TrainDirection::Backward,
                         state.lastGameStateTicks);
}

void handleHumanSwitchControl(GameState& state, int trainTrackTid, int uiTid, marklin::TrainTrackState& trackState,
                              bool leftArrow) {
  const PacmanTrainStateEntry* human = getTrainState(state, state.humanTrainId);
  if (human == nullptr) {
    notifyStatusToUI(uiTid, "Pacman control: no human train state yet.");
    return;
  }

  marklin::SwitchId switchId = findUpcomingSwitch(*human, trackState);
  if (!marklin::isValidSwitchId(switchId)) {
    notifyStatusToUI(uiTid, "Pacman control: no upcoming switch found.");
    return;
  }

  marklin::SwitchState target = mappedSwitchState(switchId, leftArrow);
  sendSwitchCommand(trainTrackTid, switchId, target);
  notifyStatusToUI(uiTid, "Pacman %s -> switch %u %c", leftArrow ? "left" : "right", switchId,
                   target == marklin::SwitchState::Straight ? 'S' : 'C');
}

void handleHumanControl(GameState& state, const PacmanMsg& msg, int trainTrackTid, int uiTid,
                        marklin::TrainTrackState& trackState) {
  switch (msg.humanControl.action) {
  case HumanControlAction::SpeedUp:
    handleHumanSpeedControl(state, trainTrackTid, uiTid, 1);
    break;
  case HumanControlAction::SpeedDown:
    handleHumanSpeedControl(state, trainTrackTid, uiTid, -1);
    break;
  case HumanControlAction::SwitchLeft:
    handleHumanSwitchControl(state, trainTrackTid, uiTid, trackState, true);
    break;
  case HumanControlAction::SwitchRight:
    handleHumanSwitchControl(state, trainTrackTid, uiTid, trackState, false);
    break;
  }
}

void notifyRoleAssignment(int uiTid, const GameState& state) {
  if (!isHumanRegistered(state)) {
    notifyStatusToUI(uiTid, "Pacman roles: no human configured.");
    return;
  }

  if (marklin::isValidTrainId(state.ghostChaserTrainId) && marklin::isValidTrainId(state.ghostAmbusherTrainId)) {
    notifyStatusToUI(uiTid, "Pacman roles: human=%u chaser=%u ambusher=%u", state.humanTrainId,
                     state.ghostChaserTrainId, state.ghostAmbusherTrainId);
  } else if (marklin::isValidTrainId(state.ghostChaserTrainId)) {
    notifyStatusToUI(uiTid, "Pacman roles: human=%u chaser=%u (need one more ghost for ambusher)", state.humanTrainId,
                     state.ghostChaserTrainId);
  } else {
    notifyStatusToUI(uiTid, "Pacman roles: human=%u (no ghosts registered)", state.humanTrainId);
  }
}

void handleRegisterGhost(GameState& state, const PacmanMsg& msg, int uiTid) {
  marklin::TrainId trainId = msg.registerGhost.trainId;
  marklin::SpeedLevel speedLevel = msg.registerGhost.speedLevel;

  if (!marklin::isValidTrainId(trainId) || !marklin::isValidSpeedLevel(speedLevel)) {
    notifyStatusToUI(uiTid, "Pacman ghost registration rejected.");
    return;
  }

  if (trainId == state.humanTrainId) {
    notifyStatusToUI(uiTid, "Pacman ghost registration rejected: train %u is human.", trainId);
    return;
  }

  bool wasRegistered = state.ghostRegistered[trainId];
  unsigned oldCount = state.registeredGhostCount;

  registerGhostTrain(state, trainId, speedLevel);

  if (!state.ghostRegistered[trainId]) {
    notifyStatusToUI(uiTid, "Pacman ghost registration full (max %u).", MAX_REGISTERED_GHOSTS);
    return;
  }

  if (wasRegistered) {
    notifyStatusToUI(uiTid, "Pacman ghost %u updated to speed %u.", trainId, speedLevel);
  } else {
    notifyStatusToUI(uiTid, "Pacman ghost %u registered at speed %u.", trainId, speedLevel);
  }

  if (state.registeredGhostCount != oldCount || wasRegistered) {
    notifyRoleAssignment(uiTid, state);
  }
}

void handleRegisterHuman(GameState& state, const PacmanMsg& msg, int uiTid) {
  marklin::TrainId trainId = msg.registerHuman.trainId;
  if (!marklin::isValidTrainId(trainId)) {
    notifyStatusToUI(uiTid, "Pacman human registration rejected.");
    return;
  }

  registerHumanTrain(state, trainId);
  notifyStatusToUI(uiTid, "Pacman human set to train %u.", trainId);
  notifyRoleAssignment(uiTid, state);
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

void maybeRouteGhosts(GameState& state, int trainTrackTid, marklin::TrainTrackState& trackState,
                      uint32_t currentTicks) {
  if (state.isGameOver || state.isGameWon) {
    return;
  }

  if (!isHumanRegistered(state)) {
    return;
  }

  const PacmanTrainStateEntry* human = getTrainState(state, state.humanTrainId);
  if (human == nullptr || !human->isTracked || human->estimatedNodeId == INVALID_TRACK_NODE_ID) {
    return;
  }

  marklin::TrackNode& humanNode = trackState.getTrackNodeById(human->estimatedNodeId);

  if (marklin::isValidTrainId(state.ghostChaserTrainId) && state.lastHumanNodeForChaser != human->estimatedNodeId &&
      debounceElapsed(state.lastChaserGotoTicks, currentTicks, GHOST_GOTO_DEBOUNCE_TICKS)) {
    sendGotoCommand(trainTrackTid, state.ghostChaserTrainId, ghostSpeedLevel(state, state.ghostChaserTrainId),
                    humanNode.name);
    state.lastHumanNodeForChaser = human->estimatedNodeId;
    state.lastChaserGotoTicks = currentTicks;
  }

  marklin::TrackNode* ambushTarget =
      projectLookAheadNode(trackState, human->estimatedNodeId, human->direction, AMBUSH_LOOKAHEAD_NODES);
  if (ambushTarget == nullptr) {
    ambushTarget = &humanNode;
  }

  if (marklin::isValidTrainId(state.ghostAmbusherTrainId) && state.lastAmbusherTargetNode != ambushTarget->id &&
      debounceElapsed(state.lastAmbusherGotoTicks, currentTicks, GHOST_GOTO_DEBOUNCE_TICKS)) {
    sendGotoCommand(trainTrackTid, state.ghostAmbusherTrainId, ghostSpeedLevel(state, state.ghostAmbusherTrainId),
                    ambushTarget->name);
    state.lastAmbusherTargetNode = ambushTarget->id;
    state.lastAmbusherGotoTicks = currentTicks;
  }
}

void updateSnapshot(GameState& state, const PacmanMsg& msg, marklin::TrainTrackState& trackState) {
  for (bool& present : state.trainPresent) {
    present = false;
  }

  state.currentTrackId = msg.gameStateUpdate.trackId;
  state.lastGameStateTicks = msg.gameStateUpdate.ticks;
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

  const PacmanTrainStateEntry* human = getTrainState(state, state.humanTrainId);
  if (human != nullptr) {
    if (!state.humanDesiredSpeedInitialized) {
      state.humanDesiredSignedSpeedLevel = signedHumanSpeedLevel(*human);
      state.humanDesiredSpeedInitialized = true;
    }

    if (state.humanReversePending && human->direction != state.humanLastDirection) {
      state.humanReversePending = false;
    }
    state.humanLastDirection = human->direction;
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
  notifyStatusToUI(uiTid, "Pacman server online. Use 'human <id>' and 'ghost <id> <speed>'.");
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
      maybeRouteGhosts(state, trainTrackTid, trackState, msg.time.ticks);
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
    case PacmanMsgType::HumanControl: {
      handleHumanControl(state, msg, trainTrackTid, uiTid, trackState);
      break;
    }
    case PacmanMsgType::RegisterGhost: {
      handleRegisterGhost(state, msg, uiTid);
      break;
    }
    case PacmanMsgType::RegisterHuman: {
      handleRegisterHuman(state, msg, uiTid);
      break;
    }
    default:
      logError("Pacman server got an invalid message type");
      break;
    }
  }
}
} // namespace k4
