#include "marklin/marklin_pathfinding.h"
#include "marklin/marklin_train_track.h"
#include "util/debug.h"
#include "util/kit_algorithm.h"
#include "util/ring_buffer.h"
#include "util/static_priority_queue.h"
#include <limits>

namespace marklin {

namespace {
constexpr int32_t EWMA_DENOMINATOR = 4;
constexpr std::array STOPPING_DISTANCE = {0,      21000,  36000,  56000,  95000,  131000,  198000, 264000,
                                          378000, 427000, 579000, 766000, 980000, 1184000, 1523000};

constexpr marklin::TrackNodeId loopSensorNodeIds[] = {
    marklin::sensorToTrackNodeId({'A', 3}),  marklin::sensorToTrackNodeId({'A', 4}),
    marklin::sensorToTrackNodeId({'B', 15}), marklin::sensorToTrackNodeId({'B', 16}),
    marklin::sensorToTrackNodeId({'C', 9}),  marklin::sensorToTrackNodeId({'C', 10}),
    marklin::sensorToTrackNodeId({'B', 1}),  marklin::sensorToTrackNodeId({'B', 2}),
    marklin::sensorToTrackNodeId({'D', 13}), marklin::sensorToTrackNodeId({'D', 14}),
    marklin::sensorToTrackNodeId({'E', 13}), marklin::sensorToTrackNodeId({'E', 14}),
    marklin::sensorToTrackNodeId({'E', 9}),  marklin::sensorToTrackNodeId({'E', 10}),
    marklin::sensorToTrackNodeId({'D', 5}),  marklin::sensorToTrackNodeId({'D', 6}),
    marklin::sensorToTrackNodeId({'E', 5}),  marklin::sensorToTrackNodeId({'E', 6}),
    marklin::sensorToTrackNodeId({'D', 3}),  marklin::sensorToTrackNodeId({'D', 4}),
    marklin::sensorToTrackNodeId({'B', 5}),  marklin::sensorToTrackNodeId({'B', 6}),
    marklin::sensorToTrackNodeId({'C', 11}), marklin::sensorToTrackNodeId({'C', 12}),
};

constexpr marklin::TrackNodeId loopClockwiseSensorNodeIds[] = {
    marklin::sensorToTrackNodeId({'A', 3}),  marklin::sensorToTrackNodeId({'C', 11}),
    marklin::sensorToTrackNodeId({'B', 5}),  marklin::sensorToTrackNodeId({'D', 3}),
    marklin::sensorToTrackNodeId({'E', 5}),  marklin::sensorToTrackNodeId({'D', 6}),
    marklin::sensorToTrackNodeId({'E', 10}), marklin::sensorToTrackNodeId({'E', 13}),
    marklin::sensorToTrackNodeId({'D', 13}), marklin::sensorToTrackNodeId({'B', 2}),
    marklin::sensorToTrackNodeId({'C', 9}),  marklin::sensorToTrackNodeId({'B', 15}),
};

constexpr marklin::TrackNodeId loopCounterClockwiseSensorNodeIds[] = {
    marklin::sensorToTrackNodeId({'A', 4}),  marklin::sensorToTrackNodeId({'B', 16}),
    marklin::sensorToTrackNodeId({'C', 10}), marklin::sensorToTrackNodeId({'B', 1}),
    marklin::sensorToTrackNodeId({'D', 14}), marklin::sensorToTrackNodeId({'E', 14}),
    marklin::sensorToTrackNodeId({'E', 9}),  marklin::sensorToTrackNodeId({'D', 5}),
    marklin::sensorToTrackNodeId({'E', 6}),  marklin::sensorToTrackNodeId({'D', 4}),
    marklin::sensorToTrackNodeId({'B', 6}),  marklin::sensorToTrackNodeId({'C', 12}),
};

static_assert([] {
  std::array<bool, NUM_TRACK_NODES> isLoopSensorNode{};
  for (auto id : loopSensorNodeIds) {
    if (isLoopSensorNode[id]) {
      return false;
    }
    isLoopSensorNode[id] = true;
  }
  for (auto id : loopClockwiseSensorNodeIds) {
    if (!isLoopSensorNode[id]) {
      return false;
    }
    isLoopSensorNode[id] = false;
  }
  for (auto id : loopCounterClockwiseSensorNodeIds) {
    if (!isLoopSensorNode[id]) {
      return false;
    }
    isLoopSensorNode[id] = false;
  }
  return true;
}());

// The node is NOT passable if:
// A different train sitting on the node on either direction
bool isPassable(TrackNode& node, TrainId id) {
  if (node.lock.hasOwner() && node.lock.owner() != id) {
    return false;
  }
  if (node.reverse->lock.hasOwner() && node.reverse->lock.owner() != id) {
    return false;
  }
  return true;
}

// Run dijkstra to find the shortest AND viable path between srce and dest.
bool dijkstra(TrainTrackState& ttState, TrainId trainId, TrackNodeId dest, Distance& outTotalPathDist) {
  // Already at the destination.
  Train& train = ttState.getTrain(trainId);
  train.path.clear();
  outTotalPathDist = 0;
  TrackNodeId srce = train.lastVisitedNode->id;
  if (srce == dest) {
    return true;
  }

  // Track min distances.
  static constexpr Distance INFINITY = std::numeric_limits<Distance>::max();
  std::array<Distance, NUM_TRACK_NODES> minDistances;
  kit::fill(minDistances.begin(), minDistances.end(), INFINITY);
  minDistances[srce] = 0;

  // Track parents.
  static constexpr TrackNodeId NO_PARENT = NUM_TRACK_NODES;
  std::array<TrackNodeId, NUM_TRACK_NODES> parents;
  kit::fill(parents.begin(), parents.end(), NO_PARENT);

  // Dijkstrak go brrrrrr.
  struct Edge {
    Distance dist;
    TrackNodeId id;
    auto operator<=>(const Edge&) const = default;
  };
  StaticPriorityQueue<Edge, NUM_TRACK_NODES * 2> queue;
  queue.push({0, srce});

  bool isReachable = false;
  while (!queue.empty()) {
    Edge u = queue.top();
    queue.pop();

    if (u.id == dest) {
      isReachable = true;
      break;
    }

    if (u.dist > minDistances[u.id]) {
      continue;
    }

    // Calculate number of edges.
    const TrackNode& uNode = ttState.getTrackNodeById(u.id);
    size_t numEdges = [&] -> size_t {
      switch (uNode.type) {
      case TrackNode::Type::Branch:
        return 2;
      case TrackNode::Type::Sensor:
      case TrackNode::Type::Merge:
      case TrackNode::Type::Enter:
        return 1;
      default:
        return 0;
      }
    }();

    // Explore neighbors.
    for (size_t i = 0; i < numEdges; ++i) {
      const TrackEdge& edge = uNode.edges[i];
      if (!isPassable(*edge.dest, trainId)) {
        continue;
      }

      // Update distance and parent.
      TrackNodeId v = edge.dest->id;
      Distance dist = u.dist + edge.dist;
      if (dist < minDistances[v]) {
        minDistances[v] = dist;
        parents[v] = u.id;
        queue.push({dist, v});
      }
    }
  }

  // Extract the path.
  if (isReachable) {
    TrackDirection lastDirection = TrackDirection::Straight;
    for (TrackNodeId u = dest; u != srce; u = parents[u]) {
      TrackNode& node = ttState.getTrackNodeById(u);
      TrackNode& parent = ttState.getTrackNodeById(parents[u]);
      if (!node.lock.tryAcquire(trainId, lastDirection)) {
        logError("lock failed even though checked passable");
      }
      lastDirection = getAdjacentDirection(parent, node);
      train.path.pushFront(&node);
      outTotalPathDist += getAdjacentDistance(parent, node);
    }
  }

  return isReachable;
}
} // namespace

Distance getStoppingDistance(SpeedLevel speedLevel) { return STOPPING_DISTANCE[speedLevel]; }

bool lockAllLoopSensorNodes(TrainTrackState& ttState, TrainId trainId) {
  for (auto id : loopSensorNodeIds) {
    if (!ttState.getTrackNodeById(id).lock.canAcquire(trainId)) {
      return false;
    }
  }
  for (auto id : loopSensorNodeIds) {
    if (!ttState.getTrackNodeById(id).lock.tryAcquire(trainId)) {
      logError("unexpected lock failure");
    }
  }
  return true;
}

void unlockAllLoopSensorNodes(TrainTrackState& ttState, TrainId trainId) {
  for (auto id : loopSensorNodeIds) {
    ttState.getTrackNodeById(id).lock.release(trainId);
  }
}

TrackNode* getNextTrackNode(TrainTrackState& state, TrackNode& srce, Distance& outDistance) {
  if (srce.type == TrackNode::Type::Exit) {
    return nullptr;
  }
  size_t dir = TrackDirection::Straight;
  if (srce.type == TrackNode::Type::Branch) {
    auto sw = state.getSwitchState(srce.num);
    dir = (sw == SwitchState::Straight ? TrackDirection::Straight : TrackDirection::Curved);
  }
  outDistance = srce.edges[dir].dist;
  return srce.edges[dir].dest;
}

TrackNode* getNextSensor(TrainTrackState& state, TrackNode& srce, Distance& outDistance) {
  outDistance = 0;
  TrackNode* node = &srce;
  while (true) {
    Distance dist = 0;
    node = getNextTrackNode(state, *node, dist);
    outDistance += dist;
    if (!node || node->type == TrackNode::Type::Sensor) {
      break;
    }
  }
  return node;
}

Distance getAdjacentDistance(TrackNode& n1, TrackNode& n2) {
  switch (n1.type) {
  case TrackNode::Type::Branch:
    if (TrackNode* u = n1.edges[0].dest; u && u->id == n2.id) {
      return n1.edges[0].dist;
    }
    if (TrackNode* u = n1.edges[1].dest; u && u->id == n2.id) {
      return n1.edges[1].dist;
    }
    logError("n1 and n2 are not adjacent");
  case TrackNode::Type::Sensor:
  case TrackNode::Type::Merge:
  case TrackNode::Type::Enter:
    if (TrackNode* u = n1.edges[0].dest; u && u->id == n2.id) {
      return n1.edges[0].dist;
    }
    logError("n1 and n2 are not adjacent");
  default:
    logError("n1 and n2 are not adjacent");
  }
}

TrackDirection getAdjacentDirection(TrackNode& n1, TrackNode& n2) {
  switch (n1.type) {
  case TrackNode::Type::Branch:
    if (TrackNode* u = n1.edges[0].dest; u && u->id == n2.id) {
      return TrackDirection::Straight;
    }
    if (TrackNode* u = n1.edges[1].dest; u && u->id == n2.id) {
      return TrackDirection::Curved;
    }
    logError("n1 and n2 are not adjacent");
  case TrackNode::Type::Sensor:
  case TrackNode::Type::Merge:
  case TrackNode::Type::Enter:
    if (TrackNode* u = n1.edges[0].dest; u && u->id == n2.id) {
      return TrackDirection::Straight;
    }
    logError("n1 and n2 are not adjacent");
  default:
    logError("n1 and n2 are not adjacent");
  }
}

bool planPath(TrainTrackState& ttState, TrainId trainId, TrackNodeId dest, Distance& outTotalPathDist) {
  // Calculate the shortest path to the destination.
  // Populates train.path and locks the nodes from the destination back to the loop exit.
  bool isReachable = dijkstra(ttState, trainId, dest, outTotalPathDist);
  if (!isReachable) {
    return false;
  }

  Train& train = ttState.getTrain(trainId);
  TrackNode* startNode = train.lastVisitedNode;

  // Trace one full lap around the captive loop.
  // Because Locating set the switches, getNextTrackNode will naturally follow the loop.
  std::array<TrackNode*, NUM_TRACK_NODES> loopArr;
  size_t loopSize = 0;
  TrackNode* curr = startNode;
  do {
    Distance dist = 0;
    curr = getNextTrackNode(ttState, *curr, dist);
    if (!curr) {
      logError("Loop trace hit a dead end!");
      return false;
    }
    loopArr[loopSize++] = curr;
  } while (curr != startNode);

  // Prepend the loop nodes to the train's path
  for (int i = static_cast<int>(loopSize) - 1; i >= 0; --i) {
    TrackNode* node = loopArr[size_t(i)];
    TrackNode* parent = (i == 0) ? startNode : loopArr[size_t(i - 1)];

    // The direction 'node' must take to reach the NEXT node in the train's path.
    TrackDirection dir = TrackDirection::Straight;
    if (!train.path.empty()) {
      dir = getAdjacentDirection(*node, *train.path.front());
    }

    if (!node->lock.tryAcquire(trainId, dir)) {
      logError("Loop node lock failed!");
    }
    train.path.pushFront(node);
    outTotalPathDist += getAdjacentDistance(*parent, *node);
  }

  // Lock startNode
  TrackDirection enterDir = TrackDirection::Straight;
  if (!train.path.empty()) {
    enterDir = getAdjacentDirection(*startNode, *train.path.front());
  }
  if (!startNode->lock.tryAcquire(trainId, enterDir)) {
    logError("Start node lock failed!");
  }

  return isReachable;
}

void calibrateTrain(Train& train, TrackNode& triggeredNode, uint32_t currentTicks, Distance dS) {
  // Update the train estimated speed.
  uint32_t dT = kit::max(1u, currentTicks - train.lastSpeedUpdateTicks);

  // Update the time weighted speed.
  if (dS != 0) {
    Speed v = dS / Speed(dT);
    if (train.estimatedSpeed == 0) {
      train.estimatedSpeed = v;
    } else {
      train.estimatedSpeed = (train.estimatedSpeed * (EWMA_DENOMINATOR - 1) + v) / EWMA_DENOMINATOR;
    }
    train.stateMachine.pathing.pathDistance -= dS;
  }

  // Update the train last triggered sensor node.
  train.lastSpeedUpdateTicks = currentTicks;
  train.lastVisitedNode = &triggeredNode;
  train.estimatedOffsetFromLast = 0;
  train.estimatedNode = &triggeredNode;
  train.estimatedOffsetFromEstimatedNode = 0;
  train.estimatedPathDistance = train.stateMachine.pathing.pathDistance;
}

} // namespace marklin
