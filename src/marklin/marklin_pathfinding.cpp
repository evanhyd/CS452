#include "marklin/marklin_pathfinding.h"
#include "marklin/marklin_message.h"
#include "marklin/marklin_train_track.h"
#include "util/debug.h"
#include "util/kit_algorithm.h"
#include "util/ring_buffer.h"
#include "util/static_priority_queue.h"
#include <limits>

namespace marklin {

namespace {
constexpr std::array STOPPING_DISTANCE = {0,      21000,  36000,  56000,  95000,  131000,  198000, 264000,
                                          378000, 427000, 579000, 766000, 980000, 1184000, 1503000};

constexpr std::array OFFLINE_SPEED = {0, 80, 250, 470, 670, 930, 1390, 1860, 2320, 2830, 3440, 4090, 4730, 5440, 6150};

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

static_assert([] {
  std::array<bool, NUM_TRACK_NODES> isLoopSensorNode{};
  for (auto id : loopSensorNodeIds) {
    if (isLoopSensorNode[id]) {
      return false;
    }
    isLoopSensorNode[id] = true;
  }
  return true;
}());

} // namespace

Distance getStoppingDistanceForLevel(SpeedLevel speedLevel) { return STOPPING_DISTANCE[speedLevel]; }

// https://www.desmos.com/calculator/uktgt5mydm
Distance getStoppingDistance(Speed speed) {
  return speed == 0 ? 0 : Distance(int64_t(speed) * speed * 253 / 10000 + speed * 815 / 10 + 18740);
}

Speed convertSpeedLevelToOfflineSpeed(SpeedLevel speedLevel) { return OFFLINE_SPEED[speedLevel]; }

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

PathFindingSystem::PathFindingSystem() { reset(); }

void PathFindingSystem::reset() {
  locks_ = {};
  ownedNodes_ = {};
}

// Run dijkstra to find the shortest AND viable path between srce and dest.
bool PathFindingSystem::dijkstra(TrainTrackState& ttState, TrainId trainId, TrackNodeId dest,
                                 Distance& outTotalPathDist) {
  // Already at the destination.
  Train& train = ttState.getTrain(trainId);
  KIT_ASSERT(!train.nav.path.empty(), "old path not released");
  train.nav.pathDistance = 0;
  outTotalPathDist = 0;
  TrackNodeId srce = train.kin.lastKnownNode->id;
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
      KIT_ASSERT(edge.dest, "edge destination is null");

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
    for (TrackNodeId u = dest; u != srce; u = parents[u]) {
      TrackNode& node = ttState.getTrackNodeById(u);
      TrackNode& parent = ttState.getTrackNodeById(parents[u]);
      train.nav.path.pushFront(&node);
      outTotalPathDist += getAdjacentDistance(parent, node);
    }
  }

  return isReachable;
}

bool PathFindingSystem::planPath(TrainTrackState& ttState, TrainId trainId, TrackNodeId dest,
                                 Distance& outTotalPathDist) {
  // Calculate the shortest path to the destination.
  // Populates train.path and locks the nodes from the destination back to the loop exit.
  bool isReachable = dijkstra(ttState, trainId, dest, outTotalPathDist);
  if (!isReachable) {
    return false;
  }

  Train& train = ttState.getTrain(trainId);
  train.nav.pathDistance = outTotalPathDist;
  train.nav.estimatedPathDistance = outTotalPathDist;
  return isReachable;

  //   TrackNode* startNode = train.kinematics.lastKnownNode;
  //   // Trace one full lap around the captive loop.
  //   // Because Locating set the switches, getNextTrackNode will naturally follow the loop.
  //   std::array<TrackNode*, NUM_TRACK_NODES> loopArr;
  //   size_t loopSize = 0;
  //   TrackNode* curr = startNode;
  //   do {
  //     Distance dist = 0;
  //     curr = getNextTrackNode(ttState, *curr, dist);
  //     if (!curr) {
  //       logError("Loop trace hit a dead end!");
  //     }
  //     if (loopSize >= NUM_TRACK_NODES) {
  //       logError("Loop trace exceeded max size!");
  //     }
  //     loopArr[loopSize++] = curr;
  //   } while (curr != startNode);

  //   // Prepend the loop nodes to the train's path
  //   for (int i = static_cast<int>(loopSize) - 1; i >= 0; --i) {
  //     TrackNode* node = loopArr[size_t(i)];
  //     TrackNode* parent = (i == 0) ? startNode : loopArr[size_t(i - 1)];
  //     train.nav.path.pushFront(node);
  //     outTotalPathDist += getAdjacentDistance(*parent, *node);
  //     train.nav.pathDistance = outTotalPathDist;
  //     train.nav.estimatedPathDistance = outTotalPathDist;
  //   }
}

void PathFindingSystem::lock(TrackNodeId nodeId, TrainId trainId, std::source_location loc) {
  {
    auto& lock = locks_[nodeId];
    KIT_ASSERT(lock.id == NO_TRAIN || lock.id == trainId, "track node is locked by other train", loc);
    lock.id = trainId;
    ++lock.time;
    ownedNodes_[trainId].pushBack(nodeId);
  }

  {
    nodeId ^= 1;
    auto& lock = locks_[nodeId];
    KIT_ASSERT(lock.id == NO_TRAIN || lock.id == trainId, "track node is locked by other train", loc);
    lock.id = trainId;
    ++lock.time;
    ownedNodes_[trainId].pushBack(nodeId);
  }
}

void PathFindingSystem::unlock(TrackNodeId nodeId, TrainId trainId, std::source_location loc) {
  {
    auto& lock = locks_[nodeId];
    KIT_ASSERT(lock.id == trainId, "track node is unlocked by non-owner", loc);
    --lock.time;
    if (lock.time == 0) {
      lock.id = NO_TRAIN;
    }

    auto& ownedNode = ownedNodes_[trainId];
    auto it = kit::find(ownedNode.begin(), ownedNode.end(), nodeId);
    KIT_ASSERT(it != ownedNode.end(), "missing track node");
    *it = ownedNode.front();
    ownedNode.popFront();
  }

  {
    nodeId ^= 1;
    auto& lock = locks_[nodeId];
    KIT_ASSERT(lock.id == trainId, "track node is unlocked by non-owner", loc);
    --lock.time;
    if (lock.time == 0) {
      lock.id = NO_TRAIN;
    }

    auto& ownedNode = ownedNodes_[trainId];
    auto it = kit::find(ownedNode.begin(), ownedNode.end(), nodeId);
    KIT_ASSERT(it != ownedNode.end(), "missing track node");
    *it = ownedNode.front();
    ownedNode.popFront();
  }
}

bool PathFindingSystem::canLock(TrackNodeId nodeId, TrainId trainId) const {
  auto& lock1 = locks_[nodeId];
  auto& lock2 = locks_[nodeId ^ 1];
  return (lock1.id == trainId || lock1.id == NO_TRAIN) && (lock2.id == trainId || lock2.id == NO_TRAIN);
}

TrainId PathFindingSystem::getOwner(TrackNodeId nodeId) const { return locks_[nodeId].id; }

const RingBuffer<TrackNodeId, MAX_NODE_PER_TRAIN>& PathFindingSystem::getOwned(TrainId trainId) const {
  return ownedNodes_[trainId];
}

} // namespace marklin
