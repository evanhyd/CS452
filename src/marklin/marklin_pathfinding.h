#pragma once
#include "marklin/marklin_def.h"
#include "marklin/marklin_train_track.h"
#include "marklin_measured_data.h"
#include "util/debug.h"
#include "util/kit_algorithm.h"
#include "util/ring_buffer.h"
#include "util/static_priority_queue.h"
#include <cassert>
#include <numeric>
#include <source_location>

namespace marklin {

constexpr TrackNode* getNextTrackNode(TrainTrackState& state, TrackNode& srce, Distance& outDistance) {
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

constexpr TrackNode* getNextSensor(TrainTrackState& state, TrackNode& srce, Distance& outDistance) {
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

// Get the distance from sensor 1 to sensor 2. Order matters.
constexpr Distance getDistanceBetweenSensor(TrackId trackId, const TrackNode& sensor1, const TrackNode& sensor2) {
  if (trackId == 0) {
    return TRACK_A_SENSOR_DISTANCE[sensor1.id][sensor2.id];
  } else {
    return TRACK_B_SENSOR_DISTANCE[sensor1.id][sensor2.id];
  }
}

static constexpr size_t NUM_RESERVATION_NODES = NUM_TRACK_NODES / 2;
static constexpr size_t MAX_PATH_NODES = NUM_TRACK_NODES;

struct TrainPath {
  struct Node {
    TrackNode* srce;
    TrackDirection direction;
  };
  TrackNode* destination;
  Distance offset = 0;
  RingBuffer<Node, MAX_PATH_NODES> nodes{};
};

class PathFindingSystem {
private:
  static constexpr Distance YIELDING_MARGIN = 300'000;            // 30 cm
  static constexpr Distance TRESPASSING_BACKOFF_MARGIN = 200'000; // 20 cm
  static constexpr Distance STOPPING_MARGIN = 300'000;            // 30 cm

  std::array<RingBuffer<TrainId, MAX_TRAIN_ID>, NUM_RESERVATION_NODES> nodeOwners{}; // FIFO reservation order.
  std::array<TrainId, NUM_RESERVATION_NODES> blockers{};
  std::array<TrainPath, MAX_TRAIN_ID> paths{};
  std::array<PathingState, MAX_TRAIN_ID> pathingStates{};

  bool isEmergencyReroutingProtocolActivated = false;
  int trespassingCount = 0;

  void reserve(TrackNodeId nodeId, TrainId trainId) {
    size_t id = nodeId / 2;
    if (58 <= id && id <= 61) {
      nodeOwners[58].pushBack(trainId);
      nodeOwners[59].pushBack(trainId);
      nodeOwners[60].pushBack(trainId);
      nodeOwners[61].pushBack(trainId);
    } else {
      nodeOwners[id].pushBack(trainId);
    }
  }

  void unreserve(TrackNodeId nodeId, TrainId trainId, std::source_location loc = std::source_location::current()) {
    size_t id = nodeId / 2;
    if (58 <= id && id <= 61) {
      KIT_ASSERT(!nodeOwners[58].empty(), "no reservation", loc);
      KIT_ASSERT(nodeOwners[58].popFront() == trainId, "58 unreserved by the wrong train", loc);
      KIT_ASSERT(!nodeOwners[59].empty(), "no reservation", loc);
      KIT_ASSERT(nodeOwners[59].popFront() == trainId, "59 unreserved by the wrong train", loc);
      KIT_ASSERT(!nodeOwners[60].empty(), "no reservation", loc);
      KIT_ASSERT(nodeOwners[60].popFront() == trainId, "60 unreserved by the wrong train", loc);
      KIT_ASSERT(!nodeOwners[61].empty(), "no reservation", loc);
      KIT_ASSERT(nodeOwners[61].popFront() == trainId, "61 unreserved by the wrong train", loc);
    } else {
      KIT_ASSERT(!nodeOwners[id].empty(), "no reservation", loc);
      KIT_ASSERT(nodeOwners[id].popFront() == trainId, "unreserved by the wrong train", loc);
    }
  }

  bool canEnter(TrackNodeId nodeId, TrainId trainId) {
    return !nodeOwners[nodeId / 2].empty() && nodeOwners[nodeId / 2].front() == trainId;
  }

  bool isPassable(TrackNodeId nodeId, TrainId trainId) {
    TrainId owner = getBlockerOwner(nodeId);
    return owner == NO_TRAIN || owner == trainId;
  }

  void setBlocker(TrackNodeId nodeId, TrainId trainId) {
    for (TrainId& owner : blockers) {
      if (owner == trainId) {
        owner = NO_TRAIN;
        break;
      }
    }
    blockers[nodeId / 2] = trainId;
  }

  TrainId getBlockerOwner(TrackNodeId nodeId) { return blockers[nodeId / 2]; }

  Distance getEdgeWeight(const TrackEdge& edge) {
    static constexpr Distance PENALTY_PER_WAITER = 3'000'000; // 3 m
    return edge.dist + Distance(nodeOwners[edge.dest->id / 2].size()) * PENALTY_PER_WAITER;
  }

  TrackDirection getAdjacentDirection(const TrackNode& n1, const TrackNode& n2) {
    if (n1.edges[Straight].dest == &n2) {
      return Straight;
    }
    if (n1.edges[Curved].dest == &n2) {
      return Curved;
    }
    logError("n1 and n2 are not adjacent");
  }

  bool isAdjacent(const TrackNode& n1, const TrackNode& n2) {
    return (n1.edges[Straight].dest == &n2) || (n1.edges[Curved].dest == &n2);
  }

  // Unreserve and remove past nodes up to but not include nodeId.
  void popPastNodes(TrainId trainId, TrackNodeId nodeId) {
    auto& nodes = paths[trainId].nodes;
    if (!kit::contains_if(nodes.begin(), nodes.end(), [&](auto& node) { return node.srce->id == nodeId; })) {
      return;
    }

    while (!nodes.empty()) {
      const auto& node = nodes.front();
      if (node.srce->id == nodeId) {
        break;
      }
      unreserve(node.srce->id, trainId);
      nodes.popFront();
    }
  }

  bool isTrespassing(TrainId trainId, const Train& train) {
    const PathingState state = pathingStates[trainId];
    if (state != PathingState::Moving && state != PathingState::Yielding) {
      return false;
    }

    // Last sensor trespassing.
    auto& nodes = paths[trainId].nodes;
    bool inPath = kit::contains_if(nodes.begin(), nodes.end(),
                                   [&](auto& node) { return node.srce->id == train.kinematics.lastSensor->id; });
    if (inPath && !canEnter(train.kinematics.lastSensor->id, trainId)) {
      return true;
    }

    // Estimated node trespassing.
    auto it = kit::find_if(nodes.begin(), nodes.end(),
                           [&](auto& node) { return node.srce->id == train.kinematics.estimatedNode->id; });
    if (state == PathingState::Yielding) {
      // Check if yield in time.
      Distance margin = -train.kinematics.estimatedNodeOffset;
      Distance lastDist = 0;
      for (; it != nodes.end(); ++it) {
        if (!canEnter(it->srce->id, trainId)) {
          break;
        }
        margin += lastDist;
        lastDist = it->srce->edges[it->direction].dist;
      }

      if (margin < TRESPASSING_BACKOFF_MARGIN) {
        return true;
      }
    } else if (state == PathingState::Moving) {
      // Check if estimated position is not part of the path.
      // Usually caused by switch failure.
      if (it == nodes.end()) {
        return true;
      }

      // Check if train runs too fast and trespassed.
      // Usually caused by not stopping in time.
      if (!canEnter(it->srce->id, trainId)) {
        return true;
      }
    }
    return false;
  }

public:
  TrainId getReserver(TrackNodeId nodeId) const {
    if (nodeOwners[nodeId / 2].empty()) {
      return NO_TRAIN;
    }
    return nodeOwners[nodeId / 2].front();
  }

  const TrainPath& getTrainPath(TrainId trainId) const { return paths[trainId]; }

  PathingState updateState(TrainId trainId, Train& train, const auto& updateSwitch,
                           [[maybe_unused]] const auto& printer) {

    if (isTrespassing(trainId, train)) {
      isEmergencyReroutingProtocolActivated = true;
    }
    if (isEmergencyReroutingProtocolActivated) {
      pathingStates[trainId] = PathingState::Trespassing;
    }

    const PathingState oldState = pathingStates[trainId];
    switch (oldState) {
    case PathingState::Idling: {
      break;
    }
    case PathingState::Yielding:
    case PathingState::Moving: {
      popPastNodes(trainId, train.kinematics.lastSensor->id);

      // Calculate the reserved distance.
      Distance enterableDistance = -train.kinematics.estimatedNodeOffset;
      Distance distToNextNode = 0;
      size_t notEnterable = paths[trainId].nodes.size();
      bool seenEstimated = false;
      int seenNodeCount = 0;

      for (auto& node : paths[trainId].nodes) {
        if (canEnter(node.srce->id, trainId)) {
          // Switch switches close to us to avoid flipipng the train.
          if (seenEstimated && seenNodeCount <= 5) {
            if (node.srce->type == TrackNode::Type::Branch) {
              updateSwitch(node.srce->num, node.direction == Straight ? SwitchState::Straight : SwitchState::Curved);
            }
          }

          // Only accumulate the enterable distance after the estimated node.
          if (!seenEstimated) {
            seenEstimated = (node.srce->id == train.kinematics.estimatedNode->id);
          } else {
            enterableDistance += distToNextNode;
            ++seenNodeCount;
          }
          distToNextNode = node.srce->edges[node.direction].dist;
          --notEnterable;
        } else {
          break;
        }
      }

      // Add the destination offset if it can enter the destination node.
      const bool isRoadClear = (notEnterable == 0);
      if (isRoadClear) {
        Distance stoppingDistance = train.kinematics.offlineSpeed ==
                                            convertSpeedLevelToOfflineSpeed(trainId, train.kinematics.offlineSpeedLevel)
                                        ? getStoppingDistanceFromLevel(trainId, train.kinematics.offlineSpeedLevel)
                                        : getStoppingDistance(trainId, train.kinematics.estimatedSpeed);

        enterableDistance += paths[trainId].offset;
        // Check arrival.
        Distance margin = [&]() {
          TrackNode* dest = paths[trainId].destination;
          if (dest->type != TrackNode::Type::Sensor) {
            return 0;
          }
          static constexpr Distance PROXIMITY_THRESHOLD = 300'000; // 30 cm
          static constexpr auto isDanger = [](const TrackNode* node) {
            return node->type == TrackNode::Type::Exit || node->type == TrackNode::Type::Branch ||
                   node->type == TrackNode::Type::Merge;
          };
          static constexpr auto hasDangerAhead = [](const TrackNode* node) {
            return node->edges[Straight].dist <= PROXIMITY_THRESHOLD && isDanger(node->edges[Straight].dest);
          };
          bool shouldUndershoot = hasDangerAhead(dest);
          bool shouldOvershoot = hasDangerAhead(dest->reverse);
          return (int(shouldOvershoot) - int(shouldUndershoot)) * STOPPING_MARGIN;
        }();
        if (enterableDistance + margin <= stoppingDistance) {
          pathingStates[trainId] = PathingState::Arriving;
        } else {
          pathingStates[trainId] = PathingState::Moving;
        }
      } else {
        Distance stoppingDistance =
            getStoppingDistanceFromLevel(trainId, train.navigation.findingPathTask.maxSpeedLevel);

        // Check yielding.
        Distance yieldingDistance = enterableDistance - YIELDING_MARGIN;
        if (yieldingDistance <= stoppingDistance) {
          pathingStates[trainId] = PathingState::Yielding;
        } else {
          pathingStates[trainId] = PathingState::Moving;
        }
      }
      break;
    }
    case PathingState::Arriving: {
      // Waiting for the train to stop.
      if (train.kinematics.isStationary()) {
        pathingStates[trainId] = PathingState::Idling;
      }
      break;
    }
    case PathingState::Trespassing: {
      // Resets the system once.
      if (isEmergencyReroutingProtocolActivated) {
        bool hasRoutedTrain = kit::contains_if(pathingStates.begin(), pathingStates.end(), [](auto s) {
          return s == PathingState::Moving || s == PathingState::Yielding;
        });
        if (!hasRoutedTrain) {
          isEmergencyReroutingProtocolActivated = false;
          nodeOwners = {};
          paths = {};
          trespassingCount = kit::count(pathingStates.begin(), pathingStates.end(), PathingState::Trespassing);
        }
      }

      if (train.kinematics.isStationary()) {
        --trespassingCount;
        setBlocker(train.kinematics.estimatedNode->id, trainId);
        pathingStates[trainId] = PathingState::Resuming;
      }
      break;
    }
    case PathingState::Resuming: {
      // Wait to synchronize all the trespassing trains.
      if (trespassingCount == 0) {

        // Tick 1: Request train to find a new path.
        if (!train.navigation.findingPathTask.reqeusetToResume) {
          train.navigation.findingPathTask.reqeusetToResume = true;
          break;
        }

        // Tick 2: Check the result.
        train.navigation.findingPathTask.reqeusetToResume = false;
        if (!train.navigation.findingPathTask.isResumed) {
          pathingStates[trainId] = PathingState::Idling;
        }
      }
      break;
    }
    }

    return pathingStates[trainId];
  }

  bool planPath(TrainTrackState& ttState, TrainId trainId, TrackNodeId srce, TrackNodeId dest, Distance offset) {
    // Track min distances.
    static constexpr Distance INFINITY = std::numeric_limits<Distance>::max();
    std::array<Distance, NUM_TRACK_NODES> minDistances;
    kit::fill(minDistances.begin(), minDistances.end(), INFINITY);
    minDistances[srce] = 0;

    // Track parents.
    static constexpr TrackNodeId NO_PARENT = NUM_TRACK_NODES;
    std::array<TrackNodeId, NUM_TRACK_NODES> parents;
    kit::fill(parents.begin(), parents.end(), NO_PARENT);

    struct Edge {
      Distance dist;
      TrackNodeId id;
      auto operator<=>(const Edge&) const = default;
    };
    StaticPriorityQueue<Edge, NUM_TRACK_NODES * 2> queue;
    queue.push({0, srce});

    TrackNodeId destRev = ttState.getTrackNodeById(dest).reverse->id;

    bool isReachable = false;
    while (!queue.empty()) {
      Edge u = queue.top();
      queue.pop();

      if (u.id == dest || u.id == destRev) {
        dest = u.id;
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
        TrackNodeId v = edge.dest->id;

        // Ignore permanent stationary.
        if (!isPassable(v, trainId)) {
          continue;
        }

        Distance dist = u.dist + getEdgeWeight(edge);
        if (dist < minDistances[v]) {
          minDistances[v] = dist;
          parents[v] = u.id;
          queue.push({dist, v});
        }
      }
    }

    // Extract the path.
    if (isReachable) {
      setBlocker(dest, trainId);
      paths[trainId].destination = &ttState.getTrackNodeById(dest);
      paths[trainId].offset = offset;

      // Remove the old path when planning new path instead of at arrival.
      // Otherwise other train can reserve this train's position and cause collision.
      for (const auto& node : paths[trainId].nodes) {
        unreserve(node.srce->id, trainId);
      }
      paths[trainId].nodes.clear();

      paths[trainId].nodes.pushFront({paths[trainId].destination, Straight});
      reserve(dest, trainId);
      for (TrackNodeId currId = dest; parents[currId] != NO_PARENT;) {
        TrackNodeId parentId = parents[currId];
        TrackNode& currNode = ttState.getTrackNodeById(currId);
        TrackNode& parentNode = ttState.getTrackNodeById(parentId);

        TrackDirection dir = getAdjacentDirection(parentNode, currNode);
        paths[trainId].nodes.pushFront({&parentNode, dir});
        reserve(parentId, trainId);
        currId = parentId;
      }

      pathingStates[trainId] = PathingState::Moving;
    }

    return isReachable;
  }
}; // namespace marklin
} // namespace marklin
