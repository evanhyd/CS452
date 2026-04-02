#pragma once
#include "marklin/marklin_def.h"
#include "marklin/marklin_train_track.h"
#include "marklin_measured_data.h"
#include "util/debug.h"
#include "util/kit_algorithm.h"
#include "util/ring_buffer.h"
#include "util/static_priority_queue.h"
#include <numeric>

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
  Distance distance = 0;
  RingBuffer<Node, MAX_PATH_NODES> nodes{};
};

class PathFindingSystem {
private:
  std::array<RingBuffer<TrainId, MAX_TRAIN_ID>, NUM_RESERVATION_NODES> nodeOwners{}; // FIFO reservation order.
  std::array<TrainId, NUM_RESERVATION_NODES> occupied{};
  std::array<TrainPath, MAX_TRAIN_ID> paths{};
  std::array<PathingState, MAX_TRAIN_ID> pathingStates{};

  void reserve(TrackNodeId nodeId, TrainId trainId) {
    size_t id = nodeId / 2;
    nodeOwners[id].pushBack(trainId);
    if (58 <= id && id <= 61) {
      // Lock the central switch pair.
      nodeOwners[id ^ 1].pushBack(trainId);
    }
  }

  void unreserve(TrackNodeId nodeId, TrainId trainId) {
    size_t id = nodeId / 2;
    KIT_ASSERT(!nodeOwners[id].empty(), "no reservation");
    KIT_ASSERT(nodeOwners[id].popFront() == trainId, "unreserved by the wrong train");

    if (58 <= id && id <= 61) {
      // Unlock the central switch pair.
      id ^= 1;
      KIT_ASSERT(!nodeOwners[id].empty(), "no reservation");
      KIT_ASSERT(nodeOwners[id].popFront() == trainId, "unreserved by the wrong train");
    }
  }

  bool isPassable(TrackNodeId nodeId, TrainId trainId) {
    return occupied[nodeId / 2] == NO_TRAIN || occupied[nodeId / 2] == trainId;
  }

  bool canEnter(TrackNodeId nodeId, TrainId trainId) {
    KIT_ASSERT(!nodeOwners[nodeId / 2].empty(), "no reservation");
    return nodeOwners[nodeId / 2].front() == trainId;
  }

  Distance getEdgeWeight(const TrackEdge& edge) {
    static constexpr Distance PENALTY_PER_WAITER = 3000'000; // 3 m
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

  // Unreserve and remove past nodes.
  // Return false is the train is trespassing.
  bool popPastNodes(TrainId trainId, TrackNodeId currentLocationId) {
    auto& nodes = paths[trainId].nodes;
    if (currentLocationId != paths[trainId].destination->id &&
        !kit::contains_if(nodes.begin(), nodes.end(), [&](auto& node) { return node.srce->id == currentLocationId; })) {
      // Planned path based on estimated position, which can be overshoot compared to the last sensor.
      // But we pop past nodes based on the last sensor, which is not part of the path.
      return true;
    }

    while (!nodes.empty()) {
      auto& node = nodes.front();
      if (node.srce->id == currentLocationId) {
        break;
      }
      if (!canEnter(node.srce->id, trainId)) {
        // Trespassing.
        return false;
      }
      paths[trainId].distance -= node.srce->edges[node.direction].dist;
      unreserve(node.srce->id, trainId);
      nodes.popFront();
    }
    return true;
  }

public:
  TrainId getReserver(TrackNodeId nodeId) const {
    if (nodeOwners[nodeId / 2].empty()) {
      return NO_TRAIN;
    }
    return nodeOwners[nodeId / 2].front();
  }

  const TrainPath& getTrainPath(TrainId trainId) const { return paths[trainId]; }

  template <typename Callback>
  PathingState updateState(TrainId trainId, const Train& train, Distance& outEnterableDistance,
                           const Callback& updateSwitch) {
    static constexpr Distance SAFETY_MARGIN = 200'000;   // 20 cm
    static constexpr Distance STOPPING_MARGIN = 200'000; // 20 cm
    outEnterableDistance = 0;
    bool isTresspassing = !popPastNodes(trainId, train.kinematics.lastSensor->id);

    switch (pathingStates[trainId]) {
    case PathingState::Idling: {
      break;
    }
    case PathingState::Moving: {
      // Detected trespassing. Stop
      if (isTresspassing) {
        pathingStates[trainId] = PathingState::Yielding;
        break;
      }

      // Calculate the reserved distance.
      outEnterableDistance = -train.kinematics.estimatedNodeOffset;
      Distance distToNextNode = 0;
      size_t notEnterable = paths[trainId].nodes.size();
      bool seenEstimated = false;

      for (auto& node : paths[trainId].nodes) {
        if (canEnter(node.srce->id, trainId)) {
          // Switch switches close to us to avoid flipipng the train.
          if (node.srce->type == TrackNode::Type::Branch) {
            updateSwitch(node.srce->num, node.direction == Straight ? SwitchState::Straight : SwitchState::Curved);
          }

          // Only accumulate the enterable distance after the estimated node.
          if (!seenEstimated) {
            seenEstimated = (node.srce->id == train.kinematics.estimatedNode->id);
          } else {
            outEnterableDistance += distToNextNode;
          }
          distToNextNode = node.srce->edges[node.direction].dist;
          --notEnterable;
        } else {
          if (!seenEstimated) {
            // The train's estimated position in a node that it does not have access to.
          }
          break;
        }
      }

      // Add the destination offset if it can enter the destination node.
      bool canArrive = (notEnterable == 0 && canEnter(paths[trainId].destination->id, trainId));
      if (canArrive) {
        outEnterableDistance += distToNextNode + paths[trainId].offset;
      }

      Distance stoppingDistance =
          train.kinematics.offlineSpeed == convertSpeedLevelToOfflineSpeed(trainId, train.kinematics.offlineSpeedLevel)
              ? getStoppingDistanceFromLevel(trainId, train.kinematics.offlineSpeedLevel)
              : getStoppingDistance(trainId, train.kinematics.estimatedSpeed);

      Distance overshoot = [&]() {
        TrackNode* dest = paths[trainId].destination;
        if (dest->type != TrackNode::Type::Sensor) {
          return -1;
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
        return (int(shouldUndershoot) - int(shouldOvershoot)) * STOPPING_MARGIN;
      }();

      if (canArrive && outEnterableDistance <= stoppingDistance + overshoot) {
        pathingStates[trainId] = PathingState::Arriving;
        break;
      }

      if (!canArrive &&
          (outEnterableDistance - getTrainHeadLength(train.kinematics.direction) - SAFETY_MARGIN) <= stoppingDistance) {
        pathingStates[trainId] = PathingState::Yielding;
        break;
      }

      break;
    }
    case PathingState::Yielding: {
      // Detected trespassing. Stop
      if (isTresspassing) {
        break;
      }

      // Calculate the reserved distance.
      outEnterableDistance = -train.kinematics.estimatedNodeOffset;
      Distance distToNextNode = 0;
      size_t notEnterable = paths[trainId].nodes.size();
      bool seenEstimated = false;

      for (auto& node : paths[trainId].nodes) {
        if (canEnter(node.srce->id, trainId)) {
          // Only accumulate the enterable distance after the estimated node.
          if (!seenEstimated) {
            seenEstimated = (node.srce->id == train.kinematics.estimatedNode->id);
          } else {
            outEnterableDistance += distToNextNode;
          }
          distToNextNode = node.srce->edges[node.direction].dist;
          --notEnterable;
        } else {
          break;
        }
      }

      // YIELDING should not transit into ARRIVING.
      // Otherwise the train's speed level will remain 0.
      bool canArrive = (notEnterable == 0 && canEnter(paths[trainId].destination->id, trainId));
      if (canArrive) {
        pathingStates[trainId] = PathingState::Moving;
        break;
      }

      Distance stoppingDistance = getStoppingDistanceFromLevel(trainId, train.navigation.findingPathTask.maxSpeedLevel);
      if ((outEnterableDistance - getTrainHeadLength(train.kinematics.direction) - SAFETY_MARGIN) > stoppingDistance) {
        pathingStates[trainId] = PathingState::Moving;
        break;
      }

      break;
    }
    case PathingState::Arriving: {
      // Must use estimated speed instead of estimated position.
      // Otherwise if the train stops a tiny bit before the destination, then it will stuck in ARRIVING.
      if (train.kinematics.estimatedSpeed == 0) {
        for (const auto& node : paths[trainId].nodes) {
          unreserve(node.srce->id, trainId);
        }
        paths[trainId].nodes.clear();
        unreserve(paths[trainId].destination->id, trainId);
        pathingStates[trainId] = PathingState::Idling;
      }
      break;
    }
    default:
      logError("unhandled pathing state");
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
      // Move the destination.
      if (paths[trainId].destination) {
        occupied[paths[trainId].destination->id / 2] = NO_TRAIN;
      }

      occupied[dest / 2] = trainId;
      paths[trainId].destination = &ttState.getTrackNodeById(dest);
      paths[trainId].offset = offset;
      paths[trainId].distance = 0;
      pathingStates[trainId] = PathingState::Moving;

      for (TrackNode* curr = &ttState.getTrackNodeById(dest);;) {
        reserve(curr->id, trainId);
        TrackNodeId p = parents[curr->id];
        if (p == NO_PARENT) {
          break;
        }
        TrackNode* parent = &ttState.getTrackNodeById(p);
        TrackDirection direction = getAdjacentDirection(*parent, *curr);
        paths[trainId].distance += parent->edges[direction].dist;
        paths[trainId].nodes.pushFront({parent, direction});
        curr = parent;
      }
    }

    return isReachable;
  }
};
} // namespace marklin
