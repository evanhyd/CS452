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
  TrackNodeId destination = NO_TRACK_NODE;
  Distance offset = 0;
  Distance distance = 0;
  RingBuffer<Node, MAX_PATH_NODES> nodes{};
};

class PathFindingSystem {
public:
  enum class State { IDLING, MOVING, YIELDING, ARRIVING };

private:
  std::array<RingBuffer<TrainId, MAX_TRAIN_ID>, NUM_RESERVATION_NODES> nodeOwners{}; // FIFO reservation order.
  std::array<TrainId, NUM_RESERVATION_NODES> destination{};
  std::array<TrainPath, MAX_TRAIN_ID> paths{};
  std::array<State, MAX_TRAIN_ID> pathingStates{};

  void reserve(TrackNodeId nodeId, TrainId trainId) { nodeOwners[nodeId / 2].pushBack(trainId); }

  void unreserve(TrackNodeId nodeId, TrainId trainId) {
    KIT_ASSERT(!nodeOwners[nodeId / 2].empty(), "no reservation");
    KIT_ASSERT(nodeOwners[nodeId / 2].popFront() == trainId, "unreserved by the wrong train");
  }

  bool isPassable(TrackNodeId nodeId, TrainId trainId) {
    return destination[nodeId / 2] == NO_TRAIN || destination[nodeId / 2] == trainId;
  }

  bool canEnter(TrackNodeId nodeId, TrainId trainId) {
    KIT_ASSERT(!nodeOwners[nodeId / 2].empty(), "no reservation");
    return nodeOwners[nodeId / 2].front() == trainId;
  }

  Distance getEdgeWeight(const TrackEdge& edge) {
    static constexpr Distance PENALTY_PER_WAITER = 500'000; // 50 cm
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

  // Unreserve and remove past nodes.
  void popPastNodes(TrainId trainId, TrackNode& currentLocation) {
    auto& nodes = paths[trainId].nodes;

    bool inPath = kit::contains_if(nodes.begin(), nodes.end(),
                                   [&](const TrainPath::Node& node) { return node.srce->id == currentLocation.id; }) ||
                  paths[trainId].destination == currentLocation.id;
    if (!inPath) {
      // Estimated node was too ahead and got corrected back to a past node.
      // Ignore it.
      return;
    }

    while (!nodes.empty()) {
      auto& node = nodes.front();
      if (node.srce->id == currentLocation.id) {
        break;
      }
      unreserve(node.srce->id, trainId);
      paths[trainId].distance -= node.srce->edges[node.direction].dist;
      nodes.popFront();
    }
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
  State updateState(TrainId trainId, TrackNode& currentLocation, Distance currentOffset, Speed currentSpeed,
                    Distance& outEnterableDistance, const Callback& updateSwitch) {

    outEnterableDistance = 0;
    popPastNodes(trainId, currentLocation);

    switch (pathingStates[trainId]) {
    case State::IDLING: {
      break;
    }
    case State::MOVING: {
      // Calculate the reserved distance.
      size_t notEnterable = paths[trainId].nodes.size();
      for (auto& node : paths[trainId].nodes) {
        if (canEnter(node.srce->id, trainId)) {
          if (node.srce->type == TrackNode::Type::Branch) {
            updateSwitch(node.srce->num, node.direction == Straight ? SwitchState::Straight : SwitchState::Curved);
          }
          outEnterableDistance += node.srce->edges[node.direction].dist;
          --notEnterable;
        } else {
          break;
        }
      }
      outEnterableDistance -= currentOffset;

      // Only consider the offset if it can enter the destination node.
      Distance stoppingDistance = getStoppingDistance(currentSpeed);
      bool canArrive = (notEnterable == 0 && canEnter(paths[trainId].destination, trainId));
      if (canArrive) {
        outEnterableDistance += paths[trainId].offset;
      }

      if (canArrive && outEnterableDistance <= stoppingDistance) {
        pathingStates[trainId] = State::ARRIVING;
        break;
      }

      if (!canArrive && outEnterableDistance <= stoppingDistance) {
        pathingStates[trainId] = State::YIELDING;
        break;
      }

      break;
    }
    case State::YIELDING: {
      // Calculate the reserved distance.
      size_t notEnterable = paths[trainId].nodes.size();
      for (auto& node : paths[trainId].nodes) {
        if (canEnter(node.srce->id, trainId)) {
          if (node.srce->type == TrackNode::Type::Branch) {
            updateSwitch(node.srce->num, node.direction == Straight ? SwitchState::Straight : SwitchState::Curved);
          }
          outEnterableDistance += node.srce->edges[node.direction].dist;
          --notEnterable;
        } else {
          break;
        }
      }
      outEnterableDistance -= currentOffset;

      // YIELDING should not transit into ARRIVING.
      // Otherwise the train's speed level will remain 0.
      bool canArrive = (notEnterable == 0 && canEnter(paths[trainId].destination, trainId));
      if (canArrive) {
        pathingStates[trainId] = State::MOVING;
        break;
      }

      Distance stoppingDistance = getStoppingDistance(currentSpeed);
      if (outEnterableDistance > stoppingDistance) {
        pathingStates[trainId] = State::MOVING;
        break;
      }

      break;
    }
    case State::ARRIVING: {
      if (currentLocation.id == paths[trainId].destination && currentOffset >= paths[trainId].offset) {
        unreserve(currentLocation.id, trainId);
        pathingStates[trainId] = State::IDLING;
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
      //   auto it = kit::find(destination.begin(), destination.end(), trainId);
      //   if (it != destination.end()) {
      //     *it = NO_TRAIN;
      //   }
      if (TrackNodeId oldDest = paths[trainId].destination; oldDest != NO_TRACK_NODE) {
        destination[oldDest / 2] = NO_TRAIN;
      }
      destination[dest / 2] = trainId;
      paths[trainId].destination = dest;
      paths[trainId].offset = offset;
      paths[trainId].distance = 0;
      pathingStates[trainId] = State::MOVING;

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
