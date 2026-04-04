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
  std::array<RingBuffer<TrainId, MAX_TRAIN_ID>, NUM_RESERVATION_NODES> nodeOwners{}; // FIFO reservation order.
  std::array<TrainId, NUM_RESERVATION_NODES> finalDestination{};
  std::array<TrainPath, MAX_TRAIN_ID> paths{};
  std::array<PathingState, MAX_TRAIN_ID> pathingStates{};

  void reserve(TrackNodeId nodeId, TrainId trainId) {
    size_t id = nodeId / 2;
    nodeOwners[id].pushBack(trainId);
    if (58 <= id && id <= 61) {
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

  bool canEnter(TrackNodeId nodeId, TrainId trainId) {
    return !nodeOwners[nodeId / 2].empty() && nodeOwners[nodeId / 2].front() == trainId;
  }

  bool isPassable(TrackNodeId nodeId, TrainId trainId) {
    return finalDestination[nodeId / 2] == NO_TRAIN || finalDestination[nodeId / 2] == trainId;
  }

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
  // Return false if the train is trespassing.
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
    auto& nodes = paths[trainId].nodes;

    // Last sensor trespassing.
    bool inPath = kit::contains_if(nodes.begin(), nodes.end(), [&](auto& node) {
      return node.srce->id == train.kinematics.lastSensor->id ||
             node.srce->reverse->id == train.kinematics.lastSensor->id;
    });
    if (inPath && !canEnter(train.kinematics.lastSensor->id, trainId)) {
      return true;
    }

    // Estimated node trespassing.
    {
      auto it = kit::find_if(nodes.begin(), nodes.end(), [&](auto& node) {
        return node.srce->id == train.kinematics.lastSensor->id ||
               node.srce->reverse->id == train.kinematics.lastSensor->id;
      });

      if (it != nodes.end()) {
        // Check if the train is too ahead.
        if (!canEnter(it->srce->id, trainId)) {
          return true;
        }

        // Check if respect yield safety margin.
        if (pathingStates[trainId] == PathingState::Yielding) {
          if (++it; it != nodes.end()) {
            if (!canEnter(it->srce->id, trainId)) {
              return true;
            }
          }
          if (++it; it != nodes.end()) {
            if (!canEnter(it->srce->id, trainId)) {
              return true;
            }
          }
        }
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

  PathingState updateState(TrainId trainId, const Train& train, const auto& updateSwitch, const auto& printer) {
    bool isTooAhead = isTrespassing(trainId, train);

    const PathingState oldState = pathingStates[trainId];
    switch (oldState) {
    case PathingState::Idling: {
      break;
    }
    case PathingState::Moving:
    case PathingState::Yielding: {
      if (isTooAhead) {
        pathingStates[trainId] = PathingState::Trespassing;
        break;
      }
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
        static constexpr Distance STOPPING_MARGIN = 300'000; // 30 cm
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
        static constexpr Distance YIELDING_MARGIN = 300'000; // 30 cm
        Distance stoppingDistance =
            getStoppingDistanceFromLevel(trainId, train.navigation.findingPathTask.maxSpeedLevel);

        // Check yielding.
        Distance yieldingDistance =
            enterableDistance - getTrainHeadLength(train.kinematics.direction) - YIELDING_MARGIN;
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
      if (!isTooAhead) {
        pathingStates[trainId] = PathingState::Resuming;
      }
      break;
    }
    case PathingState::Resuming: {
      // Waiting for the train to restore the direciton.
      if (train.navigation.findingPathTask.isResumed) {
        pathingStates[trainId] = PathingState::Yielding;
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

        // Avoid flipipng the train.
        // if (edge.dest->type == TrackNode::Type::Branch && (u.dist + edge.dist) <= 50'000) {
        //   continue;
        // }

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
        finalDestination[paths[trainId].destination->id / 2] = NO_TRAIN;
      }
      finalDestination[dest / 2] = trainId;

      // Update the path.
      paths[trainId].destination = &ttState.getTrackNodeById(dest);
      paths[trainId].offset = offset;

      decltype(paths[trainId].nodes) newPath{};
      newPath.pushFront({paths[trainId].destination, Straight});
      reserve(dest, trainId);
      for (TrackNodeId currId = dest; parents[currId] != NO_PARENT;) {
        TrackNodeId parentId = parents[currId];
        TrackNode& currNode = ttState.getTrackNodeById(currId);
        TrackNode& parentNode = ttState.getTrackNodeById(parentId);

        TrackDirection dir = getAdjacentDirection(parentNode, currNode);
        newPath.pushFront({&parentNode, dir});
        reserve(parentId, trainId);
        currId = parentId;
      }

      // Unlike the old design. We don't free up all the nodes upon arrival.
      // This makes the code more fault tolerant, as it reduces the chance of train crashing into each other.
      // When we append new path, there are two cases we must handle.
      // 1. The train continue moving forward. We need to remove the old invalid branches and duplicated nodes in the
      // old path.
      // 2. THe train reversed direction. We must unreserve all the previous path, because some other train might be
      // yielding for it.
      while (!paths[trainId].nodes.empty()) {
        auto node = paths[trainId].nodes.popBack();
        unreserve(node.srce->id, trainId);

        // The train continue moving forward.
        if (node.srce->id == srce) {
          break;
        }

        // The train reversed. Unreserve all paths to avoid deadlock.
        if (node.srce->reverse->id == srce) {
          while (!paths[trainId].nodes.empty()) {
            auto remainingNode = paths[trainId].nodes.popBack();
            unreserve(remainingNode.srce->id, trainId);
          }
          break;
        }
      }

      for (const auto& node : newPath) {
        paths[trainId].nodes.pushBack(node);
      }

      pathingStates[trainId] = PathingState::Moving;
    }

    return isReachable;
  }
};
} // namespace marklin
