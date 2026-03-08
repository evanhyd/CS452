#pragma once
#include "marklin_train_track.h"

namespace marklin {
// Get the next track node by following the track at the current configuration.
// Return nullptr if not found.
TrackNode* getNextTrackNode(TrainTrackState& state, TrackNode& srce, Distance& outDistance);

// Get the next sensor track node by following the track at the current configuration.
// Return nullptr if not found.
TrackNode* getNextSensor(TrainTrackState& state, TrackNode& srce, Distance& outDistance);

// Plan a path for train to go to the destination.
// The train must acquire the ownership of the path.
// Return true if a viable path is acquired.
bool planPath(TrainTrackState& ttState, TrainId trainId, TrackNodeId dest);

// Estimate and update the train's motion data.
void calibrateTrainFromTrackNode(TrainTrackState& ttState, TrackNodeId triggeredTrackId, uint32_t currentTicks);

} // namespace marklin
