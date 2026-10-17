/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Workspace
#include <iterator>
#include "graph_msf/core/TimeGraphKeyBuffer.h"
#include "graph_msf/interface/Terminal.h"

namespace graph_msf {

void TimeGraphKeyBuffer::addToBuffer(const double ts, const gtsam::Key& key) {
  if (verboseLevel_ >= 5) {
    std::cout << YELLOW_START << "GMsf-TimeKeyBuffer" << COLOR_END << " Adding key " << key << " to timeToKeyBuffer for time "
              << std::setprecision(14) << ts << std::endl;
  }

  // Mutex block
  {
    // Writing to IMU buffer --> acquire mutex
    const std::lock_guard<std::mutex> writeInBufferLock(writeInBufferMutex_);
    const auto existing = timeToKeyBuffer_.find(ts);
    if (existing != timeToKeyBuffer_.end()) keyToTimeBuffer_.erase(existing->second);
    timeToKeyBuffer_[ts] = key;
    keyToTimeBuffer_[key] = ts;
    if (ts > tLatestInBuffer_) {
      tLatestInBuffer_ = ts;
    }
    if (timeToKeyBuffer_.size() > static_cast<size_t>(bufferLength_)) {
      const auto oldest = timeToKeyBuffer_.begin();
      keyToTimeBuffer_.erase(oldest->second);
      timeToKeyBuffer_.erase(oldest);
    }
  }
}

bool TimeGraphKeyBuffer::getClosestKeyAndTimestamp(double& tInGraph, gtsam::Key& key, const std::string& callingName,
                                                   const double maxSearchDeviation, const double tK) {
  // Keep the iterator lookup and dereference under the same lock. Measurements
  // can arrive before the first key or after the newest IMU key.
  const std::lock_guard<std::mutex> writeInBufferLock(writeInBufferMutex_);
  tInGraph = 0.0;
  key = 0;
  if (timeToKeyBuffer_.empty()) {
    std::cerr << YELLOW_START << "GMsf-TimeKeyBuffer " << RED_START << "called from " << callingName << ": Buffer is empty!" << COLOR_END
              << std::endl;
    return false;
  }

  const auto upperIterator = timeToKeyBuffer_.lower_bound(tK);
  auto closestIterator = upperIterator;
  if (upperIterator == timeToKeyBuffer_.end()) {
    closestIterator = std::prev(timeToKeyBuffer_.end());
  } else if (upperIterator != timeToKeyBuffer_.begin()) {
    const auto lowerIterator = std::prev(upperIterator);
    if (std::abs(tK - lowerIterator->first) <= std::abs(upperIterator->first - tK)) {
      closestIterator = lowerIterator;
    }
  }
  tInGraph = closestIterator->first;
  key = closestIterator->second;
  double timeDeviation = tInGraph - tK;

  if (verboseLevel_ >= 2) {
    std::cout << YELLOW_START << "GMsf-TimeKeyBuffer" << COLOR_END << " " << callingName << std::setprecision(14)
              << " searched time step: " << tK << std::endl;
    std::cout << YELLOW_START << "GMsf-TimeKeyBuffer" << COLOR_END << " " << callingName << std::setprecision(14)
              << " found time step: " << tInGraph << " at key " << key << std::endl;
    std::cout << YELLOW_START << "GMsf-TimeKeyBuffer" << COLOR_END << " Time Deviation (t_graph-t_request): " << 1000 * timeDeviation
              << " ms" << std::endl;
    std::cout << YELLOW_START << "GMsf-TimeKeyBuffer" << COLOR_END << " Latest IMU timestamp: " << tLatestInBuffer_
              << ", hence absolut delay of measurement is " << 1000 * (tLatestInBuffer_ - tK) << "ms." << std::endl;
  }

  // Check for error and warn user
  if (std::abs(timeDeviation) > maxSearchDeviation) {
    return false;
  }

  return true;
}

}  // namespace graph_msf
