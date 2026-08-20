/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GncOutlierSampling.h
 * @brief   Deterministic outlier sampling for GNC bundle-adjustment timing
 * @author  Ruogu Li
 * @date    Jul 19, 2026
 */

#pragma once

#include <algorithm>
#include <cstddef>
#include <random>
#include <utility>
#include <vector>

namespace gtsam::timing {

using OutlierMeasurement = std::pair<size_t, size_t>;

inline std::vector<OutlierMeasurement> SelectConstrainedOutlierMeasurements(
    const std::vector<size_t>& trackSizes, size_t requested,
    unsigned int seed) {
  if (requested == 0) return {};

  std::vector<OutlierMeasurement> candidates;
  size_t totalCapacity = 0;
  for (size_t track = 0; track < trackSizes.size(); ++track) {
    const size_t measurements = trackSizes[track];
    if (measurements <= 2) continue;
    totalCapacity += measurements - 2;
    for (size_t measurement = 0; measurement < measurements; ++measurement) {
      candidates.emplace_back(track, measurement);
    }
  }

  std::mt19937 rng(seed);
  std::shuffle(candidates.begin(), candidates.end(), rng);

  const size_t target = std::min(requested, totalCapacity);
  std::vector<size_t> selectedPerTrack(trackSizes.size(), 0);
  std::vector<OutlierMeasurement> selected;
  selected.reserve(target);
  for (const auto& candidate : candidates) {
    const size_t track = candidate.first;
    if (selectedPerTrack[track] >= trackSizes[track] - 2) continue;
    selected.push_back(candidate);
    ++selectedPerTrack[track];
    if (selected.size() == target) break;
  }
  return selected;
}

}  // namespace gtsam::timing
