/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DsfTrackGenerator.cpp
 * @date October 2022
 * @author John Lambert
 * @brief Identifies connected components in the keypoint matches graph.
 */

#include <gtsam/sfm/DsfTrackGenerator.h>

#include <algorithm>
#include <iostream>
#include <iomanip>

namespace gtsam {

namespace gtsfm {

typedef DSFMap<IndexPair> DSFMapIndexPair;

/// Generate the DSF to form tracks.
static DSFMapIndexPair generateDSF(const MatchIndicesMap& matches) {
  DSFMapIndexPair dsf;

  for (const auto& kv : matches) {
    const auto pair_indices = kv.first;
    const auto corr_indices = kv.second;

    // Image pair is (i1,i2).
    size_t i1 = pair_indices.first;
    size_t i2 = pair_indices.second;
    for (size_t k = 0; k < corr_indices.rows(); k++) {
      // Measurement indices are found in a single matrix row, as (k1,k2).
      size_t k1 = corr_indices(k, 0), k2 = corr_indices(k, 1);
      // Unique key for DSF is (i,k), representing keypoint index in an image.
      dsf.merge({i1, k1}, {i2, k2});
    }
  }

  return dsf;
}

/// Generate a single track from a set of index pairs
static SfmTrack2d trackFromIndexPairs(const std::set<IndexPair>& index_pair_set,
                                      const KeypointsVector& keypoints) {
  // Initialize track from measurements.
  SfmTrack2d track2d;

  for (const auto& index_pair : index_pair_set) {
    // Camera index is represented by i, and measurement index is
    // represented by k.
    size_t i = index_pair.i();
    size_t k = index_pair.j();
    // Add measurement to this track.
    track2d.addMeasurement(i, keypoints[i].coordinates.row(k));
  }

  return track2d;
}

/// Generate tracks from the DSF.
static std::vector<SfmTrack2d> tracksFromDSF(const DSFMapIndexPair& dsf,
                                             const KeypointsVector& keypoints) {
  const std::map<IndexPair, std::set<IndexPair> > key_sets = dsf.sets();

  // Return immediately if no sets were found.
  if (key_sets.empty()) return {};

  // Create a list of tracks.
  // Each track will be represented as a list of (camera_idx, measurements).
  std::vector<SfmTrack2d> tracks2d;
  for (const auto& kv : key_sets) {
    // Initialize track from measurements.
    SfmTrack2d track2d = trackFromIndexPairs(kv.second, keypoints);
    tracks2d.emplace_back(track2d);
  }
  return tracks2d;
}

/**
 * @brief Creates a list of tracks from 2d point correspondences.
 *
 * Creates a disjoint-set forest (DSF) and 2d tracks from pairwise matches.
 * We create a singleton for union-find set elements from camera index of a
 * detection and the index of that detection in that camera's keypoint list,
 * i.e. (i,k).
 *
 * @param Map from (i1,i2) image pair indices to (K,2) matrix, for K
 *        correspondence indices, from each image.
 * @param Length-N list of keypoints, for N images/cameras.
 */
std::vector<SfmTrack2d> tracksFromPairwiseMatches(
    const MatchIndicesMap& matches, const KeypointsVector& keypoints,
    bool verbose) {
  // Generate the DSF to form tracks.
  if (verbose) std::cout << "[SfmTrack2d] Starting Union-Find..." << std::endl;
  DSFMapIndexPair dsf = generateDSF(matches);
  if (verbose) std::cout << "[SfmTrack2d] Union-Find Complete" << std::endl;

  std::vector<SfmTrack2d> tracks2d = tracksFromDSF(dsf, keypoints);

  // Filter out erroneous tracks that had repeated measurements within the
  // same image. This is an expected result from an incorrect correspondence
  // slipping through.
  std::vector<SfmTrack2d> validTracks;
  std::copy_if(
      tracks2d.begin(), tracks2d.end(), std::back_inserter(validTracks),
      [](const SfmTrack2d& track2d) { return track2d.hasUniqueCameras(); });

  if (verbose) {
    size_t erroneous_track_count = tracks2d.size() - validTracks.size();
    double erroneous_percentage = static_cast<float>(erroneous_track_count) /
                                  static_cast<float>(tracks2d.size()) * 100;

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "DSF Union-Find: " << erroneous_percentage;
    std::cout << "% of tracks discarded from multiple obs. in a single image."
              << std::endl;
  }

  // TODO(johnwlambert): return the Transitivity failure percentage here.
  return tracks2d;
}

}  // namespace gtsfm

}  // namespace gtsam
