/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DsfTrackGenerator.h
 * @date July 2022
 * @author John Lambert
 * @brief Identifies connected components in the keypoint matches graph.
 */

#pragma once
#include <Eigen/Core>

#include <gtsam/base/DSFMap.h>
#include <gtsam/geometry/Point2.h>

#include <algorithm>
#include <iostream>
#include <map>
#include <optional>
#include <vector>

namespace gtsam {

typedef DSFMap<IndexPair> DSFMapIndexPair;
typedef Eigen::MatrixX2i CorrespondenceIndices; // N x 2 array

//struct Keypoints;
using KeypointCoordinates = Eigen::MatrixX2d;


// Output of detections in an image.
// Coordinate system convention:
// 1. The x coordinate denotes the horizontal direction (+ve direction towards the right).
// 2. The y coordinate denotes the vertical direction (+ve direction downwards).
// 3. Origin is at the top left corner of the image.
struct Keypoints {

  // The (x, y) coordinates of the features, of shape Nx2.
  KeypointCoordinates coordinates;

  // Optional scale of the detections, of shape N.
  // Note: gtsam::Vector is typedef'd for Eigen::VectorXd.
  boost::optional<gtsam::Vector> scales;

  /// Optional confidences/responses for each detection, of shape N.
  boost::optional<gtsam::Vector> responses;

  Keypoints(const gtsam::KeypointCoordinates& coordinates): coordinates(coordinates) {}; // boost::none
};

using KeypointsVector = std::vector<Keypoints>;
// Mapping from each image pair to (N,2) array representing indices of matching keypoints.
using MatchIndicesMap = std::map<IndexPair, CorrespondenceIndices>;


// @param camera index
// @param 2d measurement
// Implemented as named tuple, instead of std::pair (like SfmMeasurement in SfmTrack.h)
struct NamedSfmMeasurement {
  size_t i;
  gtsam::Point2 uv;

  NamedSfmMeasurement(size_t i, gtsam::Point2 uv) : i(i), uv(uv) {}
};


/**
 * @brief Track containing 2D measurements associated with a single 3D point.
 * Note: Equivalent to gtsam.SfmTrack, but without the 3d measurement.
 * This class holds data temporarily before 3D point is initialized.
 */
class SfmTrack2d {
 private:
  std::vector<NamedSfmMeasurement> measurements_;

 public:
  // Default constructor.
  SfmTrack2d() = default;

  // Constructor from measurements.
  SfmTrack2d(std::vector<NamedSfmMeasurement> &measurements) : measurements_(measurements) {}

  // Add a measurement to the track.
  void addMeasurement(const NamedSfmMeasurement &m) {
    measurements_.emplace_back(m);
  }

  /// The measurement at index `idx`
  NamedSfmMeasurement measurement(size_t idx) const { return measurements_[idx]; }

  // Return all measurements in the track.
  std::vector<NamedSfmMeasurement> measurements() {return measurements_; }

  /// Total number of measurements in this track.
  size_t numberMeasurements() const { return measurements_.size(); }

  // @brief Validates the track by checking that no two measurements are from the same camera.
  //
  // returns boolean result of the validation.
  bool hasUniqueCameras()
  {
    std::vector<int> track_cam_idxs;
    for (auto & measurement : measurements_)
    {
      track_cam_idxs.emplace_back(measurement.i);
    }
    auto i = std::adjacent_find(track_cam_idxs.begin(), track_cam_idxs.end());
    bool all_cameras_unique = (i == track_cam_idxs.end());
    return all_cameras_unique;
  }
};

using SfmTrack2dVector = std::vector<gtsam::SfmTrack2d>;
using NamedSfmMeasurementVector = std::vector<NamedSfmMeasurement>;


/**
 * @brief Generates point tracks from connected components in the keypoint matches graph.
 */
class DsfTrackGenerator {

 public:
  /** Default constructor. */
  DsfTrackGenerator() {}

  // Creates a list of tracks from 2d point correspondences.
  //
  // Creates a disjoint-set forest (DSF) and 2d tracks from pairwise matches.
  // We create a singleton for union-find set elements from camera index of a
  // detection and the index of that detection in that camera's keypoint list,
  // i.e. (i,k).
  // @param Map from (i1,i2) image pair indices to (K,2) matrix, for K
  //        correspondence indices, from each image.
  // @param Length-N list of keypoints, for N images/cameras.
  std::vector<SfmTrack2d> generate_tracks_from_pairwise_matches(
    const MatchIndicesMap& matches_dict,
    const KeypointsVector& keypoints_list,
    bool verbose = false) {
    std::vector<SfmTrack2d> track_2d_list;

    if (verbose) std::cout << "[SfmTrack2d] Starting Union-Find..." << std::endl;
    // Generate the DSF to form tracks.
    DSFMapIndexPair dsf;

    for (const auto& kv : matches_dict) {
      const auto pair_idxs = kv.first;
      const auto corr_idxs = kv.second;

      // Image pair is (i1,i2).
      size_t i1 = pair_idxs.first;
      size_t i2 =  pair_idxs.second;
      for (size_t k = 0; k < corr_idxs.rows(); k++)
      {
        // Measurement indices are found in a single matrix row, as (k1,k2).
        size_t k1 = corr_idxs(k, 0);
        size_t k2 = corr_idxs(k, 1);
        // Unique keys for the DSF are (i,k), representing keypoint index in an image.
        dsf.merge(IndexPair(i1, k1), IndexPair(i2, k2));
      }
    }

    if (verbose) std::cout << "[SfmTrack2d] Union-Find Complete" << std::endl;
    const std::map<IndexPair, std::set<IndexPair> > key_sets = dsf.sets();

    // Return immediately if no sets were found.
    if (key_sets.empty()) return track_2d_list;

    size_t erroneous_track_count = 0;
    // Create a list of tracks.
    // Each track will be represented as a list of (camera_idx, measurements).
    for (const auto& kv : key_sets) {
      const auto set_id = kv.first;
      const auto index_pair_set = kv.second;

      // Initialize track from measurements.
      SfmTrack2d track_2d;

      for (const auto& index_pair : index_pair_set)
      {
        // Camera index is represented by i, and measurement index is represented by k.
        size_t i = index_pair.i();
        size_t k = index_pair.j();
        // Add measurement to this track.
        track_2d.addMeasurement(NamedSfmMeasurement(i, keypoints_list[i].coordinates.row(k)));
      }

      // Skip erroneous track that had repeated measurements within the same image.
      // This is an expected result from an incorrect correspondence slipping through.
      if (track_2d.hasUniqueCameras())
      {
        track_2d_list.emplace_back(track_2d);
      } else {
        erroneous_track_count++;
      }
    }

    if (verbose) {
      double erroneous_track_pct = static_cast<float>(erroneous_track_count)
                                   / static_cast<float>(key_sets.size()) * 100;

      // TODO(johnwlambert): restrict decimal places to 2 decimals.
      std::cout << "DSF Union-Find: " << erroneous_track_pct;
      std::cout << "% of tracks discarded from multiple obs. in a single image." << std::endl;
    }
    // TODO(johnwlambert): return the Transitivity failure percentage here.
    return track_2d_list;
  }
};

} // namespace gtsam

