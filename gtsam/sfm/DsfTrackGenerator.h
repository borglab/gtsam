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
 * @brief Identifies connected components in the keypoint matched graph.
 */

#pragma once
#include <pybind11/stl.h>
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
typedef std::pair<size_t, size_t> ImagePair;
typedef Eigen::MatrixX2i CorrespondenceIndices; // N x 2 array

//struct Keypoints;
using KeypointCoordinates = Eigen::MatrixX2d;


struct Keypoints
{
  KeypointCoordinates coordinates;
  // typedef'd for Eigen::VectorXd
  boost::optional<gtsam::Vector> scales;
  boost::optional<gtsam::Vector> responses;

  Keypoints(const gtsam::KeypointCoordinates& coordinates): coordinates(coordinates) {}; // boost::none
};

using KeypointsList = std::vector<Keypoints>;
using KeypointsVector = std::vector<Keypoints>; // TODO(johnwlambert): prefer KeypointsSet?
using MatchIndicesMap = std::map<ImagePair, CorrespondenceIndices>;


// @param camera index
// @param 2d measurement
// Implemented as named tuple, instead of std::pair (like SfmMeasurement in SfmTrack.h)
struct NamedSfmMeasurement
{
  size_t i;
  gtsam::Point2 uv;

  NamedSfmMeasurement(size_t i, gtsam::Point2 uv) : i(i), uv(uv) {}
};


class SfmTrack2d
{
  private:
    std::vector<NamedSfmMeasurement> measurements_;

  public:
    void addMeasurement(const NamedSfmMeasurement &m) {
      measurements_.emplace_back(m);
    }
    std::vector<NamedSfmMeasurement> measurements() {return measurements_; }

  // @brief Validates the track by checking that no two measurements are from the same camera.
  //
  // returns boolean result of the validation.
  bool validate_unique_cameras()
  {
    std::vector<int> track_cam_idxs;
    for (auto & measurement: measurements_)
    {
      track_cam_idxs.emplace_back(measurement.i);
    }
    auto i = std::adjacent_find(track_cam_idxs.begin(), track_cam_idxs.end());
    bool all_cameras_unique = (i == track_cam_idxs.end());
    return all_cameras_unique;
  }
};


class DsfTrackGenerator
{

  public:
    DsfTrackGenerator() {}

    // Creates a list of tracks from 2d point correspondences.
    //
    // Creates a disjoint-set forest (DSF) and 2d tracks from pairwise matches. We create a singleton for union-find
    // set elements from camera index of a detection and the index of that detection in that camera's keypoint list,
    // i.e. (i,k).
    std::vector<SfmTrack2d> generate_tracks_from_pairwise_matches(
        const MatchIndicesMap& matches_dict,
        const KeypointsList& keypoints_list)
    {
      std::vector<SfmTrack2d> track_2d_list;

      std::cout << "[SfmTrack2d] Starting Union-Find..." << std::endl;
      // Generate the DSF to form tracks.
      DSFMapIndexPair dsf;
      
      for (const auto& [pair_idxs, corr_idxs]: matches_dict) {
        // Image pair is (i1,i2).
        size_t i1 = pair_idxs.first;
        size_t i2 =  pair_idxs.second;
        for (size_t k = 0; k < corr_idxs.rows(); k++)
        {
          // Measurement indices are found in a single matrix row, as (k1,k2).
          size_t k1 = corr_idxs(k,0);
          size_t k2 = corr_idxs(k,1);
          // Unique keys for the DSF are (i,k), representing keypoint index in an image.
          dsf.merge(IndexPair(i1, k1), IndexPair(i2, k2));
        }
      }

      std::cout << "[SfmTrack2d] Union-Find Complete" << std::endl;
      const std::map<IndexPair, std::set<IndexPair> > key_sets = dsf.sets();

      size_t erroneous_track_count = 0;
      // Create a list of tracks.
      // Each track will be represented as a list of (camera_idx, measurements).
      for (const auto& [set_id, index_pair_set]: key_sets) {
        // Initialize track from measurements.
        SfmTrack2d track_2d;

        for (const auto& index_pair: index_pair_set)
        {
          // Camera index is represented by i, and measurement index is represented by k.
          size_t i = index_pair.i();
          size_t k = index_pair.j();
          // Add measurement to this track.
          track_2d.addMeasurement(NamedSfmMeasurement(i, keypoints_list[i].coordinates.row(k)));
        }

        // Skip erroneous track that had repeated measurements within the same image.
        // This is an expected result from an incorrect correspondence slipping through.
        if (track_2d.validate_unique_cameras())
        {
          track_2d_list.emplace_back(track_2d);
        } else {
          erroneous_track_count += 1;
        }
      }

      double erroneous_track_pct = key_sets.size() > 0 ? erroneous_track_count / key_sets.size() * 100 : std::nan("1");
      // TODO(johnwlambert): restrict decimal places to 2 decimals.
      std::cout << "DSF Union-Find: " << erroneous_track_pct;
      std::cout << "% of tracks discarded from multiple obs. in a single image." << std::endl;
      
      // TODO(johnwlambert): return the Transitivity failure percentage here.
      return track_2d_list;
    }
};

} // namespace gtsam

