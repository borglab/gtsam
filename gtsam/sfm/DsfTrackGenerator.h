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
#include <gtsam/base/DSFMap.h>
#include <gtsam/sfm/SfmTrack.h>

#include <Eigen/Core>
#include <map>
#include <optional>
#include <vector>

namespace gtsam {

namespace gtsfm {

typedef Eigen::MatrixX2i CorrespondenceIndices;  // N x 2 array

// Output of detections in an image.
// Coordinate system convention:
// 1. The x coordinate denotes the horizontal direction (+ve direction towards
// the right).
// 2. The y coordinate denotes the vertical direction (+ve direction downwards).
// 3. Origin is at the top left corner of the image.
struct Keypoints {
  // The (x, y) coordinates of the features, of shape Nx2.
  Eigen::MatrixX2d coordinates;

  // Optional scale of the detections, of shape N.
  // Note: gtsam::Vector is typedef'd for Eigen::VectorXd.
  boost::optional<gtsam::Vector> scales;

  /// Optional confidences/responses for each detection, of shape N.
  boost::optional<gtsam::Vector> responses;

  Keypoints(const Eigen::MatrixX2d& coordinates)
      : coordinates(coordinates){};  // boost::none
};

using KeypointsVector = std::vector<Keypoints>;
// Mapping from each image pair to (N,2) array representing indices of matching
// keypoints.
using MatchIndicesMap = std::map<IndexPair, CorrespondenceIndices>;

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
    bool verbose = false);

}  // namespace gtsfm

}  // namespace gtsam
