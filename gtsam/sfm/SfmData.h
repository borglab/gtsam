/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SfmData.h
 * @date January 2022
 * @author Frank dellaert
 * @brief Data structure for dealing with Structure from Motion data
 */

#pragma once

#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/sfm/SfmTrack.h>

#include <string>
#include <vector>

namespace gtsam {

/// Define the structure for the camera poses
typedef PinholeCamera<Cal3Bundler> SfmCamera;

/**
 * @brief SfmData stores a bunch of SfmTracks
 * @addtogroup sfm
 */
struct SfmData {
  std::vector<SfmCamera> cameras;  ///< Set of cameras

  std::vector<SfmTrack> tracks;  ///< Sparse set of points

  /// @name Standard Interface
  /// @{

  /// Add a track to SfmData
  void addTrack(const SfmTrack& t) { tracks.push_back(t); }

  /// Add a camera to SfmData
  void addCamera(const SfmCamera& cam) { cameras.push_back(cam); }

  /// The number of reconstructed 3D points
  size_t nrTracks() const { return tracks.size(); }

  /// The number of cameras
  size_t nrCameras() const { return cameras.size(); }

  /// The track formed by series of landmark measurements
  SfmTrack track(size_t idx) const { return tracks[idx]; }

  /// The camera pose at frame index `idx`
  SfmCamera camera(size_t idx) const { return cameras[idx]; }

  /// @}
  /// @name Testable
  /// @{

  /// print
  void print(const std::string& s = "") const;

  /// assert equality up to a tolerance
  bool equals(const SfmData& sfmData, double tol = 1e-9) const;

  /// @}
#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V42
  /// @name Deprecated
  /// @{
  void GTSAM_DEPRECATED add_track(const SfmTrack& t) { tracks.push_back(t); }
  void GTSAM_DEPRECATED add_camera(const SfmCamera& cam) {
    cameras.push_back(cam);
  }
  size_t GTSAM_DEPRECATED number_tracks() const { return tracks.size(); }
  size_t GTSAM_DEPRECATED number_cameras() const { return cameras.size(); }
  /// @}
#endif
  /// @name Serialization
  /// @{

  /** Serialization function */
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(cameras);
    ar& BOOST_SERIALIZATION_NVP(tracks);
  }

  /// @}
};

/// traits
template <>
struct traits<SfmData> : public Testable<SfmData> {};

}  // namespace gtsam
