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
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/SfmTrack.h>

#include <string>
#include <vector>

namespace gtsam {

/// Define the structure for the camera poses
typedef PinholeCamera<Cal3Bundler> SfmCamera;

/**
 * @brief SfmData stores a bunch of SfmTracks
 * @ingroup sfm
 */
struct GTSAM_EXPORT SfmData {
  std::vector<SfmCamera> cameras;  ///< Set of cameras

  std::vector<SfmTrack> tracks;  ///< Sparse set of points

  /// @name Create from file
  /// @{

  /**
   * @brief Parses a bundler output file and return result as SfmData instance.
   * @param filename The name of the bundler file
   * @param data SfM structure where the data is stored
   * @return true if the parsing was successful, false otherwise
   */
  static SfmData FromBundlerFile(const std::string& filename);

  /**
   * @brief Parse a "Bundle Adjustment in the Large" (BAL) file and return
   * result as SfmData instance.
   * @param filename The name of the BAL file.
   * @return SfM structure where the data is stored.
   */
  static SfmData FromBalFile(const std::string& filename);

  /// @}
  /// @name Standard Interface
  /// @{

  /// Add a track to SfmData
  void addTrack(const SfmTrack& t) { tracks.push_back(t); }

  /// Add a camera to SfmData
  void addCamera(const SfmCamera& cam) { cameras.push_back(cam); }

  /// The number of reconstructed 3D points
  size_t numberTracks() const { return tracks.size(); }

  /// The number of cameras
  size_t numberCameras() const { return cameras.size(); }

  /// The track formed by series of landmark measurements
  const SfmTrack& track(size_t idx) const { return tracks[idx]; }

  /// The camera pose at frame index `idx`
  const SfmCamera& camera(size_t idx) const { return cameras[idx]; }

  /// Getters
  const std::vector<SfmCamera>& cameraList() const { return cameras; }
  const std::vector<SfmTrack>& trackList() const { return tracks; }

  /**
   * @brief Create projection factors using keys i and P(j)
   *
   * @param model a noise model for projection errors
   * @return NonlinearFactorGraph
   */
  NonlinearFactorGraph generalSfmFactors(
      const SharedNoiseModel& model = noiseModel::Isotropic::Sigma(2,
                                                                   1.0)) const;

  /**
   * @brief Create factor graph with projection factors and gauge fix.
   *
   * Note: pose keys are simply integer indices, points use Symbol('p', j).
   *
   * @param model a noise model for projection errors
   * @param fixedCamera which camera to fix, if any (use boost::none if none)
   * @param fixedPoint which point to fix, if any (use boost::none if none)
   * @return NonlinearFactorGraph
   */
  NonlinearFactorGraph sfmFactorGraph(
      const SharedNoiseModel& model = noiseModel::Isotropic::Sigma(2, 1.0),
      boost::optional<size_t> fixedCamera = 0,
      boost::optional<size_t> fixedPoint = 0) const;

  /// @}
  /// @name Testable
  /// @{

  /// print
  void print(const std::string& s = "") const;

  /// assert equality up to a tolerance
  bool equals(const SfmData& sfmData, double tol = 1e-9) const;

  /// @}
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

/**
 * @brief This function parses a "Bundle Adjustment in the Large" (BAL) file and
 * returns the data as a SfmData structure. Mainly used by wrapped code.
 * @param filename The name of the BAL file.
 * @return SfM structure where the data is stored.
 */
GTSAM_EXPORT SfmData readBal(const std::string& filename);

/**
 * @brief This function writes a "Bundle Adjustment in the Large" (BAL) file
 * from a SfmData structure
 * @param filename The name of the BAL file to write
 * @param data SfM structure where the data is stored
 * @return true if the parsing was successful, false otherwise
 */
GTSAM_EXPORT bool writeBAL(const std::string& filename, const SfmData& data);

/**
 * @brief This function writes a "Bundle Adjustment in the Large" (BAL) file
 * from a SfmData structure and a value structure (measurements are the same as
 * the SfM input data, while camera poses and values are read from Values)
 * @param filename The name of the BAL file to write
 * @param data SfM structure where the data is stored
 * @param values structure where the graph values are stored (values can be
 * either Pose3 or PinholeCamera<Cal3Bundler> for the cameras, and should be
 * Point3 for the 3D points). Note: assumes that the keys are "i" for pose i
 * and "Symbol::('p',j)" for landmark j.
 * @return true if the parsing was successful, false otherwise
 */
GTSAM_EXPORT bool writeBALfromValues(const std::string& filename,
                                     const SfmData& data, const Values& values);

/**
 * @brief This function converts an openGL camera pose to an GTSAM camera pose
 * @param R rotation in openGL
 * @param tx x component of the translation in openGL
 * @param ty y component of the translation in openGL
 * @param tz z component of the translation in openGL
 * @return Pose3 in GTSAM format
 */
GTSAM_EXPORT Pose3 openGL2gtsam(const Rot3& R, double tx, double ty, double tz);

/**
 * @brief This function converts a GTSAM camera pose to an openGL camera pose
 * @param R rotation in GTSAM
 * @param tx x component of the translation in GTSAM
 * @param ty y component of the translation in GTSAM
 * @param tz z component of the translation in GTSAM
 * @return Pose3 in openGL format
 */
GTSAM_EXPORT Pose3 gtsam2openGL(const Rot3& R, double tx, double ty, double tz);

/**
 * @brief This function converts a GTSAM camera pose to an openGL camera pose
 * @param PoseGTSAM pose in GTSAM format
 * @return Pose3 in openGL format
 */
GTSAM_EXPORT Pose3 gtsam2openGL(const Pose3& PoseGTSAM);

/**
 * @brief This function creates initial values for cameras from db.
 *
 * No symbol is used, so camera keys are simply integer indices.
 *
 * @param SfmData
 * @return Values
 */
GTSAM_EXPORT Values initialCamerasEstimate(const SfmData& db);

/**
 * @brief This function creates initial values for cameras and points from db
 *
 * Note: Pose keys are simply integer indices, points use Symbol('p', j).
 *
 * @param SfmData
 * @return Values
 */
GTSAM_EXPORT Values initialCamerasAndPointsEstimate(const SfmData& db);

}  // namespace gtsam
