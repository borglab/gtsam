/**
 * @file    CameraMarkerFactor.h
 * @brief   Non-linear factors for EasySLAM involving camera and markers.
 *          Includes generic factor, factor for known marker, and factor for known robot pose.
 * @author  Alireza Fathi
 * @author  Frank Dellaert
 */

#pragma once

#include "ARRobotMarker.h" // Assumed to be in the same include path

#include <gtsam/Cal3_S2.h>
#include <gtsam/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Vector.h>

#include <string>
#include <list>


 /* ************************************************************************* */
 // Class: CameraMarkerFactor (estimates both robot and marker pose)
 /* ************************************************************************* */
class CameraMarkerFactor : public gtsam::NonlinearFactor {
private:
  int robotNumber_, markerNumber_;
  std::string robotName_, markerName_;

  gtsam::Cal3_S2 K_;
  double markerSize_;

public:
  typedef boost::shared_ptr<CameraMarkerFactor> shared_ptr;

  /**
   * Performs initialization of additional (non-measurement) information.
   * @param rn is the index of the robot frame.
   * @param mn is the marker number.
   * @param K is the camera calibration.
   * @param markerSize is the size of the marker.
   */
  void init(int rn, int mn, gtsam::Cal3_S2 K, int markerSize);

  /**
   * Constructor that takes explicit measurements as input.
   * @param z is the 8-dimensional concatenation of 4 measurements.
   * @param sigma is the standard deviation.
   * @param rn is the index of the robot frame.
   * @param mn is the marker number.
   * @param K is the camera calibration.
   * @param markerSize is the size of the marker.
   */
  CameraMarkerFactor(const gtsam::Vector& z, double sigma, int rn, int mn,
    gtsam::Cal3_S2 K, int markerSize);

  /**
   * Constructor that takes an AR marker as input.
   * @param marker with measurement information.
   * @param sigma the standard deviation.
   */
  CameraMarkerFactor(const ARRobotMarker& marker, double sigma);

  /** Destructor */
  ~CameraMarkerFactor() {}

  /** Print factor information */
  void print(const std::string& s = "CameraMarkerFactor") const;

  /** Calculate the error vector */
  gtsam::Vector error_vector(const gtsam::FGConfig& c) const;

  /** Linearize factor */
  gtsam::LinearFactor::shared_ptr linearize(const gtsam::FGConfig& c) const;

  /** Check equality with another factor */
  bool equals(const gtsam::Factor& f, double tol = 1e-9) const;

  int getRobotNumber() const { return robotNumber_; }
  int getMarkerNumber() const { return markerNumber_; }
  const gtsam::Cal3_S2& calibration() const { return K_; }
  double getMarkerSize() const { return markerSize_; } // Renamed for consistency

  /** Dump the information of the factor into a string */
  std::string dump() const;
};

/* ************************************************************************* */
// Class: CameraMarkerFactor0 (estimates robot pose, marker pose is known)
/* ************************************************************************* */
class CameraMarkerFactor0 : public gtsam::NonlinearFactor {
private:
  int robotNumber_;
  std::string robotName_;

  gtsam::Cal3_S2 K_;
  double markerSize_;
  gtsam::Pose3 marker_; // Known marker pose

public:
  typedef boost::shared_ptr<CameraMarkerFactor0> shared_ptr;

  /**
   * Constructor that takes explicit measurements as input.
   * @param z is the 8-dimensional concatenation of 4 measurements.
   * @param sigma is the standard deviation.
   * @param rn is the index of the robot frame.
   * @param K is the camera calibration.
   * @param markerSize is the size of the marker.
   * @param knownMarker is the known pose of the marker.
   */
  CameraMarkerFactor0(const gtsam::Vector& z, double sigma, int rn, gtsam::Cal3_S2 K,
    int markerSize,
    const gtsam::Pose3& knownMarker = gtsam::Pose3());

  /**
   * Constructor that takes an AR marker as input.
   * @param marker with measurement information.
   * @param sigma the standard deviation.
   * @param knownMarker is the known pose of the marker.
   */
  CameraMarkerFactor0(const ARRobotMarker& marker, double sigma,
    const gtsam::Pose3& knownMarker = gtsam::Pose3());

  /** Destructor */
  ~CameraMarkerFactor0() {}

  /** Print factor information */
  void print(const std::string& s = "CameraMarkerFactor0") const;

  /** Calculate the error vector */
  gtsam::Vector error_vector(const gtsam::FGConfig& c) const;

  /** Linearize factor */
  gtsam::LinearFactor::shared_ptr linearize(const gtsam::FGConfig& c) const;

  /** Check equality with another factor */
  bool equals(const gtsam::Factor& f, double tol = 1e-9) const;

  int getRobotNumber() const { return robotNumber_; }
  const gtsam::Cal3_S2& calibration() const { return K_; }
  double getMarkerSize() const { return markerSize_; } // Renamed for consistency
  const gtsam::Pose3& knownMarkerPose() const { return marker_; }

  /** Dump the information of the factor into a string */
  std::string dump() const;
};

/* ************************************************************************* */
// Class: CameraMarkerFactor1 (estimates marker pose, robot pose is known)
/* ************************************************************************* */
class CameraMarkerFactor1 : public gtsam::NonlinearFactor {
private:
  int markerNumber_;
  std::string markerName_;

  gtsam::Cal3_S2 K_;
  double markerSize_;
  gtsam::Pose3 robotPose_; // Known robot pose (camera pose)

public:
  typedef boost::shared_ptr<CameraMarkerFactor1> shared_ptr;

  /**
   * Constructor that takes explicit measurements as input.
   * @param z is the 8-dimensional concatenation of 4 measurements.
   * @param sigma is the standard deviation.
   * @param mn is the marker number.
   * @param K is the camera calibration.
   * @param markerSize is the size of the marker.
   * @param knownRobot is the known pose of the robot (camera).
   */
  CameraMarkerFactor1(const gtsam::Vector& z, double sigma, int mn, gtsam::Cal3_S2 K,
    int markerSize,
    const gtsam::Pose3& knownRobot = gtsam::Pose3());

  /**
   * Constructor that takes an AR marker as input.
   * @param marker with measurement information.
   * @param sigma the standard deviation.
   * @param knownRobot is the known pose of the robot (camera).
   */
  CameraMarkerFactor1(const ARRobotMarker& marker, double sigma,
    const gtsam::Pose3& knownRobot = gtsam::Pose3());

  /** Destructor */
  ~CameraMarkerFactor1() {}

  /** Print factor information */
  void print(const std::string& s = "CameraMarkerFactor1") const;

  /** Calculate the error vector */
  gtsam::Vector error_vector(const gtsam::FGConfig& c) const;

  /** Linearize factor */
  gtsam::LinearFactor::shared_ptr linearize(const gtsam::FGConfig& c) const;

  /** Check equality with another factor */
  bool equals(const gtsam::Factor& f, double tol = 1e-9) const;

  int getMarkerNumber() const { return markerNumber_; }
  const gtsam::Cal3_S2& calibration() const { return K_; }
  double getMarkerSize() const { return markerSize_; } // Renamed for consistency
  const gtsam::Pose3& knownRobotPose() const { return robotPose_; }

  /** Dump the information of the factor into a string */
  std::string dump() const;
};