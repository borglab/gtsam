/**
 * @file    cameraMarkerFactor0.h
 * @brief   A non-linear factor of a pose seeing a known marker. Prior on a
 * Marker.
 * @author  Alireza Fathi
 * @author  Frank Dellaert
 */

#pragma once

#include <gtsam/Cal3_S2.h>
#include <gtsam/NonlinearFactor.h>

#include "ARRobotMarker.h"
#include "CameraMarkerFactor.h"

/* ************************************************************************* */
class CameraMarkerFactor1 : public gtsam::NonlinearFactor {
 private:
  int markerNumber_;
  std::string markerName_;

  // for now, should be passed in constructor
  gtsam::Cal3_S2 K_;
  double markerSize_;
  gtsam::Pose3 marker_;

 public:
  typedef boost::shared_ptr<CameraMarkerFactor1> shared_ptr;

  /**
   * Constructor that takes explicit measurements as input
   * @param z is the 8 dimensional concatenation of 4 measurements
   * @param sigma is the standard deviation
   * @param robotFrame is the index of the robot frame
   * @param markerNumber is the marker number
   */
  CameraMarkerFactor1(const Vector& z, double sigma, int rn, gtsam::Cal3_S2 K,
                      int markerSize,
                      const gtsam::Pose3& knownMarker = gtsam::Pose3());

  /**
   * Constructor that takes an AR marker as input
   * please comment !
   */
  CameraMarkerFactor1(const ARRobotMarker& marker, double sigma,
                      const gtsam::Pose3& knownMarker = gtsam::Pose3());

  /**
   * print
   * @param s optional string naming the factor
   */
  void print(const std::string& s = "CameraMarkerFactor0") const;

  /**
   * calculate the error of the factor
   */
  Vector error_vector(const gtsam::FGConfig&) const;

  /**
   * linerarization
   */
  gtsam::LinearFactor::shared_ptr linearize(const gtsam::FGConfig&) const;

  /**
   * equals
   */
  bool equals(const gtsam::Factor&, double tol = 1e-9) const;

  int getMarkerNumber() const { return markerNumber_; }

  /**
   * dump the information of the factor into the string
   */
  std::string dump() const;
};
