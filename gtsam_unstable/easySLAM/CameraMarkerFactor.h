/**
 * @file    cameraMarkerFactor.h
 * @brief   A non-linear factor specialized to the EasySLAM problem. Factor for
 * measurement between robot and marker.
 * @author  Alireza Fathi
 * @author  Frank Dellaert
 */

#pragma once

#include <gtsam/Cal3_S2.h>
#include <gtsam/NonlinearFactor.h>

#include "ARRobotMarker.h"

/* ************************************************************************* */
class CameraMarkerFactor : public gtsam::NonlinearFactor {
 private:
  int robotNumber_, markerNumber_;
  std::string robotName_, markerName_;

  // for now, should be passed in constructor
  gtsam::Cal3_S2 K_;
  double markerSize_;

 public:
  typedef boost::shared_ptr<CameraMarkerFactor> shared_ptr;

  /**
   * performs initialization of additional (non-measurement) information
   * @param robotFrame is the index of the robot frame
   * @param markerNumber is the marker number
   * @param K is the camera calibration
   * @param markerSize is the size of the marker
   */
  void init(int rn, int mn, gtsam::Cal3_S2 K, int markerSize);

  /**
   * Constructor that takes explicit measurements as input
   * @param z is the 8 dimensional concatenation of 4 measurements
   * @param sigma is the standard deviation
   * @param robotFrame is the index of the robot frame
   * @param markerNumber is the marker number
   * @param K is the camera calibration
   * @param markerSize is the size of the marker
   */
  CameraMarkerFactor(const Vector& z, double sigma, int rn, int mn,
                     gtsam::Cal3_S2 K, int markerSize);

  /**
   * Constructor that takes an AR marker as input
   * @param marker with measurement information
   * @param sigma the standard deviation passed to the underlying
   * nonlinearfactor
   */
  CameraMarkerFactor(const ARRobotMarker& marker, double sigma);

  /**
   * Destructor
   */
  ~CameraMarkerFactor() {}

  /**
   * print
   * @param s optional string naming the factor
   */
  void print(const std::string& s = "CameraMarkerFactor") const;

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

  int getRobotNumber() const { return robotNumber_; }

  int getMarkerNumber() const { return markerNumber_; }

  /**
   * dump the information of the factor into the string
   */
  std::string dump() const;
};
