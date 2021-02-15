/*
 * @file LocalOrientedPlane3Factor.h
 * @brief LocalOrientedPlane3 Factor class
 * @author David Wisth
 * @date February 12, 2021
 */

#pragma once

#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/**
 * Factor to measure a planar landmark from a given pose, with a given local
 * linearization point.
 *
 * This factor is based on the relative plane factor formulation proposed in:
 * Equation 25,
 * M. Kaess, "Simultaneous Localization and Mapping with Infinite Planes",
 * IEEE International Conference on Robotics and Automation, 2015.
 * 
 * Note: This uses the retraction from the OrientedPlane3, not the quaternion-
 * based representation proposed by Kaess.
 */
class LocalOrientedPlane3Factor: public NoiseModelFactor3<Pose3, Pose3, 
                                                         OrientedPlane3> {
protected:
  OrientedPlane3 measured_p_;
  typedef NoiseModelFactor3<Pose3, Pose3, OrientedPlane3> Base;
public:
  /// Constructor
  LocalOrientedPlane3Factor() {}

  virtual ~LocalOrientedPlane3Factor() {}

  /** Constructor with measured plane (a,b,c,d) coefficients
   * @param z measured plane (a,b,c,d) coefficients as 4D vector
   * @param noiseModel noiseModel Gaussian noise model
   * @param poseKey Key or symbol for unknown pose
   * @param anchorPoseKey Key or symbol for the plane's linearization point.
   * @param landmarkKey Key or symbol for unknown planar landmark
   *
   * Note: The anchorPoseKey can simply be chosen as the first pose a plane
   * is observed.  
   */
  LocalOrientedPlane3Factor(const Vector4& z, const SharedGaussian& noiseModel,
                            Key poseKey, Key anchorPoseKey, Key landmarkKey)
      : Base(noiseModel, poseKey, anchorPoseKey, landmarkKey), measured_p_(z) {}

  LocalOrientedPlane3Factor(const OrientedPlane3& z, const SharedGaussian& noiseModel,
                          Key poseKey, Key anchorPoseKey, Key landmarkKey)
    : Base(noiseModel, poseKey, anchorPoseKey, landmarkKey), measured_p_(z) {}

  /// print
  void print(const std::string& s = "LocalOrientedPlane3Factor",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

  /// evaluateError
  Vector evaluateError(
      const Pose3& basePose, const Pose3& anchorPose, const OrientedPlane3& plane,
      boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none,
      boost::optional<Matrix&> H3 = boost::none) const override;
};

} // gtsam

