/*
 * @file LocalOrientedPlane3Factor.h
 * @brief LocalOrientedPlane3 Factor class
 * @author David Wisth
 * @date February 12, 2021
 */

#pragma once

#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam_unstable/dllexport.h>

#include <string>

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
 *
 * The main purpose of this factor is to improve the numerical stability of the
 * optimization, especially compared to gtsam::OrientedPlane3Factor. This
 * is especially relevant when the sensor is far from the origin (and thus
 * the derivatives associated to transforming the plane are large).
 *
 * x0 is the current sensor pose, and x1 is the local "anchor pose" - i.e.
 * a local linearisation point for the plane. The plane is representated and
 * optimized in x1 frame in the optimization.
 */
class GTSAM_UNSTABLE_EXPORT LocalOrientedPlane3Factor
    : public NoiseModelFactorN<Pose3, Pose3, OrientedPlane3> {
 protected:
  OrientedPlane3 measured_p_;
  typedef NoiseModelFactorN<Pose3, Pose3, OrientedPlane3> Base;
public:
  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /// Constructor
  LocalOrientedPlane3Factor() {}

  ~LocalOrientedPlane3Factor() override {}

  /** Constructor with measured plane (a,b,c,d) coefficients
   * @param z measured plane (a,b,c,d) coefficients as 4D vector
   * @param noiseModel noiseModel Gaussian noise model
   * @param poseKey Key or symbol for unknown pose
   * @param anchorPoseKey Key or symbol for the plane's linearization point,
                          (called the "anchor pose").
   * @param landmarkKey Key or symbol for unknown planar landmark
   *
   * Note: The anchorPoseKey can simply be chosen as the first pose a plane
   * is observed.  
   */
  LocalOrientedPlane3Factor(const Vector4& z, const SharedNoiseModel& noiseModel,
                            Key poseKey, Key anchorPoseKey, Key landmarkKey)
      : Base(noiseModel, poseKey, anchorPoseKey, landmarkKey), measured_p_(z) {}

  LocalOrientedPlane3Factor(const OrientedPlane3& z,
                            const SharedNoiseModel& noiseModel,
                            Key poseKey, Key anchorPoseKey, Key landmarkKey)
    : Base(noiseModel, poseKey, anchorPoseKey, landmarkKey), measured_p_(z) {}

  /// print
  void print(const std::string& s = "LocalOrientedPlane3Factor",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

  /***
    * Vector of errors
    * @brief Error = measured_plane_.error(a_plane.transform(inv(wTwa) * wTwi))
    * 
    * This is the error of the measured and predicted plane in the current
    * sensor frame, i. The plane is represented in the anchor pose, a.
    *
    * @param wTwi The pose of the sensor in world coordinates
    * @param wTwa The pose of the anchor frame in world coordinates
    * @param a_plane The estimated plane in anchor frame.
    *
    * Note: The optimized plane is represented in anchor frame, a, not the
    * world frame.
    */
  Vector evaluateError(const Pose3& wTwi, const Pose3& wTwa,
      const OrientedPlane3& a_plane, OptionalMatrixType H1, 
	  OptionalMatrixType H2, OptionalMatrixType H3) const override;
};

}  // namespace gtsam

