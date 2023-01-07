/*
 * @file OrientedPlane3Factor.cpp
 * @brief OrientedPlane3 Factor class
 * @author Alex Trevor
 * @date December 22, 2013
 */

#pragma once

#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/**
 * Factor to measure a planar landmark from a given pose
 */
class GTSAM_EXPORT OrientedPlane3Factor: public NoiseModelFactorN<Pose3, OrientedPlane3> {
 protected:
  OrientedPlane3 measured_p_;
  typedef NoiseModelFactorN<Pose3, OrientedPlane3> Base;

 public:
  using NoiseModelFactor2<Pose3, OrientedPlane3>::evaluateError;
  /// Constructor
  OrientedPlane3Factor() {
  }
  ~OrientedPlane3Factor() override {}

  /** Constructor with measured plane (a,b,c,d) coefficients
   * @param z measured plane (a,b,c,d) coefficients as 4D vector
   * @param noiseModel noiseModel Gaussian noise model
   * @param poseKey Key or symbol for unknown pose
   * @param landmarkKey Key or symbol for unknown planar landmark
   * @return the transformed plane
   */
  OrientedPlane3Factor(const Vector4& z, const SharedGaussian& noiseModel,
                       Key poseKey, Key landmarkKey)
      : Base(noiseModel, poseKey, landmarkKey), measured_p_(z) {}

  /// print
  void print(const std::string& s = "OrientedPlane3Factor",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

  /// evaluateError
  Vector evaluateError(
      const Pose3& pose, const OrientedPlane3& plane,
      OptionalMatrixType H1, OptionalMatrixType H2) const override;
};

// TODO: Convert this factor to dimension two, three dimensions is redundant for direction prior
class GTSAM_EXPORT OrientedPlane3DirectionPrior : public NoiseModelFactorN<OrientedPlane3> {
 protected:
  OrientedPlane3 measured_p_;  /// measured plane parameters
  typedef NoiseModelFactorN<OrientedPlane3> Base;

 public:
  using Base::evaluateError;
  typedef OrientedPlane3DirectionPrior This;
  /// Constructor
  OrientedPlane3DirectionPrior() {
  }

  /// Constructor with measured plane coefficients (a,b,c,d), noise model, landmark symbol
  OrientedPlane3DirectionPrior(Key key, const Vector4& z,
                               const SharedGaussian& noiseModel)
      : Base(noiseModel, key), measured_p_(z) {}

  /// print
  void print(const std::string& s = "OrientedPlane3DirectionPrior",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

  /// equals
  bool equals(const NonlinearFactor& expected, double tol = 1e-9) const override;

  Vector evaluateError(const OrientedPlane3& plane, OptionalMatrixType H) const override;
};

} // gtsam

