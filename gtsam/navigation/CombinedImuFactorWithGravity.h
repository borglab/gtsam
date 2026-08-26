/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  CombinedImuFactorWithGravity.h
 *  @author Nikhil Khedekar
 **/

#pragma once

#include <gtsam/navigation/CombinedImuFactor.h>

namespace gtsam {

/**
 * CombinedImuFactorWithGravityT is CombinedImuFactorT with an additional
 * GRAVITY variable, so that gravity can be optimized instead of being fixed
 * by the preintegration parameters. The 15-dimensional residual is identical
 * to CombinedImuFactorT (9 preintegration rows and 6 bias random walk rows);
 * only the preintegration rows depend on gravity. Jointly estimating gravity
 * and the (evolving) accelerometer bias requires sufficient rotation
 * excitation, see Nemiroff, Chen and Lopez, "Joint On-Manifold Gravity and
 * Accelerometer Intrinsics Estimation for Inertially Aligned Mapping", 2023.
 *
 * See ImuFactorWithGravityDirection / ImuFactorWithGravityVector for the two
 * gravity parametrizations and their usage guidance.
 *
 * @ingroup navigation
 */
template <class PIM = PreintegratedCombinedMeasurements, class GRAVITY = Unit3>
class GTSAM_EXPORT CombinedImuFactorWithGravityT
    : public NoiseModelFactorN<Pose3, Vector3, Pose3, Vector3,
                               imuBias::ConstantBias, imuBias::ConstantBias,
                               GRAVITY> {
 private:
  typedef CombinedImuFactorWithGravityT<PIM, GRAVITY> This;
  typedef NoiseModelFactorN<Pose3, Vector3, Pose3, Vector3,
                            imuBias::ConstantBias, imuBias::ConstantBias,
                            GRAVITY>
      Base;

  PIM pim_;
  double gravityMagnitude_;  ///< used by the Unit3 parametrization only

 public:
  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /** Shorthand for a smart pointer to a factor */
  typedef std::shared_ptr<This> shared_ptr;

  /** Default constructor - only use for serialization */
  CombinedImuFactorWithGravityT() : gravityMagnitude_(0.0) {}

  /**
   * Constructor
   * @param pose_i Previous pose key
   * @param vel_i  Previous velocity key
   * @param pose_j Current pose key
   * @param vel_j  Current velocity key
   * @param bias_i Previous bias key
   * @param bias_j Current bias key
   * @param gravity Gravity key
   * @param preintegratedMeasurements Combined IMU measurements
   * @param gravityMagnitude The known gravity magnitude for the Unit3
   * parametrization; defaults to the norm of the gravity vector in the
   * preintegration params. Must not be provided for the Point3
   * parametrization, where the magnitude is part of the optimized variable;
   * to constrain it, add a VectorNormFactor<3> on the gravity variable.
   */
  CombinedImuFactorWithGravityT(Key pose_i, Key vel_i, Key pose_j, Key vel_j,
                                Key bias_i, Key bias_j, Key gravity,
                                const PIM& preintegratedMeasurements,
                                std::optional<double> gravityMagnitude = {})
      : Base(noiseModel::Gaussian::Covariance(
                 preintegratedMeasurements.residualCovariance()),
             pose_i, vel_i, pose_j, vel_j, bias_i, bias_j, gravity),
        pim_(preintegratedMeasurements),
        gravityMagnitude_(
            gravityMagnitude
                ? *gravityMagnitude
                : preintegratedMeasurements.params()->n_gravity.norm()) {
    if (internal::GravityParametrization<GRAVITY>::usesMagnitude) {
      if (!(gravityMagnitude_ > 0.0))
        throw std::invalid_argument(
            "CombinedImuFactorWithGravityT: gravityMagnitude must be positive");
    } else if (gravityMagnitude) {
      throw std::invalid_argument(
          "CombinedImuFactorWithGravityT: gravityMagnitude is only used by the "
          "Unit3 parametrization; the Point3 parametrization optimizes the "
          "magnitude as part of the gravity variable - to constrain it, add a "
          "VectorNormFactor<3> on the gravity variable instead");
    }
  }

  ~CombinedImuFactorWithGravityT() override {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<This>(*this);
  }

  /// @name Testable
  /// @{
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;
  bool equals(const NonlinearFactor& expected, double tol = 1e-9) const override;
  /// @}

  /** Access the preintegrated measurements. */
  const PIM& preintegratedMeasurements() const { return pim_; }

  /** The gravity magnitude used by the Unit3 parametrization. */
  double gravityMagnitude() const { return gravityMagnitude_; }

  /** implement functions needed to derive from Factor */

  /// vector of errors
  Vector evaluateError(const Pose3& pose_i, const Vector3& vel_i,
                       const Pose3& pose_j, const Vector3& vel_j,
                       const imuBias::ConstantBias& bias_i,
                       const imuBias::ConstantBias& bias_j,
                       const GRAVITY& gravity, OptionalMatrixType H1,
                       OptionalMatrixType H2, OptionalMatrixType H3,
                       OptionalMatrixType H4, OptionalMatrixType H5,
                       OptionalMatrixType H6, OptionalMatrixType H7) const override;

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    // Archive name for the base follows the sibling factors' convention:
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor7", boost::serialization::base_object<Base>(*this));
    ar& BOOST_SERIALIZATION_NVP(pim_);
    ar& BOOST_SERIALIZATION_NVP(gravityMagnitude_);
  }
#endif
};
// class CombinedImuFactorWithGravityT

/// CombinedImuFactor variant optimizing the gravity direction (Unit3) with a
/// fixed, known magnitude; see ImuFactorWithGravityDirection.
using CombinedImuFactorWithGravityDirection =
    CombinedImuFactorWithGravityT<PreintegratedCombinedMeasurements, Unit3>;

/// CombinedImuFactor variant optimizing the free gravity vector (Point3);
/// see ImuFactorWithGravityVector.
using CombinedImuFactorWithGravityVector =
    CombinedImuFactorWithGravityT<PreintegratedCombinedMeasurements, Point3>;

// operator<< for CombinedImuFactorWithGravityT
template <class PIM, class GRAVITY>
GTSAM_EXPORT std::ostream& operator<<(
    std::ostream& os, const CombinedImuFactorWithGravityT<PIM, GRAVITY>& f);

template <class PIM, class GRAVITY>
struct traits<CombinedImuFactorWithGravityT<PIM, GRAVITY>>
    : public Testable<CombinedImuFactorWithGravityT<PIM, GRAVITY>> {};

}  // namespace gtsam
