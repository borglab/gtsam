/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  ImuFactorWithGravity.h
 *  @author Nikhil Khedekar
 **/

#pragma once

#include <gtsam/navigation/ImuFactor.h>

namespace gtsam {

/**
 * ImuFactorWithGravityT is a 6-ways factor: in addition to the previous and
 * current states (pose and velocity) and the bias estimate of ImuFactorT, it
 * involves a GRAVITY variable so that gravity can be optimized instead of
 * being fixed by the preintegration parameters. Two parametrizations are
 * provided, see ImuFactorWithGravityDirection and ImuFactorWithGravityVector.
 *
 * As gravity in the nav frame is entangled with the initial attitude (only
 * their combination is observed by the accelerometer), a graph should anchor
 * one of the two: either prior knowledge of attitude (roll/pitch), or a prior
 * on the gravity variable - not both tightly.
 *
 * Like ImuFactorT, this factor does not model temporal consistency of the
 * biases, which is up to the caller; see CombinedImuFactorWithGravityT for a
 * variant that does.
 *
 * @ingroup navigation
 */
template <class PIM = PreintegratedImuMeasurements, class GRAVITY = Unit3>
class GTSAM_EXPORT ImuFactorWithGravityT
    : public NoiseModelFactorN<Pose3, Vector3, Pose3, Vector3,
                               imuBias::ConstantBias, GRAVITY> {
private:

  typedef ImuFactorWithGravityT<PIM, GRAVITY> This;
  typedef NoiseModelFactorN<Pose3, Vector3, Pose3, Vector3,
                            imuBias::ConstantBias, GRAVITY> Base;

  PIM pim_;
  double gravityMagnitude_;  ///< used by the Unit3 parametrization only

public:

  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /** Shorthand for a smart pointer to a factor */
  typedef std::shared_ptr<This> shared_ptr;

  /** Default constructor - only use for serialization */
  ImuFactorWithGravityT() : gravityMagnitude_(0.0) {}

  /**
   * Constructor
   * @param pose_i Previous pose key
   * @param vel_i  Previous velocity key
   * @param pose_j Current pose key
   * @param vel_j  Current velocity key
   * @param bias   Previous bias key
   * @param gravity Gravity key
   * @param preintegratedMeasurements The preintegrated measurements since the
   * last pose
   * @param gravityMagnitude The known gravity magnitude for the Unit3
   * parametrization; defaults to the norm of the gravity vector in the
   * preintegration params. Must not be provided for the Point3
   * parametrization, where the magnitude is part of the optimized variable;
   * to constrain it, add a VectorNormFactor<3> on the gravity variable.
   */
  ImuFactorWithGravityT(Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key bias,
      Key gravity, const PIM& preintegratedMeasurements,
      std::optional<double> gravityMagnitude = {})
      : Base(noiseModel::Gaussian::Covariance(preintegratedMeasurements.preintMeasCov()),
             pose_i, vel_i, pose_j, vel_j, bias, gravity),
        pim_(preintegratedMeasurements),
        gravityMagnitude_(gravityMagnitude
                              ? *gravityMagnitude
                              : preintegratedMeasurements.params()->n_gravity.norm()) {
    if (internal::GravityParametrization<GRAVITY>::usesMagnitude) {
      if (!(gravityMagnitude_ > 0.0))
        throw std::invalid_argument(
            "ImuFactorWithGravityT: gravityMagnitude must be positive");
    } else if (gravityMagnitude) {
      throw std::invalid_argument(
          "ImuFactorWithGravityT: gravityMagnitude is only used by the Unit3 "
          "parametrization; the Point3 parametrization optimizes the magnitude "
          "as part of the gravity variable - to constrain it, add a "
          "VectorNormFactor<3> on the gravity variable instead");
    }
  }

  ~ImuFactorWithGravityT() override {
  }

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
  const PIM& preintegratedMeasurements() const {
    return pim_;
  }

  /** The gravity magnitude used by the Unit3 parametrization. */
  double gravityMagnitude() const {
    return gravityMagnitude_;
  }

  /** implement functions needed to derive from Factor */

  /// vector of errors
  Vector evaluateError(const Pose3& pose_i, const Vector3& vel_i,
                       const Pose3& pose_j, const Vector3& vel_j,
                       const imuBias::ConstantBias& bias_i,
                       const GRAVITY& gravity, OptionalMatrixType H1,
                       OptionalMatrixType H2, OptionalMatrixType H3,
                       OptionalMatrixType H4, OptionalMatrixType H5,
                       OptionalMatrixType H6) const override;

  /// Merge two factors sharing bias and gravity keys, with consecutive states
  template <typename MethodPIMArg = PIM,
    typename = typename std::enable_if<
        std::is_same<MethodPIMArg, PreintegratedImuMeasurementsT<TangentPreintegration>>::value
    >::type
  >
  static typename ImuFactorWithGravityT<MethodPIMArg, GRAVITY>::shared_ptr Merge(
    const typename ImuFactorWithGravityT<MethodPIMArg, GRAVITY>::shared_ptr& f01,
    const typename ImuFactorWithGravityT<MethodPIMArg, GRAVITY>::shared_ptr& f12
  ) {
    if (f01->template key<5>() != f12->template key<5>())
      throw std::domain_error("ImuFactorWithGravityT::Merge: IMU bias keys must be the same");

    if (f01->template key<6>() != f12->template key<6>())
      throw std::domain_error("ImuFactorWithGravityT::Merge: gravity keys must be the same");

    if (internal::GravityParametrization<GRAVITY>::usesMagnitude &&
        std::abs(f01->gravityMagnitude() - f12->gravityMagnitude()) > 1e-9)
      throw std::domain_error(
          "ImuFactorWithGravityT::Merge: gravity magnitudes must be the same");

    if (f01->template key<3>() != f12->template key<1>() || f01->template key<4>() != f12->template key<2>())
      throw std::domain_error(
          "ImuFactorWithGravityT::Merge: intermediate pose, velocity keys need to match up");

    auto pim02 = ImuFactorT<MethodPIMArg>::Merge(f01->preintegratedMeasurements(),
                                                 f12->preintegratedMeasurements());

    return std::make_shared<This>(
        f01->template key<1>(),  // P0
        f01->template key<2>(),  // V0
        f12->template key<3>(),  // P2
        f12->template key<4>(),  // V2
        f01->template key<5>(),  // B
        f01->template key<6>(),  // G
        pim02,
        internal::GravityParametrization<GRAVITY>::usesMagnitude
            ? std::optional<double>(f01->gravityMagnitude())
            : std::nullopt);
  }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    // Archive name for the base follows the sibling factors' convention:
    ar & boost::serialization::make_nvp("NoiseModelFactor6",
         boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(pim_);
    ar & BOOST_SERIALIZATION_NVP(gravityMagnitude_);
  }
#endif
};
// class ImuFactorWithGravityT

/**
 * ImuFactor variant with the gravity direction as an optimized Unit3 variable
 * and a fixed, known magnitude (given at construction, defaulting to the norm
 * of the gravity vector in the preintegration params). This is the preferred
 * parametrization when the magnitude is known, eg. on Earth where standard
 * gravity is accurate to ~0.3% everywhere; it removes the magnitude degree of
 * freedom by construction. See eg. Nemiroff, Chen and Lopez, "Joint
 * On-Manifold Gravity and Accelerometer Intrinsics Estimation for Inertially
 * Aligned Mapping", 2023.
 */
using ImuFactorWithGravityDirection =
    ImuFactorWithGravityT<PreintegratedImuMeasurements, Unit3>;

/**
 * ImuFactor variant with the gravity vector as a free Point3 variable, ie.
 * direction and magnitude both optimized, following Lupton and Sukkarieh,
 * "Visual-Inertial-Aided Navigation for High-Dynamic Motion in Built
 * Environments Without Initial Conditions", TRO 2012. If the magnitude is
 * approximately known, add a VectorNormFactor<3> on the gravity variable once
 * (Lupton's magnitude pseudo-observation); if it is known exactly, prefer
 * ImuFactorWithGravityDirection.
 */
using ImuFactorWithGravityVector =
    ImuFactorWithGravityT<PreintegratedImuMeasurements, Point3>;

// operator<< for ImuFactorWithGravityT
template <class PIM, class GRAVITY>
GTSAM_EXPORT std::ostream& operator<<(std::ostream& os,
                                      const ImuFactorWithGravityT<PIM, GRAVITY>& f);

template <class PIM, class GRAVITY>
struct traits<ImuFactorWithGravityT<PIM, GRAVITY>>
    : public Testable<ImuFactorWithGravityT<PIM, GRAVITY>> {};

} /// namespace gtsam
