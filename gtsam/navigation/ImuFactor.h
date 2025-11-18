/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  ImuFactor.h
 *  @author Luca Carlone
 *  @author Stephen Williams
 *  @author Richard Roberts
 *  @author Vadim Indelman
 *  @author David Jensen
 *  @author Frank Dellaert
 **/

#pragma once

/* GTSAM includes */
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/TangentPreintegration.h>
#include <gtsam/base/debug.h>

#include <type_traits> // For std::is_same, std::enable_if
 
namespace gtsam {

// Determine default preintegration backend
#ifdef GTSAM_TANGENT_PREINTEGRATION
typedef TangentPreintegration DefaultPreintegrationType;
#else
typedef ManifoldPreintegration DefaultPreintegrationType;
#endif

/*
 * If you are using the factor, please cite:
 * Christian Forster, Luca Carlone, Frank Dellaert, and Davide Scaramuzza,
 * "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry", IEEE
 * Transactions on Robotics, 2017.
 *
 * REFERENCES:
 * [1] G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie Groups",
 *     Volume 2, 2008.
 * [2] T. Lupton and S.Sukkarieh, "Visual-Inertial-Aided Navigation for
 *     High-Dynamic Motion in Built Environments Without Initial Conditions",
 *     TRO, 28(1):61-76, 2012.
 * [3] L. Carlone, S. Williams, R. Roberts, "Preintegrated IMU factor:
 *     Computation of the Jacobian Matrices", Tech. Report, 2013.
 *     Available in this repo as "PreintegratedIMUJacobians.pdf".
 * [4] C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza, "IMU Preintegration
 * on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation",
 *     Robotics: Science and Systems (RSS), 2015.
 */

/**
 * PreintegratedImuMeasurements accumulates (integrates) the IMU measurements
 * (rotation rates and accelerations) and the corresponding covariance matrix.
 * The measurements are then used to build the Preintegrated IMU factor.
 * Integration is done incrementally (ideally, one integrates the measurement
 * as soon as it is received from the IMU) so as to avoid costly integration
 * at time of factor construction.
 *
 * @ingroup navigation
 */
template <class PreintegrationType>
class GTSAM_EXPORT PreintegratedImuMeasurementsT: public PreintegrationType {
 
  template <class PIM> friend class ImuFactorT;
  template <class PIM> friend class ImuFactor2T;

protected:

  Matrix9 preintMeasCov_; ///< COVARIANCE OF: [PreintROTATION PreintPOSITION PreintVELOCITY]
  ///< (first-order propagation from *measurementCovariance*).

public:

  /// Default constructor for serialization and wrappers
  PreintegratedImuMeasurementsT() {
    this->resetIntegration();
  }

 /**
   *  Constructor, initializes the class with no measurements
   *  @param p       Parameters, typically fixed in a single application
   *  @param biasHat Current estimate of acceleration and rotation rate biases
   */
  PreintegratedImuMeasurementsT(const std::shared_ptr<PreintegrationParams>& p,
      const imuBias::ConstantBias& biasHat = imuBias::ConstantBias()) :
      PreintegrationType(p, biasHat) {
    this->resetIntegration();
  }

/**
  *  Construct preintegrated directly from members: base class and preintMeasCov
  *  @param base               PreintegrationType instance
  *  @param preintMeasCov      Covariance matrix used in noise model.
  */
  PreintegratedImuMeasurementsT(const PreintegrationType& base, const Matrix9& preintMeasCov)
     : PreintegrationType(base),
       preintMeasCov_(preintMeasCov) {
    this->PreintegrationType::resetIntegration();
  }
 
  /// Virtual destructor
  ~PreintegratedImuMeasurementsT() override {
  }

  /// print
  void print(const std::string& s = "Preintegrated Measurements:") const override;

  /// equals
  bool equals(const PreintegratedImuMeasurementsT<PreintegrationType>& expected, double tol = 1e-9) const;

  /// Re-initialize PreintegratedImuMeasurements
  void resetIntegration() override;

  /**
   * Add a single IMU measurement to the preintegration.
   * Both accelerometer and gyroscope measurements are taken to be in the sensor
   * frame and conversion to the body frame is handled by `body_P_sensor` in
   * `PreintegrationParams`.
   *
   * @param measuredAcc Measured acceleration (as given by the sensor)
   * @param measuredOmega Measured angular velocity (as given by the sensor)
   * @param dt Time interval between this and the last IMU measurement
   */
  void integrateMeasurement(const Vector3& measuredAcc,
      const Vector3& measuredOmega, const double dt) override;

  /// Add multiple measurements, in matrix columns
  void integrateMeasurements(const Matrix& measuredAccs, const Matrix& measuredOmegas,
                             const Matrix& dts);

  /// Return pre-integrated measurement covariance
  Matrix preintMeasCov() const { return preintMeasCov_; }

  /// Merge in a different set of measurements and update bias derivatives accordingly
  /// This method is specific to TangentPreintegration backend.
  template <typename PB = PreintegrationType,
             // This method is only callable when PreintegrationType is TangentPreintegration.
             typename = typename std::enable_if<std::is_same<PB, TangentPreintegration>::value>::type>
  void mergeWith(const PreintegratedImuMeasurementsT<TangentPreintegration>& pim12, Matrix9* H1, Matrix9* H2) {
    // The `this->PreintegrationType::mergeWith` implies calling TangentPreintegration's mergeWith.
    // Since pim12 is PreintegratedImuMeasurementsT<TangentPreintegration>, it is a TangentPreintegration.
    this->PreintegrationType::mergeWith(pim12, H1, H2);
    // NOTE(gareth): Temporary P is needed as of Eigen 3.3
    const Matrix9 P = *H1 * preintMeasCov_ * H1->transpose();
    preintMeasCov_ = P + *H2 * pim12.preintMeasCov_ * H2->transpose();
  }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION  ///
  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    namespace bs = ::boost::serialization;
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(PreintegrationType);
    ar & BOOST_SERIALIZATION_NVP(preintMeasCov_);
  }
#endif
};

// For backward compatibility (so that the compiler flag GTSAM_TANGENT_PREINTEGRATION still
// controls which class PreintegratedImuMeasurements uses):
using PreintegratedImuMeasurements = PreintegratedImuMeasurementsT<DefaultPreintegrationType>;

/**
 * ImuFactor is a 5-ways factor involving previous state (pose and velocity of
 * the vehicle at previous time step), current state (pose and velocity at
 * current time step), and the bias estimate. Following the preintegration
 * scheme proposed in [2], the ImuFactor includes many IMU measurements, which
 * are "summarized" using the PreintegratedImuMeasurements class.
 * Note that this factor does not model "temporal consistency" of the biases
 * (which are usually slowly varying quantities), which is up to the caller.
 * See also CombinedImuFactor for a class that does this for you.
 *
 * @ingroup navigation
 */
template <class PIM = PreintegratedImuMeasurements>
class GTSAM_EXPORT ImuFactorT: public NoiseModelFactorN<Pose3, Vector3, Pose3, Vector3,
    imuBias::ConstantBias> {
private:

  typedef ImuFactorT<PIM> This;
  typedef NoiseModelFactorN<Pose3, Vector3, Pose3, Vector3,
      imuBias::ConstantBias> Base;

  PIM pim_;

public:

  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /** Shorthand for a smart pointer to a factor */
  typedef std::shared_ptr<This> shared_ptr;


  /** Default constructor - only use for serialization */
  ImuFactorT() {}

  /**
   * Constructor
   * @param pose_i Previous pose key
   * @param vel_i  Previous velocity key
   * @param pose_j Current pose key
   * @param vel_j  Current velocity key
   * @param bias   Previous bias key
   * @param preintegratedMeasurements The preintegreated measurements since the
   * last pose.
   */
  ImuFactorT(Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key bias,
      const PIM& preintegratedMeasurements)
      : Base(noiseModel::Gaussian::Covariance(preintegratedMeasurements.preintMeasCov()),
             pose_i, vel_i, pose_j, vel_j, bias),
        pim_(preintegratedMeasurements) {}

  ~ImuFactorT() override {
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

  /** implement functions needed to derive from Factor */

  /// vector of errors
  Vector evaluateError(const Pose3& pose_i, const Vector3& vel_i,
      const Pose3& pose_j, const Vector3& vel_j,
      const imuBias::ConstantBias& bias_i, OptionalMatrixType H1, OptionalMatrixType H2,
      OptionalMatrixType H3, OptionalMatrixType H4, OptionalMatrixType H5) const override;

  /// Merge two pre-integrated measurement classes
  template <typename MethodPIMArg = PIM,
    // This method is only callable when PIM is PreintegratedImuMeasurementsT<TangentPreintegration>.
    typename = typename std::enable_if<
        std::is_same<MethodPIMArg, PreintegratedImuMeasurementsT<TangentPreintegration>>::value
    >::type
  >
  static MethodPIMArg Merge(
    const MethodPIMArg& pim01,
    const MethodPIMArg& pim12
  ) {
    // When this template is instantiated:
    // 1. MethodPIMArg = PIM. It's mirrored to avoid error C7637 from strict compilers.
    // 2. The SFINAE condition ensures MethodPIMArg IS PreintegratedImuMeasurementsT<TangentPreintegration>.
    // So, arguments are const PreintegratedImuMeasurementsT<TangentPreintegration>&
    // and return is PreintegratedImuMeasurementsT<TangentPreintegration>.

    if (!pim01.matchesParamsWith(pim12))
      throw std::domain_error(
          "Cannot merge PreintegratedImuMeasurements with different params");

    if (pim01.p_->body_P_sensor)
      throw std::domain_error(
          "Cannot merge PreintegratedImuMeasurements with sensor pose yet");

    // the bias for the merged factor will be the bias from 01
    MethodPIMArg pim02 = pim01;

    Matrix9 H1, H2;
    pim02.mergeWith(pim12, &H1, &H2);

    return pim02;
  }

  /// Merge two factors
  template <
    typename MethodPIMArg = PIM,
    // This method is only callable when PIM is PreintegratedImuMeasurementsT<TangentPreintegration>.
    typename = typename std::enable_if<
        std::is_same<MethodPIMArg, PreintegratedImuMeasurementsT<TangentPreintegration>>::value
    >::type
  >
  static typename ImuFactorT<MethodPIMArg>::shared_ptr Merge(
    const typename ImuFactorT<MethodPIMArg>::shared_ptr& f01,
    const typename ImuFactorT<MethodPIMArg>::shared_ptr& f12
  ) {
    // When this template is instantiated:
    // 1. MethodPIMArg = PIM. It's mirrored to avoid error C7637 from strict compilers.
    // 2. The SFINAE condition ensures MethodPIMArg IS PreintegratedImuMeasurementsT<TangentPreintegration>.
    // So, ImuFactorT<MethodPIMArg> is effectively ImuFactorT<PIM>, which is `This`.
    // The signature effectively becomes:
    // static typename This::shared_ptr Merge(const typename This::shared_ptr&, const typename This::shared_ptr&)

  // IMU bias keys must be the same.
  if (f01->template key<5>() != f12->template key<5>())
    throw std::domain_error("ImuFactor::Merge: IMU bias keys must be the same");

  // expect intermediate pose, velocity keys to matchup.
  if (f01->template key<3>() != f12->template key<1>() || f01->template key<4>() != f12->template key<2>())
    throw std::domain_error(
        "ImuFactor::Merge: intermediate pose, velocity keys need to match up");

  // return new factor
  auto pim02 = This::Merge(f01->preintegratedMeasurements(), f12->preintegratedMeasurements());

  return std::make_shared<This>( // `This` is ImuFactorT<MethodPIMArg> (i.e. ImuFactorT<PIM>)
      f01->template key<1>(),  // P0
      f01->template key<2>(),  // V0
      f12->template key<3>(),  // P2
      f12->template key<4>(),  // V2
      f01->template key<5>(),  // B
      pim02);
  }

 private:
  /** Serialization function */
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    // NoiseModelFactor5 instead of NoiseModelFactorN for backward compatibility
    ar & boost::serialization::make_nvp("NoiseModelFactor5",
         boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(pim_);
  }
#endif
};
// class ImuFactorT

// For backward compatibility:
using ImuFactor = ImuFactorT<>;
 
// operator<< for ImuFactorT
template <class PIM>
GTSAM_EXPORT std::ostream& operator<<(std::ostream& os, const ImuFactorT<PIM>& f);

/**
 * ImuFactor2 is a ternary factor that uses NavStates rather than Pose/Velocity.
 * @ingroup navigation
 */
template <class PIM = PreintegratedImuMeasurements>
class GTSAM_EXPORT ImuFactor2T : public NoiseModelFactorN<NavState, NavState, imuBias::ConstantBias> {
private:

  typedef ImuFactor2T<PIM> This;
  typedef NoiseModelFactorN<NavState, NavState, imuBias::ConstantBias> Base;

  PIM pim_;

public:

  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /** Default constructor - only use for serialization */
  ImuFactor2T() {}

  /**
   * Constructor
   * @param state_i Previous state key
   * @param state_j Current state key
   * @param bias    Previous bias key
   */
  ImuFactor2T(Key state_i, Key state_j, Key bias,
             const PIM& preintegratedMeasurements)
      : Base(noiseModel::Gaussian::Covariance(preintegratedMeasurements.preintMeasCov()),
             state_i, state_j, bias),
        pim_(preintegratedMeasurements) {}


  ~ImuFactor2T() override {
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

  /** implement functions needed to derive from Factor */

  /// vector of errors
  Vector evaluateError(const NavState& state_i, const NavState& state_j,
                       const imuBias::ConstantBias& bias_i,  //
                       OptionalMatrixType H1, OptionalMatrixType H2,
                       OptionalMatrixType H3) const override;

private:

#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    // NoiseModelFactor3 instead of NoiseModelFactorN for backward compatibility
    ar & boost::serialization::make_nvp("NoiseModelFactor3",
         boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(pim_);
  }
#endif
};
// class ImuFactor2T

// For backward compatibility:
using ImuFactor2 = ImuFactor2T<>;

// operator<< for ImuFactor2T
template <class PIM>
GTSAM_EXPORT std::ostream& operator<<(std::ostream& os, const ImuFactor2T<PIM>& f);

template <class PreintegrationType>
struct traits<PreintegratedImuMeasurementsT<PreintegrationType>> : public Testable<PreintegratedImuMeasurementsT<PreintegrationType>> {};

template <class PIM>
struct traits<ImuFactorT<PIM>> : public Testable<ImuFactorT<PIM>> {};

template <class PIM>
struct traits<ImuFactor2T<PIM>> : public Testable<ImuFactor2T<PIM>> {};

} /// namespace gtsam
