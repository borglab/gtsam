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

namespace gtsam {

#ifdef GTSAM_TANGENT_PREINTEGRATION
typedef TangentPreintegration PreintegrationType;
#else
typedef ManifoldPreintegration PreintegrationType;
#endif

/*
 * If you are using the factor, please cite:
 * L. Carlone, Z. Kira, C. Beall, V. Indelman, F. Dellaert, "Eliminating
 * conditionally independent sets in factor graphs: a unifying perspective based
 * on smart factors", Int. Conf. on Robotics and Automation (ICRA), 2014.
 *
 * C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza, "IMU Preintegration on
 * Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation",
 * Robotics: Science and Systems (RSS), 2015.
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
 * [4] C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza, "IMU Preintegration on
 *     Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation",
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
class GTSAM_EXPORT PreintegratedImuMeasurements: public PreintegrationType {

  friend class ImuFactor;
  friend class ImuFactor2;

protected:

  Matrix9 preintMeasCov_; ///< COVARIANCE OF: [PreintROTATION PreintPOSITION PreintVELOCITY]
  ///< (first-order propagation from *measurementCovariance*).

public:

  /// Default constructor for serialization and wrappers
  PreintegratedImuMeasurements() {
    preintMeasCov_.setZero();
  }

 /**
   *  Constructor, initializes the class with no measurements
   *  @param p       Parameters, typically fixed in a single application
   *  @param biasHat Current estimate of acceleration and rotation rate biases
   */
  PreintegratedImuMeasurements(const boost::shared_ptr<PreintegrationParams>& p,
      const imuBias::ConstantBias& biasHat = imuBias::ConstantBias()) :
      PreintegrationType(p, biasHat) {
    preintMeasCov_.setZero();
  }

/**
  *  Construct preintegrated directly from members: base class and preintMeasCov
  *  @param base               PreintegrationType instance
  *  @param preintMeasCov      Covariance matrix used in noise model.
  */
  PreintegratedImuMeasurements(const PreintegrationType& base, const Matrix9& preintMeasCov)
     : PreintegrationType(base),
       preintMeasCov_(preintMeasCov) {
  }

  /// Virtual destructor
  ~PreintegratedImuMeasurements() override {
  }

  /// print
  void print(const std::string& s = "Preintegrated Measurements:") const override;

  /// equals
  bool equals(const PreintegratedImuMeasurements& expected, double tol = 1e-9) const;

  /// Re-initialize PreintegratedIMUMeasurements
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

#ifdef GTSAM_TANGENT_PREINTEGRATION
  /// Merge in a different set of measurements and update bias derivatives accordingly
  void mergeWith(const PreintegratedImuMeasurements& pim, Matrix9* H1, Matrix9* H2);
#endif

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    namespace bs = ::boost::serialization;
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(PreintegrationType);
    ar & BOOST_SERIALIZATION_NVP(preintMeasCov_);
  }
};

/**
 * ImuFactor is a 5-ways factor involving previous state (pose and velocity of
 * the vehicle at previous time step), current state (pose and velocity at
 * current time step), and the bias estimate. Following the preintegration
 * scheme proposed in [2], the ImuFactor includes many IMU measurements, which
 * are "summarized" using the PreintegratedIMUMeasurements class.
 * Note that this factor does not model "temporal consistency" of the biases
 * (which are usually slowly varying quantities), which is up to the caller.
 * See also CombinedImuFactor for a class that does this for you.
 *
 * @ingroup navigation
 */
class GTSAM_EXPORT ImuFactor: public NoiseModelFactor5<Pose3, Vector3, Pose3, Vector3,
    imuBias::ConstantBias> {
private:

  typedef ImuFactor This;
  typedef NoiseModelFactor5<Pose3, Vector3, Pose3, Vector3,
      imuBias::ConstantBias> Base;

  PreintegratedImuMeasurements _PIM_;

public:

  /** Shorthand for a smart pointer to a factor */
#if !defined(_MSC_VER) && __GNUC__ == 4 && __GNUC_MINOR__ > 5
  typedef typename boost::shared_ptr<ImuFactor> shared_ptr;
#else
  typedef boost::shared_ptr<ImuFactor> shared_ptr;
#endif

  /** Default constructor - only use for serialization */
  ImuFactor() {}

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
  ImuFactor(Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key bias,
      const PreintegratedImuMeasurements& preintegratedMeasurements);

  ~ImuFactor() override {
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override;

  /// @name Testable
  /// @{
  GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os, const ImuFactor&);
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;
  bool equals(const NonlinearFactor& expected, double tol = 1e-9) const override;
  /// @}

  /** Access the preintegrated measurements. */

  const PreintegratedImuMeasurements& preintegratedMeasurements() const {
    return _PIM_;
  }

  /** implement functions needed to derive from Factor */

  /// vector of errors
  Vector evaluateError(const Pose3& pose_i, const Vector3& vel_i,
      const Pose3& pose_j, const Vector3& vel_j,
      const imuBias::ConstantBias& bias_i, boost::optional<Matrix&> H1 =
          boost::none, boost::optional<Matrix&> H2 = boost::none,
      boost::optional<Matrix&> H3 = boost::none, boost::optional<Matrix&> H4 =
          boost::none, boost::optional<Matrix&> H5 = boost::none) const override;

#ifdef GTSAM_TANGENT_PREINTEGRATION
  /// Merge two pre-integrated measurement classes
  static PreintegratedImuMeasurements Merge(
      const PreintegratedImuMeasurements& pim01,
      const PreintegratedImuMeasurements& pim12);

  /// Merge two factors
  static shared_ptr Merge(const shared_ptr& f01, const shared_ptr& f12);
#endif

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("NoiseModelFactor5",
         boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(_PIM_);
  }
};
// class ImuFactor

/**
 * ImuFactor2 is a ternary factor that uses NavStates rather than Pose/Velocity.
 * @ingroup navigation
 */
class GTSAM_EXPORT ImuFactor2 : public NoiseModelFactor3<NavState, NavState, imuBias::ConstantBias> {
private:

  typedef ImuFactor2 This;
  typedef NoiseModelFactor3<NavState, NavState, imuBias::ConstantBias> Base;

  PreintegratedImuMeasurements _PIM_;

public:

  /** Default constructor - only use for serialization */
  ImuFactor2() {}

  /**
   * Constructor
   * @param state_i Previous state key
   * @param state_j Current state key
   * @param bias    Previous bias key
   */
  ImuFactor2(Key state_i, Key state_j, Key bias,
             const PreintegratedImuMeasurements& preintegratedMeasurements);

  ~ImuFactor2() override {
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override;

  /// @name Testable
  /// @{
  GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os, const ImuFactor2&);
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;
  bool equals(const NonlinearFactor& expected, double tol = 1e-9) const override;
  /// @}

  /** Access the preintegrated measurements. */

  const PreintegratedImuMeasurements& preintegratedMeasurements() const {
    return _PIM_;
  }

  /** implement functions needed to derive from Factor */

  /// vector of errors
  Vector evaluateError(const NavState& state_i, const NavState& state_j,
                       const imuBias::ConstantBias& bias_i,  //
                       boost::optional<Matrix&> H1 = boost::none,
                       boost::optional<Matrix&> H2 = boost::none,
                       boost::optional<Matrix&> H3 = boost::none) const override;

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("NoiseModelFactor3",
         boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(_PIM_);
  }
};
// class ImuFactor2

template <>
struct traits<PreintegratedImuMeasurements> : public Testable<PreintegratedImuMeasurements> {};

template <>
struct traits<ImuFactor> : public Testable<ImuFactor> {};

template <>
struct traits<ImuFactor2> : public Testable<ImuFactor2> {};

} /// namespace gtsam
