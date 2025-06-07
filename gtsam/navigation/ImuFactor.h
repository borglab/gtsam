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
typedef TangentPreintegration DefaultPreintegrationBackend;
#else
typedef ManifoldPreintegration DefaultPreintegrationBackend;
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
 * Interface to support user PIM specification for ImuFactor & ImuFactor2.
 * This class is the minimum interface required for an ImuFactor to interact
 * with its underlying PreintegratedImuMeasurements, which may be based on
 * ManifoldPreintegration, TangentPreintegration, etc.
 */
class PreintegratedImuMeasurementsInterface {
 public:
  virtual Vector9 computeError(const NavState& state_i, const NavState& state_j,
    const imuBias::ConstantBias& bias_i,
    OptionalJacobian<9, 9> H1, OptionalJacobian<9, 9> H2,
    OptionalJacobian<9, 6> H3) const = 0;

  virtual Vector9 computeErrorAndJacobians(const Pose3& pose_i, const Vector3& vel_i,
    const Pose3& pose_j, const Vector3& vel_j,
    const imuBias::ConstantBias& bias_i, 
    OptionalJacobian<9, 6> H1 = {}, OptionalJacobian<9, 3> H2 = {},
    OptionalJacobian<9, 6> H3 = {}, OptionalJacobian<9, 3> H4 = {}, 
    OptionalJacobian<9, 6> H5 = {}) const = 0;

  virtual Matrix preintMeasCov() const = 0;

  virtual void print(const std::string& s) const = 0;
  virtual bool equals(const PreintegratedImuMeasurementsInterface& expected, double tol) const = 0;
  
  virtual ~PreintegratedImuMeasurementsInterface() = default;
  
  /**
  * @brief Create a deep copy of this object and return a shared pointer.
  * Named deepCopy to avoid conflict with TangentPreintegration.clone().
  */
  virtual std::shared_ptr<const PreintegratedImuMeasurementsInterface> deepCopy() const = 0;
};

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
template <class PreintegrationBackend>
class GTSAM_EXPORT PreintegratedImuMeasurementsT: public PreintegrationBackend, 
                                                  public virtual PreintegratedImuMeasurementsInterface {
 
  friend class ImuFactorT;
  friend class ImuFactor2T;

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
      PreintegrationBackend(p, biasHat) {
    this->resetIntegration();
  }

/**
  *  Construct preintegrated directly from members: base class and preintMeasCov
  *  @param base               PreintegrationBackend instance
  *  @param preintMeasCov      Covariance matrix used in noise model.
  */
  PreintegratedImuMeasurementsT(const PreintegrationBackend& base, const Matrix9& preintMeasCov)
     : PreintegrationBackend(base),
       preintMeasCov_(preintMeasCov) {
    this->PreintegrationBackend::resetIntegration();
  }
 
  /// Virtual destructor
  ~PreintegratedImuMeasurementsT() override {
  }

  /// print
  void print(const std::string& s = "Preintegrated Measurements:") const override;

  /// equals
  bool equalsConcrete(const PreintegratedImuMeasurementsT<PreintegrationBackend>& expected, double tol = 1e-9) const;
  bool equals(const PreintegratedImuMeasurementsInterface& other_interface, double tol = 1e-9) const override;

  /// Re-initialize PreintegratedImuMeasurements
  void resetIntegration() override;
  
  /// Create a deep copy of this object, wrapped in a shared_ptr.
  std::shared_ptr<const PreintegratedImuMeasurementsInterface> deepCopy() const override {
    return std::make_shared<const PreintegratedImuMeasurementsT<PreintegrationBackend>>(*this);
  }

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

  /// Redeclare compute functions for override
  Vector9 computeError(const NavState& state_i, const NavState& state_j,
      const imuBias::ConstantBias& bias_i,
      OptionalJacobian<9, 9> H1, OptionalJacobian<9, 9> H2,
      OptionalJacobian<9, 6> H3) const override {
    return PreintegrationBackend::computeError(
      state_i, state_j, bias_i, H1, H2, H3);
  }
  Vector9 computeErrorAndJacobians(const Pose3& pose_i, const Vector3& vel_i,
      const Pose3& pose_j, const Vector3& vel_j,
      const imuBias::ConstantBias& bias_i, 
      OptionalJacobian<9, 6> H1 = {}, OptionalJacobian<9, 3> H2 = {},
      OptionalJacobian<9, 6> H3 = {}, OptionalJacobian<9, 3> H4 = {}, 
      OptionalJacobian<9, 6> H5 = {}) const override {
    return PreintegrationBackend::computeErrorAndJacobians(
      pose_i, vel_i, pose_j, vel_j, bias_i, H1, H2, H3, H4, H5);
  }

  /// Merge in a different set of measurements and update bias derivatives accordingly
  /// This method is specific to TangentPreintegration backend.
  template <typename PB = PreintegrationBackend,
             // This method is only callable when PreintegrationBackend is TangentPreintegration.
             typename = typename std::enable_if<std::is_same<PB, TangentPreintegration>::value>::type>
  void mergeWith(const PreintegratedImuMeasurementsT<TangentPreintegration>& pim12, Matrix9* H1, Matrix9* H2) {
    // The `this->PreintegrationBackend::mergeWith` implies calling TangentPreintegration's mergeWith.
    // Since pim12 is PreintegratedImuMeasurementsT<TangentPreintegration>, it is a TangentPreintegration.
    this->PreintegrationBackend::mergeWith(pim12, H1, H2);
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
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(PreintegrationBackend);
    ar & BOOST_SERIALIZATION_NVP(preintMeasCov_);
  }
#endif
};

// For backward compatibility (so that the compiler flag GTSAM_TANGENT_PREINTEGRATION still
// controls which class PreintegratedImuMeasurements uses):
typedef PreintegratedImuMeasurementsT<DefaultPreintegrationBackend> PreintegratedImuMeasurements;

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
class GTSAM_EXPORT ImuFactor: public NoiseModelFactorN<Pose3, Vector3, Pose3, Vector3,
    imuBias::ConstantBias> {
private:

  typedef ImuFactor This;
  typedef NoiseModelFactorN<Pose3, Vector3, Pose3, Vector3,
      imuBias::ConstantBias> Base;

  std::shared_ptr<const PreintegratedImuMeasurementsInterface> _PIM_;

public:

  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /** Shorthand for a smart pointer to a factor */
#if !defined(_MSC_VER) && __GNUC__ == 4 && __GNUC_MINOR__ > 5
  typedef typename std::shared_ptr<ImuFactor> shared_ptr;
#else
  typedef std::shared_ptr<ImuFactor> shared_ptr;
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
    const std::shared_ptr<const PreintegratedImuMeasurementsInterface>& preintegratedMeasurements);

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

  /** 
   * Access the preintegrated measurements. 
   * User is responsible for casting to their underlying PIM type.
   */
  const std::shared_ptr<const PreintegratedImuMeasurementsInterface> preintegratedMeasurements() const {
    return _PIM_;
  }

  /** implement functions needed to derive from Factor */

  /// vector of errors
  Vector evaluateError(const Pose3& pose_i, const Vector3& vel_i,
      const Pose3& pose_j, const Vector3& vel_j,
      const imuBias::ConstantBias& bias_i, OptionalMatrixType H1, OptionalMatrixType H2,
      OptionalMatrixType H3, OptionalMatrixType H4, OptionalMatrixType H5) const override;

  /// Merge two pre-integrated measurement classes
  static PreintegratedImuMeasurementsT<TangentPreintegration> Merge(
    const PreintegratedImuMeasurementsT<TangentPreintegration>& pim01,
    const PreintegratedImuMeasurementsT<TangentPreintegration>& pim12);

  /// Merge two factors
  static shared_ptr Merge(const shared_ptr& f01, const shared_ptr& f12);

 private:
  /** Serialization function */
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    // NoiseModelFactor5 instead of NoiseModelFactorN for backward compatibility
    ar & boost::serialization::make_nvp("NoiseModelFactor5",
         boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(_PIM_);
  }
#endif
};
// class ImuFactor

/**
 * ImuFactor2 is a ternary factor that uses NavStates rather than Pose/Velocity.
 * @ingroup navigation
 */
class GTSAM_EXPORT ImuFactor2 : public NoiseModelFactorN<NavState, NavState, imuBias::ConstantBias> {
private:

  typedef ImuFactor2 This;
  typedef NoiseModelFactorN<NavState, NavState, imuBias::ConstantBias> Base;

  std::shared_ptr<const PreintegratedImuMeasurementsInterface> _PIM_;

public:

  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /** Default constructor - only use for serialization */
  ImuFactor2() {}

  /**
   * Constructor
   * @param state_i Previous state key
   * @param state_j Current state key
   * @param bias    Previous bias key
   */
  ImuFactor2(Key state_i, Key state_j, Key bias,
    std::shared_ptr<const PreintegratedImuMeasurementsInterface> preintegratedMeasurements);

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

  const std::shared_ptr<const PreintegratedImuMeasurementsInterface> preintegratedMeasurements() const {
    return _PIM_;
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
    ar & BOOST_SERIALIZATION_NVP(_PIM_);
  }
#endif
};
// class ImuFactor2

template <class PreintegrationBackend>
struct traits<PreintegratedImuMeasurementsT<PreintegrationBackend>> : public Testable<PreintegratedImuMeasurementsT<PreintegrationBackend>> {};

template <>
struct traits<ImuFactor> : public Testable<ImuFactor> {};

template <>
struct traits<ImuFactor2> : public Testable<ImuFactor2> {};

} /// namespace gtsam
