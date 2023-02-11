/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  CombinedImuFactor.h
 *  @author Luca Carlone
 *  @author Stephen Williams
 *  @author Richard Roberts
 *  @author Vadim Indelman
 *  @author David Jensen
 *  @author Frank Dellaert
 *  @author Varun Agrawal
 **/

#pragma once

/* GTSAM includes */
#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/TangentPreintegration.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>

namespace gtsam {

#ifdef GTSAM_TANGENT_PREINTEGRATION
typedef TangentPreintegration PreintegrationType;
#else
typedef ManifoldPreintegration PreintegrationType;
#endif

/*
 * If you are using the factor, please cite:
 * L. Carlone, Z. Kira, C. Beall, V. Indelman, F. Dellaert, Eliminating
 * conditionally independent sets in factor graphs: a unifying perspective based
 * on smart factors, Int. Conf. on Robotics and Automation (ICRA), 2014.
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
 * [4] C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza, IMU Preintegration on
 *     Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation,
 *     Robotics: Science and Systems (RSS), 2015.
 */

/// Parameters for pre-integration using PreintegratedCombinedMeasurements:
/// Usage: Create just a single Params and pass a shared pointer to the constructor
struct GTSAM_EXPORT PreintegrationCombinedParams : PreintegrationParams {
  Matrix3 biasAccCovariance;    ///< continuous-time "Covariance" describing accelerometer bias random walk
  Matrix3 biasOmegaCovariance;  ///< continuous-time "Covariance" describing gyroscope bias random walk
  Matrix6 biasAccOmegaInt;     ///< covariance of bias used as initial estimate.

  /// Default constructor makes uninitialized params struct.
  /// Used for serialization.
  PreintegrationCombinedParams()
      : biasAccCovariance(I_3x3),
        biasOmegaCovariance(I_3x3),
        biasAccOmegaInt(I_6x6) {}

  /// See two named constructors below for good values of n_gravity in body frame
  PreintegrationCombinedParams(const Vector3& _n_gravity) :
    PreintegrationParams(_n_gravity), biasAccCovariance(I_3x3),
    biasOmegaCovariance(I_3x3), biasAccOmegaInt(I_6x6) {

  }

  // Default Params for a Z-down navigation frame, such as NED: gravity points along positive Z-axis
  static std::shared_ptr<PreintegrationCombinedParams> MakeSharedD(double g = 9.81) {
    return std::shared_ptr<PreintegrationCombinedParams>(new PreintegrationCombinedParams(Vector3(0, 0, g)));
  }

  // Default Params for a Z-up navigation frame, such as ENU: gravity points along negative Z-axis
  static std::shared_ptr<PreintegrationCombinedParams> MakeSharedU(double g = 9.81) {
    return std::shared_ptr<PreintegrationCombinedParams>(new PreintegrationCombinedParams(Vector3(0, 0, -g)));
  }

  void print(const std::string& s="") const override;
  bool equals(const PreintegratedRotationParams& other, double tol) const override;

  void setBiasAccCovariance(const Matrix3& cov) { biasAccCovariance=cov; }
  void setBiasOmegaCovariance(const Matrix3& cov) { biasOmegaCovariance=cov; }
  void setBiasAccOmegaInit(const Matrix6& cov) { biasAccOmegaInt=cov; }
  
  const Matrix3& getBiasAccCovariance() const { return biasAccCovariance; }
  const Matrix3& getBiasOmegaCovariance() const { return biasOmegaCovariance; }
  const Matrix6& getBiasAccOmegaInit() const { return biasAccOmegaInt; }
  
private:

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    namespace bs = ::boost::serialization;
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(PreintegrationParams);
    ar & BOOST_SERIALIZATION_NVP(biasAccCovariance);
    ar & BOOST_SERIALIZATION_NVP(biasOmegaCovariance);
    ar & BOOST_SERIALIZATION_NVP(biasAccOmegaInt);
  }
#endif

public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * PreintegratedCombinedMeasurements integrates the IMU measurements
 * (rotation rates and accelerations) and the corresponding covariance matrix.
 * The measurements are then used to build the CombinedImuFactor. Integration
 * is done incrementally (ideally, one integrates the measurement as soon as
 * it is received from the IMU) so as to avoid costly integration at time of
 * factor construction.
 *
 * @ingroup navigation
 */
class GTSAM_EXPORT PreintegratedCombinedMeasurements : public PreintegrationType {

public:
  typedef PreintegrationCombinedParams Params;

 protected:
  /* Covariance matrix of the preintegrated measurements
   * COVARIANCE OF: [PreintROTATION PreintPOSITION PreintVELOCITY BiasAcc BiasOmega]
   * (first-order propagation from *measurementCovariance*).
   * PreintegratedCombinedMeasurements also include the biases and keep the correlation
   * between the preintegrated measurements and the biases
   */
  Eigen::Matrix<double, 15, 15> preintMeasCov_;

  friend class CombinedImuFactor;

 public:
  /// @name Constructors
  /// @{

  /// Default constructor only for serialization and wrappers
  PreintegratedCombinedMeasurements() {
    preintMeasCov_.setZero();
  }

  /**
   *  Default constructor, initializes the class with no measurements
   *  @param p       Parameters, typically fixed in a single application
   *  @param biasHat Current estimate of acceleration and rotation rate biases
   */
  PreintegratedCombinedMeasurements(
      const std::shared_ptr<Params>& p,
      const imuBias::ConstantBias& biasHat = imuBias::ConstantBias())
      : PreintegrationType(p, biasHat) {
    preintMeasCov_.setZero();
  }

  /**
  *  Construct preintegrated directly from members: base class and preintMeasCov
  *  @param base               PreintegrationType instance
  *  @param preintMeasCov      Covariance matrix used in noise model.
  */
  PreintegratedCombinedMeasurements(const PreintegrationType& base, const Eigen::Matrix<double, 15, 15>& preintMeasCov)
     : PreintegrationType(base),
       preintMeasCov_(preintMeasCov) {
  }

  /// Virtual destructor
  ~PreintegratedCombinedMeasurements() override {}

  /// @}

  /// @name Basic utilities
  /// @{

  /// Re-initialize PreintegratedCombinedMeasurements
  void resetIntegration() override;

  /// const reference to params, shadows definition in base class
  Params& p() const { return *std::static_pointer_cast<Params>(this->p_); }
  /// @}

  /// @name Access instance variables
  /// @{
  /// Return pre-integrated measurement covariance
  Matrix preintMeasCov() const { return preintMeasCov_; }
  /// @}

  /// @name Testable
  /// @{
  /// print
  void print(const std::string& s = "Preintegrated Measurements:") const override;
  /// equals
  bool equals(const PreintegratedCombinedMeasurements& expected,
              double tol = 1e-9) const;
  /// @}


  /// @name Main functionality
  /// @{

  /**
   * Add a single IMU measurement to the preintegration.
   * Both accelerometer and gyroscope measurements are taken to be in the sensor
   * frame and conversion to the body frame is handled by `body_P_sensor` in
   * `PreintegrationParams`.
   *
   * @param measuredAcc Measured acceleration (as given by the sensor)
   * @param measuredOmega Measured angular velocity (as given by the sensor)
   * @param dt Time interval between two consecutive IMU measurements
   */
  void integrateMeasurement(const Vector3& measuredAcc,
      const Vector3& measuredOmega, const double dt) override;

  /// @}

 private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION  ///
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    namespace bs = ::boost::serialization;
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(PreintegrationType);
    ar& BOOST_SERIALIZATION_NVP(preintMeasCov_);
  }
#endif

public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * CombinedImuFactor is a 6-ways factor involving previous state (pose and
 * velocity of the vehicle, as well as bias at previous time step), and current
 * state (pose, velocity, bias at current time step). Following the pre-
 * integration scheme proposed in [2], the CombinedImuFactor includes many IMU
 * measurements, which are "summarized" using the PreintegratedCombinedMeasurements
 * class. There are 3 main differences wrpt the ImuFactor class:
 * 1) The factor is 6-ways, meaning that it also involves both biases (previous
 *    and current time step).Therefore, the factor internally imposes the biases
 *    to be slowly varying; in particular, the matrices "biasAccCovariance" and
 *    "biasOmegaCovariance" described the random walk that models bias evolution.
 * 2) The preintegration covariance takes into account the noise in the bias
 *    estimate used for integration.
 * 3) The covariance matrix of the PreintegratedCombinedMeasurements preserves
 *    the correlation between the bias uncertainty and the preintegrated
 *    measurements uncertainty.
 *
 * @ingroup navigation
 */
class GTSAM_EXPORT CombinedImuFactor: public NoiseModelFactorN<Pose3, Vector3, Pose3,
    Vector3, imuBias::ConstantBias, imuBias::ConstantBias> {
public:

private:

  typedef CombinedImuFactor This;
  typedef NoiseModelFactorN<Pose3, Vector3, Pose3, Vector3,
      imuBias::ConstantBias, imuBias::ConstantBias> Base;

  PreintegratedCombinedMeasurements _PIM_;

public:

  // Provide access to Matrix& version of evaluateError:
  using Base::evaluateError;

  /** Shorthand for a smart pointer to a factor */
#if !defined(_MSC_VER) && __GNUC__ == 4 && __GNUC_MINOR__ > 5
  typedef typename std::shared_ptr<CombinedImuFactor> shared_ptr;
#else
  typedef std::shared_ptr<CombinedImuFactor> shared_ptr;
#endif

  /** Default constructor - only use for serialization */
  CombinedImuFactor() {}

  /**
   * Constructor
   * @param pose_i Previous pose key
   * @param vel_i  Previous velocity key
   * @param pose_j Current pose key
   * @param vel_j  Current velocity key
   * @param bias_i Previous bias key
   * @param bias_j Current bias key
   * @param PreintegratedCombinedMeasurements Combined IMU measurements
   */
  CombinedImuFactor(
      Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key bias_i, Key bias_j,
      const PreintegratedCombinedMeasurements& preintegratedMeasurements);

  ~CombinedImuFactor() override {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override;

  /** implement functions needed for Testable */

  /// @name Testable
  /// @{
  GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os,
                                               const CombinedImuFactor&);
  /// print
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;

  /// equals
  bool equals(const NonlinearFactor& expected, double tol = 1e-9) const override;
  /// @}

  /** Access the preintegrated measurements. */

  const PreintegratedCombinedMeasurements& preintegratedMeasurements() const {
    return _PIM_;
  }

  /** implement functions needed to derive from Factor */

  /// vector of errors
  Vector evaluateError(const Pose3& pose_i, const Vector3& vel_i,
      const Pose3& pose_j, const Vector3& vel_j,
      const imuBias::ConstantBias& bias_i, const imuBias::ConstantBias& bias_j,
      OptionalMatrixType H1, OptionalMatrixType H2, 
      OptionalMatrixType H3, OptionalMatrixType H4, 
      OptionalMatrixType H5, OptionalMatrixType H6) const override;

 private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    // NoiseModelFactor6 instead of NoiseModelFactorN for backward compatibility
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor6", boost::serialization::base_object<Base>(*this));
    ar& BOOST_SERIALIZATION_NVP(_PIM_);
  }
#endif

public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};
// class CombinedImuFactor

template <>
struct traits<PreintegrationCombinedParams>
    : public Testable<PreintegrationCombinedParams> {};

template <>
struct traits<PreintegratedCombinedMeasurements>
    : public Testable<PreintegratedCombinedMeasurements> {};

template <>
struct traits<CombinedImuFactor> : public Testable<CombinedImuFactor> {};

}  // namespace gtsam
