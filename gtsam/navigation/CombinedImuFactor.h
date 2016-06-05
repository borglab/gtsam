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
 * [4] C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza, IMU Preintegration on
 *     Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation,
 *     Robotics: Science and Systems (RSS), 2015.
 */

/**
 * PreintegratedCombinedMeasurements integrates the IMU measurements
 * (rotation rates and accelerations) and the corresponding covariance matrix.
 * The measurements are then used to build the CombinedImuFactor. Integration
 * is done incrementally (ideally, one integrates the measurement as soon as
 * it is received from the IMU) so as to avoid costly integration at time of
 * factor construction.
 *
 * @addtogroup SLAM
 */
class PreintegratedCombinedMeasurements : public PreintegrationType {

public:

  /// Parameters for pre-integration:
  /// Usage: Create just a single Params and pass a shared pointer to the constructor
  struct Params : PreintegrationParams {
    Matrix3 biasAccCovariance;    ///< continuous-time "Covariance" describing accelerometer bias random walk
    Matrix3 biasOmegaCovariance;  ///< continuous-time "Covariance" describing gyroscope bias random walk
    Matrix6 biasAccOmegaInt;     ///< covariance of bias used for pre-integration

    /// See two named constructors below for good values of n_gravity in body frame
    Params(const Vector3& n_gravity) :
        PreintegrationParams(n_gravity), biasAccCovariance(I_3x3), biasOmegaCovariance(
            I_3x3), biasAccOmegaInt(I_6x6) {
    }

    // Default Params for a Z-down navigation frame, such as NED: gravity points along positive Z-axis
    static boost::shared_ptr<Params> MakeSharedD(double g = 9.81) {
      return boost::make_shared<Params>(Vector3(0, 0, g));
    }

    // Default Params for a Z-up navigation frame, such as ENU: gravity points along negative Z-axis
    static boost::shared_ptr<Params> MakeSharedU(double g = 9.81) {
      return boost::make_shared<Params>(Vector3(0, 0, -g));
    }

   private:
    /// Default constructor makes unitialized params struct
    Params() {}

    /** Serialization function */
    friend class boost::serialization::access;
    template <class ARCHIVE>
    void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
      ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(PreintegratedRotation::Params);
      ar& BOOST_SERIALIZATION_NVP(biasAccCovariance);
      ar& BOOST_SERIALIZATION_NVP(biasOmegaCovariance);
      ar& BOOST_SERIALIZATION_NVP(biasAccOmegaInt);
    }
  };

 protected:
  /* Covariance matrix of the preintegrated measurements
   * COVARIANCE OF: [PreintPOSITION PreintVELOCITY PreintROTATION BiasAcc BiasOmega]
   * (first-order propagation from *measurementCovariance*).
   * PreintegratedCombinedMeasurements also include the biases and keep the correlation
   * between the preintegrated measurements and the biases
   */
  Eigen::Matrix<double, 15, 15> preintMeasCov_;

  PreintegratedCombinedMeasurements() {}

  friend class CombinedImuFactor;

 public:
  /// @name Constructors
  /// @{

  /**
   *  Default constructor, initializes the class with no measurements
   *  @param bias Current estimate of acceleration and rotation rate biases
   */
  PreintegratedCombinedMeasurements(
      const boost::shared_ptr<Params>& p,
      const imuBias::ConstantBias& biasHat = imuBias::ConstantBias())
      : PreintegrationType(p, biasHat) {
    preintMeasCov_.setZero();
  }

  /// @}

  /// @name Basic utilities
  /// @{

  /// Re-initialize PreintegratedCombinedMeasurements
  void resetIntegration() override;

  /// const reference to params, shadows definition in base class
  Params& p() const { return *boost::static_pointer_cast<Params>(this->p_);}
  /// @}

  /// @name Access instance variables
  /// @{
  Matrix preintMeasCov() const { return preintMeasCov_; }
  /// @}

  /// @name Testable
  /// @{
  void print(const std::string& s = "Preintegrated Measurements:") const override;
  bool equals(const PreintegratedCombinedMeasurements& expected, double tol = 1e-9) const;
  /// @}

  /// @name Main functionality
  /// @{

  /**
   * Add a single IMU measurement to the preintegration.
   * @param measuredAcc Measured acceleration (in body frame, as given by the
   * sensor)
   * @param measuredOmega Measured angular velocity (as given by the sensor)
   * @param deltaT Time interval between two consecutive IMU measurements
   * @param body_P_sensor Optional sensor frame (pose of the IMU in the body
   * frame)
   */
  void integrateMeasurement(const Vector3& measuredAcc,
      const Vector3& measuredOmega, const double dt) override;

  /// @}

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V4
  /// deprecated constructor
  /// NOTE(frank): assumes Z-Down convention, only second order integration supported
  PreintegratedCombinedMeasurements(const imuBias::ConstantBias& biasHat,
      const Matrix3& measuredAccCovariance,
      const Matrix3& measuredOmegaCovariance,
      const Matrix3& integrationErrorCovariance,
      const Matrix3& biasAccCovariance, const Matrix3& biasOmegaCovariance,
      const Matrix6& biasAccOmegaInt, const bool use2ndOrderIntegration = true);
#endif

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(PreintegrationType);
    ar& BOOST_SERIALIZATION_NVP(preintMeasCov_);
  }
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
 * @addtogroup SLAM
 */
class CombinedImuFactor: public NoiseModelFactor6<Pose3, Vector3, Pose3,
    Vector3, imuBias::ConstantBias, imuBias::ConstantBias> {
public:

private:

  typedef CombinedImuFactor This;
  typedef NoiseModelFactor6<Pose3, Vector3, Pose3, Vector3,
      imuBias::ConstantBias, imuBias::ConstantBias> Base;

  PreintegratedCombinedMeasurements _PIM_;

  /** Default constructor - only use for serialization */
  CombinedImuFactor() {}

public:

  /** Shorthand for a smart pointer to a factor */
#if !defined(_MSC_VER) && __GNUC__ == 4 && __GNUC_MINOR__ > 5
  typedef typename boost::shared_ptr<CombinedImuFactor> shared_ptr;
#else
  typedef boost::shared_ptr<CombinedImuFactor> shared_ptr;
#endif

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

  virtual ~CombinedImuFactor() {}

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const;

  /** implement functions needed for Testable */

  /// print
  virtual void print(const std::string& s, const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const;

  /// equals
  virtual bool equals(const NonlinearFactor& expected, double tol = 1e-9) const;

  /** Access the preintegrated measurements. */

  const PreintegratedCombinedMeasurements& preintegratedMeasurements() const {
    return _PIM_;
  }

  /** implement functions needed to derive from Factor */

  /// vector of errors
  Vector evaluateError(const Pose3& pose_i, const Vector3& vel_i,
      const Pose3& pose_j, const Vector3& vel_j,
      const imuBias::ConstantBias& bias_i, const imuBias::ConstantBias& bias_j,
      boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 =
          boost::none, boost::optional<Matrix&> H3 = boost::none,
      boost::optional<Matrix&> H4 = boost::none, boost::optional<Matrix&> H5 =
          boost::none, boost::optional<Matrix&> H6 = boost::none) const;

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V4
  /// @deprecated typename
  typedef gtsam::PreintegratedCombinedMeasurements CombinedPreintegratedMeasurements;

  /// @deprecated constructor
  CombinedImuFactor(Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key bias_i,
                    Key bias_j, const CombinedPreintegratedMeasurements& pim,
                    const Vector3& n_gravity, const Vector3& omegaCoriolis,
                    const boost::optional<Pose3>& body_P_sensor = boost::none,
                    const bool use2ndOrderCoriolis = false);

  // @deprecated use PreintegrationBase::predict
  static void Predict(const Pose3& pose_i, const Vector3& vel_i, Pose3& pose_j,
                      Vector3& vel_j, const imuBias::ConstantBias& bias_i,
                      CombinedPreintegratedMeasurements& pim,
                      const Vector3& n_gravity, const Vector3& omegaCoriolis,
                      const bool use2ndOrderCoriolis = false);
#endif

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("NoiseModelFactor6",
         boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(_PIM_);
  }
};
// class CombinedImuFactor

} /// namespace gtsam
