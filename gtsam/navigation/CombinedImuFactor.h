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
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/navigation/PreintegrationBase.h>
#include <gtsam/navigation/ImuFactorBase.h>
#include <gtsam/base/debug.h>

namespace gtsam {

/**
 *
 * @addtogroup SLAM
 *
 * If you are using the factor, please cite:
 * L. Carlone, Z. Kira, C. Beall, V. Indelman, F. Dellaert, Eliminating
 * conditionally independent sets in factor graphs: a unifying perspective based
 * on smart factors, Int. Conf. on Robotics and Automation (ICRA), 2014.
 *
 ** REFERENCES:
 * [1] G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie Groups",
 *     Volume 2, 2008.
 * [2] T. Lupton and S.Sukkarieh, "Visual-Inertial-Aided Navigation for
 *     High-Dynamic Motion in Built Environments Without Initial Conditions",
 *     TRO, 28(1):61-76, 2012.
 * [3] L. Carlone, S. Williams, R. Roberts, "Preintegrated IMU factor:
 *     Computation of the Jacobian Matrices", Tech. Report, 2013.
 */

/**
 * CombinedImuFactor is a 6-ways factor involving previous state (pose and
 * velocity of the vehicle, as well as bias at previous time step), and current
 * state (pose, velocity, bias at current time step). Following the pre-
 * integration scheme proposed in [2], the CombinedImuFactor includes many IMU
 * measurements, which are "summarized" using the CombinedPreintegratedMeasurements
 * class. There are 3 main differences wrpt the ImuFactor class:
 * 1) The factor is 6-ways, meaning that it also involves both biases (previous
 *    and current time step).Therefore, the factor internally imposes the biases
 *    to be slowly varying; in particular, the matrices "biasAccCovariance" and
 *    "biasOmegaCovariance" described the random walk that models bias evolution.
 * 2) The preintegration covariance takes into account the noise in the bias
 *    estimate used for integration.
 * 3) The covariance matrix of the CombinedPreintegratedMeasurements preserves
 *    the correlation between the bias uncertainty and the preintegrated
 *    measurements uncertainty.
 */
class CombinedImuFactor: public NoiseModelFactor6<Pose3, Vector3, Pose3,
    Vector3, imuBias::ConstantBias, imuBias::ConstantBias>, public ImuFactorBase {
public:

  /**
   * CombinedPreintegratedMeasurements integrates the IMU measurements
   * (rotation rates and accelerations) and the corresponding covariance matrix.
   * The measurements are then used to build the CombinedImuFactor. Integration
   * is done incrementally (ideally, one integrates the measurement as soon as
   * it is received from the IMU) so as to avoid costly integration at time of
   * factor construction.
   */
  class CombinedPreintegratedMeasurements: public PreintegrationBase {

    friend class CombinedImuFactor;

  protected:

    Matrix3 biasAccCovariance_; ///< continuous-time "Covariance" describing accelerometer bias random walk
    Matrix3 biasOmegaCovariance_; ///< continuous-time "Covariance" describing gyroscope bias random walk
    Matrix6 biasAccOmegaInit_; ///< covariance of bias used for pre-integration

    Eigen::Matrix<double, 15, 15> preintMeasCov_; ///< Covariance matrix of the preintegrated measurements
    ///< COVARIANCE OF: [PreintPOSITION PreintVELOCITY PreintROTATION BiasAcc BiasOmega]
    ///< (first-order propagation from *measurementCovariance*). CombinedPreintegratedMeasurements also include the biases and keep the correlation
    ///< between the preintegrated measurements and the biases

  public:

    /**
     *  Default constructor, initializes the class with no measurements
     *  @param bias                       Current estimate of acceleration and rotation rate biases
     *  @param measuredAccCovariance      Covariance matrix of measuredAcc
     *  @param measuredOmegaCovariance    Covariance matrix of measured Angular Rate
     *  @param integrationErrorCovariance Covariance matrix of integration errors (velocity -> position)
     *  @param biasAccCovariance          Covariance matrix of biasAcc (random walk describing BIAS evolution)
     *  @param biasOmegaCovariance        Covariance matrix of biasOmega (random walk describing BIAS evolution)
     *  @param biasAccOmegaInit           Covariance of biasAcc & biasOmega when preintegrating measurements
     *  @param use2ndOrderIntegration     Controls the order of integration
     *  (if false: p(t+1) = p(t) + v(t) deltaT ; if true: p(t+1) = p(t) + v(t) deltaT + 0.5 * acc(t) deltaT^2)
     */
    CombinedPreintegratedMeasurements(const imuBias::ConstantBias& bias,
        const Matrix3& measuredAccCovariance,
        const Matrix3& measuredOmegaCovariance,
        const Matrix3& integrationErrorCovariance,
        const Matrix3& biasAccCovariance, const Matrix3& biasOmegaCovariance,
        const Matrix& biasAccOmegaInit, const bool use2ndOrderIntegration =
            false);

    /// print
    void print(const std::string& s = "Preintegrated Measurements:") const;

    /// equals
    bool equals(const CombinedPreintegratedMeasurements& expected, double tol =
        1e-9) const;

    /// Re-initialize CombinedPreintegratedMeasurements
    void resetIntegration();

    /**
     * Add a single IMU measurement to the preintegration.
     * @param measuredAcc Measured acceleration (in body frame, as given by the sensor)
     * @param measuredOmega Measured angular velocity (as given by the sensor)
     * @param deltaT Time interval between two consecutive IMU measurements
     * @param body_P_sensor Optional sensor frame (pose of the IMU in the body frame)
     */
    void integrateMeasurement(const Vector3& measuredAcc,
        const Vector3& measuredOmega, double deltaT,
        boost::optional<const Pose3&> body_P_sensor = boost::none,
        boost::optional<Matrix&> F_test = boost::none,
        boost::optional<Matrix&> G_test = boost::none);

    /// methods to access class variables
    Matrix preintMeasCov() const {
      return preintMeasCov_;
    }

  private:

    /// Serialization function
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(PreintegrationBase);
      ar & BOOST_SERIALIZATION_NVP(preintMeasCov_);
    }
  };

private:

  typedef CombinedImuFactor This;
  typedef NoiseModelFactor6<Pose3, Vector3, Pose3, Vector3,
      imuBias::ConstantBias, imuBias::ConstantBias> Base;

  CombinedPreintegratedMeasurements _PIM_;

public:

  /** Shorthand for a smart pointer to a factor */
#if !defined(_MSC_VER) && __GNUC__ == 4 && __GNUC_MINOR__ > 5
  typedef typename boost::shared_ptr<CombinedImuFactor> shared_ptr;
#else
  typedef boost::shared_ptr<CombinedImuFactor> shared_ptr;
#endif

  /** Default constructor - only use for serialization */
  CombinedImuFactor();

  /**
   * Constructor
   * @param pose_i Previous pose key
   * @param vel_i  Previous velocity key
   * @param pose_j Current pose key
   * @param vel_j  Current velocity key
   * @param bias_i Previous bias key
   * @param bias_j Current bias key
   * @param CombinedPreintegratedMeasurements CombinedPreintegratedMeasurements IMU measurements
   * @param gravity Gravity vector expressed in the global frame
   * @param omegaCoriolis Rotation rate of the global frame w.r.t. an inertial frame
   * @param body_P_sensor Optional pose of the sensor frame in the body frame
   * @param use2ndOrderCoriolis When true, the second-order term is used in the calculation of the Coriolis Effect
   */
  CombinedImuFactor(Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key bias_i,
      Key bias_j,
      const CombinedPreintegratedMeasurements& preintegratedMeasurements,
      const Vector3& gravity, const Vector3& omegaCoriolis,
      boost::optional<const Pose3&> body_P_sensor = boost::none,
      const bool use2ndOrderCoriolis = false);

  virtual ~CombinedImuFactor() {
  }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const;

  /** implement functions needed for Testable */

  /// print
  virtual void print(const std::string& s, const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const;

  /// equals
  virtual bool equals(const NonlinearFactor& expected, double tol = 1e-9) const;

  /** Access the preintegrated measurements. */

  const CombinedPreintegratedMeasurements& preintegratedMeasurements() const {
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

  /** @deprecated The following function has been deprecated, use PreintegrationBase::predict with the same signature instead */
  static void Predict(const Pose3& pose_i, const Vector3& vel_i, Pose3& pose_j,
      Vector3& vel_j, const imuBias::ConstantBias& bias_i,
      const CombinedPreintegratedMeasurements& PIM, const Vector3& gravity,
      const Vector3& omegaCoriolis, const bool use2ndOrderCoriolis = false,
      boost::optional<Vector3&> deltaPij_biascorrected_out = boost::none,
      boost::optional<Vector3&> deltaVij_biascorrected_out = boost::none) {
    PoseVelocityBias PVB(
        PIM.predict(pose_i, vel_i, bias_i, gravity, omegaCoriolis,
            use2ndOrderCoriolis, deltaPij_biascorrected_out,
            deltaVij_biascorrected_out));
    pose_j = PVB.pose;
    vel_j = PVB.velocity;
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("NoiseModelFactor6",
         boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(_PIM_);
    ar & BOOST_SERIALIZATION_NVP(gravity_);
    ar & BOOST_SERIALIZATION_NVP(omegaCoriolis_);
    ar & BOOST_SERIALIZATION_NVP(body_P_sensor_);
  }
};
// class CombinedImuFactor

typedef CombinedImuFactor::CombinedPreintegratedMeasurements CombinedImuFactorPreintegratedMeasurements;

} /// namespace gtsam
