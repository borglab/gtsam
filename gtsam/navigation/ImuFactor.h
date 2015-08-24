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
#include <gtsam/navigation/PreintegrationBase.h>
#include <gtsam/base/debug.h>

namespace gtsam {

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
 * [4] C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza, "IMU Preintegration on
 *     Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation",
 *     Robotics: Science and Systems (RSS), 2015.
 */

/**
 * PreintegratedIMUMeasurements accumulates (integrates) the IMU measurements
 * (rotation rates and accelerations) and the corresponding covariance matrix.
 * The measurements are then used to build the Preintegrated IMU factor.
 * Integration is done incrementally (ideally, one integrates the measurement
 * as soon as it is received from the IMU) so as to avoid costly integration
 * at time of factor construction.
 *
 * @addtogroup SLAM
 */
class PreintegratedImuMeasurements: public PreintegrationBase {

  friend class ImuFactor;

protected:

  Matrix9 preintMeasCov_; ///< COVARIANCE OF: [PreintPOSITION PreintVELOCITY PreintROTATION]
  ///< (first-order propagation from *measurementCovariance*).

  /// Default constructor for serialization
  PreintegratedImuMeasurements() {}

public:

 /**
   *  Constructor, initializes the class with no measurements
   *  @param bias Current estimate of acceleration and rotation rate biases
   *  @param p    Parameters, typically fixed in a single application
   */
  PreintegratedImuMeasurements(const boost::shared_ptr<Params>& p,
      const imuBias::ConstantBias& biasHat) :
      PreintegrationBase(p, biasHat) {
    preintMeasCov_.setZero();
  }

  /// print
  void print(const std::string& s = "Preintegrated Measurements:") const;

  /// equals
  bool equals(const PreintegratedImuMeasurements& expected,
      double tol = 1e-9) const;

  /// Re-initialize PreintegratedIMUMeasurements
  void resetIntegration();

  /**
   * Add a single IMU measurement to the preintegration.
   * @param measuredAcc Measured acceleration (in body frame, as given by the sensor)
   * @param measuredOmega Measured angular velocity (as given by the sensor)
   * @param dt Time interval between this and the last IMU measurement
   */
  void integrateMeasurement(const Vector3& measuredAcc,
      const Vector3& measuredOmega, double dt);

  /// Return pre-integrated measurement covariance
  Matrix preintMeasCov() const { return preintMeasCov_; }

  /// @deprecated constructor
  /// NOTE(frank): assumes Z-Down convention, only second order integration supported
  PreintegratedImuMeasurements(const imuBias::ConstantBias& biasHat,
      const Matrix3& measuredAccCovariance,
      const Matrix3& measuredOmegaCovariance,
      const Matrix3& integrationErrorCovariance,
      bool use2ndOrderIntegration = true);

  /// @deprecated version of integrateMeasurement with body_P_sensor
  /// Use parameters instead
  void integrateMeasurement(const Vector3& measuredAcc,
      const Vector3& measuredOmega, double dt,
      boost::optional<Pose3> body_P_sensor);

private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(PreintegrationBase);
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
 * @addtogroup SLAM
 */
class ImuFactor: public NoiseModelFactor5<Pose3, Vector3, Pose3, Vector3,
    imuBias::ConstantBias> {
public:

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
  ImuFactor();

  /**
   * Constructor
   * @param pose_i Previous pose key
   * @param vel_i  Previous velocity key
   * @param pose_j Current pose key
   * @param vel_j  Current velocity key
   * @param bias   Previous bias key
   */
  ImuFactor(Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key bias,
      const PreintegratedImuMeasurements& preintegratedMeasurements);

  virtual ~ImuFactor() {
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

  const PreintegratedImuMeasurements& preintegratedMeasurements() const {
    return _PIM_;
  }

  /** implement functions needed to derive from Factor */

  /// vector of errors
  Vector evaluateError(const Pose3& pose_i, const Vector3& vel_i,
      const Pose3& pose_j, const Vector3& vel_j,
      const imuBias::ConstantBias& bias, boost::optional<Matrix&> H1 =
          boost::none, boost::optional<Matrix&> H2 = boost::none,
      boost::optional<Matrix&> H3 = boost::none, boost::optional<Matrix&> H4 =
          boost::none, boost::optional<Matrix&> H5 = boost::none) const;

  /// @deprecated typename
  typedef PreintegratedImuMeasurements PreintegratedMeasurements;

  /// @deprecated constructor, in the new one gravity, coriolis settings are in Params
  ImuFactor(Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key bias,
      const PreintegratedMeasurements& preintegratedMeasurements,
      const Vector3& n_gravity, const Vector3& omegaCoriolis,
      const boost::optional<Pose3>& body_P_sensor = boost::none,
      const bool use2ndOrderCoriolis = false);

  /// @deprecated use PreintegrationBase::predict,
  /// in the new one gravity, coriolis settings are in Params
  static void Predict(const Pose3& pose_i, const Vector3& vel_i, Pose3& pose_j,
      Vector3& vel_j, const imuBias::ConstantBias& bias_i,
      PreintegratedMeasurements& pim, const Vector3& n_gravity,
      const Vector3& omegaCoriolis, const bool use2ndOrderCoriolis = false);

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

} /// namespace gtsam
