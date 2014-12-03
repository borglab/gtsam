/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  PreintegrationBase.h
 *  @author Luca Carlone
 *  @author Stephen Williams
 *  @author Richard Roberts
 *  @author Vadim Indelman
 *  @author David Jensen
 *  @author Frank Dellaert
 **/

#pragma once

/* GTSAM includes */
#include <gtsam/navigation/ImuBias.h>

namespace gtsam {


/**
 * PreintegrationBase is the base class for PreintegratedMeasurements (in ImuFactor.h) and
 * CombinedPreintegratedMeasurements (in CombinedImuFactor.h). It includes the definitions of the
 * preintegrated variables and the methods to access, print, and compare them.
 */
class PreintegrationBase {

  friend class ImuFactor;
  friend class CombinedImuFactor;

protected:
  imuBias::ConstantBias biasHat_; ///< Acceleration and angular rate bias values used during preintegration
  bool use2ndOrderIntegration_; ///< Controls the order of integration

  Vector3 deltaPij_; ///< Preintegrated relative position (does not take into account velocity at time i, see deltap+, in [2]) (in frame i)
  Vector3 deltaVij_; ///< Preintegrated relative velocity (in global frame)
  Rot3 deltaRij_;    ///< Preintegrated relative orientation (in frame i)
  double deltaTij_;  ///< Time interval from i to j

  Matrix3 delPdelBiasAcc_;   ///< Jacobian of preintegrated position w.r.t. acceleration bias
  Matrix3 delPdelBiasOmega_; ///< Jacobian of preintegrated position w.r.t. angular rate bias
  Matrix3 delVdelBiasAcc_;   ///< Jacobian of preintegrated velocity w.r.t. acceleration bias
  Matrix3 delVdelBiasOmega_; ///< Jacobian of preintegrated velocity w.r.t. angular rate bias
  Matrix3 delRdelBiasOmega_; ///< Jacobian of preintegrated rotation w.r.t. angular rate bias

public:

  /**
   *  Default constructor, initializes the variables in the base class
   *  @param bias Current estimate of acceleration and rotation rate biases
   *  @param use2ndOrderIntegration     Controls the order of integration
   *  (if false: p(t+1) = p(t) + v(t) deltaT ; if true: p(t+1) = p(t) + v(t) deltaT + 0.5 * acc(t) deltaT^2)
   */
  PreintegrationBase(const imuBias::ConstantBias& bias, const bool use2ndOrderIntegration) :
    biasHat_(bias), use2ndOrderIntegration_(use2ndOrderIntegration),
    deltaPij_(Vector3::Zero()), deltaVij_(Vector3::Zero()),
    deltaRij_(Rot3()), deltaTij_(0.0),
    delPdelBiasAcc_(Z_3x3), delPdelBiasOmega_(Z_3x3),
    delVdelBiasAcc_(Z_3x3), delVdelBiasOmega_(Z_3x3),
    delRdelBiasOmega_(Z_3x3) { }

  /// Needed for testable
  void print(const std::string& s) const {
    std::cout << s << std::endl;
    biasHat_.print("  biasHat");
    std::cout << "  deltaTij " << deltaTij_ << std::endl;
    std::cout << "  deltaPij [ " << deltaPij_.transpose() << " ]" << std::endl;
    std::cout << "  deltaVij [ " << deltaVij_.transpose() << " ]" << std::endl;
    deltaRij_.print("  deltaRij ");
  }

  /// Needed for testable
  bool equals(const PreintegrationBase& expected, double tol) const {
    return biasHat_.equals(expected.biasHat_, tol)
        && equal_with_abs_tol(deltaPij_, expected.deltaPij_, tol)
        && equal_with_abs_tol(deltaVij_, expected.deltaVij_, tol)
        && deltaRij_.equals(expected.deltaRij_, tol)
        && fabs(deltaTij_ - expected.deltaTij_) < tol
        && equal_with_abs_tol(delPdelBiasAcc_, expected.delPdelBiasAcc_, tol)
    && equal_with_abs_tol(delPdelBiasOmega_, expected.delPdelBiasOmega_, tol)
    && equal_with_abs_tol(delVdelBiasAcc_, expected.delVdelBiasAcc_, tol)
    && equal_with_abs_tol(delVdelBiasOmega_, expected.delVdelBiasOmega_, tol)
    && equal_with_abs_tol(delRdelBiasOmega_, expected.delRdelBiasOmega_, tol);
  }

  /// Re-initialize PreintegratedMeasurements
  void resetIntegration(){
    deltaPij_ = Vector3::Zero();
    deltaVij_ = Vector3::Zero();
    deltaRij_ = Rot3();
    deltaTij_ = 0.0;
    delPdelBiasAcc_ = Z_3x3;
    delPdelBiasOmega_ = Z_3x3;
    delVdelBiasAcc_ = Z_3x3;
    delVdelBiasOmega_ = Z_3x3;
    delRdelBiasOmega_ = Z_3x3;
  }

  /// methods to access class variables
  Matrix deltaRij() const {return deltaRij_.matrix();}
  double deltaTij() const{return deltaTij_;}
  Vector deltaPij() const {return deltaPij_;}
  Vector deltaVij() const {return deltaVij_;}
  Vector biasHat() const { return biasHat_.vector();}
  Matrix delPdelBiasAcc() const { return delPdelBiasAcc_;}
  Matrix delPdelBiasOmega() const { return delPdelBiasOmega_;}
  Matrix delVdelBiasAcc() const { return delVdelBiasAcc_;}
  Matrix delVdelBiasOmega() const { return delVdelBiasOmega_;}
  Matrix delRdelBiasOmega() const{ return delRdelBiasOmega_;}

  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  // This function is only used for test purposes (compare numerical derivatives wrt analytic ones)
  static inline Vector PreIntegrateIMUObservations_delta_vel(const Vector& msr_gyro_t, const Vector& msr_acc_t, const double msr_dt,
      const Vector3& delta_angles, const Vector& delta_vel_in_t0){
    // Note: all delta terms refer to an IMU\sensor system at t0
    Vector body_t_a_body = msr_acc_t;
    Rot3 R_t_to_t0 = Rot3::Expmap(delta_angles);
    return delta_vel_in_t0 + R_t_to_t0.matrix() * body_t_a_body * msr_dt;
  }

  // This function is only used for test purposes (compare numerical derivatives wrt analytic ones)
  static inline Vector PreIntegrateIMUObservations_delta_angles(const Vector& msr_gyro_t, const double msr_dt,
      const Vector3& delta_angles){
    // Note: all delta terms refer to an IMU\sensor system at t0
    // Calculate the corrected measurements using the Bias object
    Vector body_t_omega_body= msr_gyro_t;
    Rot3 R_t_to_t0 = Rot3::Expmap(delta_angles);
    R_t_to_t0    = R_t_to_t0 * Rot3::Expmap( body_t_omega_body*msr_dt );
    return Rot3::Logmap(R_t_to_t0);
  }

  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_NVP(biasHat_);
    ar & BOOST_SERIALIZATION_NVP(deltaPij_);
    ar & BOOST_SERIALIZATION_NVP(deltaVij_);
    ar & BOOST_SERIALIZATION_NVP(deltaRij_);
    ar & BOOST_SERIALIZATION_NVP(deltaTij_);
    ar & BOOST_SERIALIZATION_NVP(delPdelBiasAcc_);
    ar & BOOST_SERIALIZATION_NVP(delPdelBiasOmega_);
    ar & BOOST_SERIALIZATION_NVP(delVdelBiasAcc_);
    ar & BOOST_SERIALIZATION_NVP(delVdelBiasOmega_);
    ar & BOOST_SERIALIZATION_NVP(delRdelBiasOmega_);
  }
};

} /// namespace gtsam
