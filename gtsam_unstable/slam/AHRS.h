/*
 * AHRS.h
 *
 *  Created on: Jan 26, 2012
 *      Author: cbeall3
 */

#pragma once

#include "Mechanization_bRn2.h"
#include <gtsam_unstable/dllexport.h>
#include <gtsam/linear/KalmanFilter.h>

namespace gtsam {

class GTSAM_UNSTABLE_EXPORT AHRS {

private:

  // Initial state
  Mechanization_bRn2 mech0_; ///< Initial mechanization state
  KalmanFilter KF_;          ///< initial Kalman filter

  // Quantities needed for integration
  Matrix3 F_g_;              ///< gyro bias dynamics
  Matrix3 F_a_;              ///< acc bias dynamics

  typedef Eigen::Matrix<double,12,1> Variances;
  Variances var_w_;            ///< dynamic noise variances

  // Quantities needed for aiding
  Vector3 sigmas_v_a_;       ///< measurement sigma
  Vector3 n_g_;              ///< gravity in nav frame
  Matrix3 n_g_cross_;        ///< and its skew-symmetric matrix

  Matrix3 Pg_, Pa_;

public:

  typedef Eigen::Matrix<double,3,Eigen::Dynamic> Vector3s;
  static Matrix3 Cov(const Vector3s& m);

  /**
   * AHRS constructor
   * @param stationaryU initial interval of gyro measurements, 3xn matrix
   * @param stationaryF initial interval of accelerator measurements, 3xn matrix
   * @param g_e if given, initializes gravity with correct value g_e
   */
  AHRS(const Matrix& stationaryU, const Matrix& stationaryF, double g_e, bool flat=false);

  std::pair<Mechanization_bRn2, KalmanFilter::State> initialize(double g_e);

  std::pair<Mechanization_bRn2, KalmanFilter::State> integrate(
      const Mechanization_bRn2& mech, KalmanFilter::State state,
      const Vector& u, double dt);

  bool isAidingAvailable(const Mechanization_bRn2& mech,
      const Vector& f, const Vector& u, double ge) const;

  /**
   * Aid the AHRS with an accelerometer reading, typically called when isAidingAvailable == true
   * @param mech current mechanization state
   * @param state current Kalman filter state
   * @param f accelerometer reading
   * @param Farrell
   */
  std::pair<Mechanization_bRn2, KalmanFilter::State> aid(
      const Mechanization_bRn2& mech, KalmanFilter::State state,
      const Vector& f, bool Farrell=0) const;

  std::pair<Mechanization_bRn2, KalmanFilter::State> aidGeneral(
      const Mechanization_bRn2& mech, KalmanFilter::State state,
      const Vector& f, const Vector& f_expected, const Rot3& R_previous) const;

  /// print
  void print(const std::string& s = "") const;

  virtual ~AHRS();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} /* namespace gtsam */
