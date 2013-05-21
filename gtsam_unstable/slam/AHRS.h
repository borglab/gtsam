/*
 * AHRS.h
 *
 *  Created on: Jan 26, 2012
 *      Author: cbeall3
 */

#ifndef AHRS_H_
#define AHRS_H_

#include "Mechanization_bRn2.h"
#include <gtsam/linear/KalmanFilter.h>

namespace gtsam {

Matrix cov(const Matrix& m);

class AHRS {

private:

  // Initial state
  Mechanization_bRn2 mech0_; ///< Initial mechanization state
  KalmanFilter KF_;          ///< initial Kalman filter

  // Quantities needed for integration
  Matrix F_g_;              ///< gyro bias dynamics
  Matrix F_a_;              ///< acc bias dynamics
  Matrix var_w_;            ///< dynamic noise variances

  // Quantities needed for aiding
  Vector sigmas_v_a_;       ///< measurement sigma
  Vector n_g_;              ///< gravity in nav frame
  Matrix n_g_cross_;        ///< and its skew-symmetric matrix

  Matrix Pg_, Pa_;

public:
  /**
   * AHRS constructor
   * @param stationaryU initial interval of gyro measurements, 3xn matrix
   * @param stationaryF initial interval of accelerator measurements, 3xn matrix
   * @param g_e if given, initializes gravity with correct value g_e
   */
  AHRS(const Matrix& stationaryU, const Matrix& stationaryF, double g_e);

  std::pair<Mechanization_bRn2, KalmanFilter::State> initialize(double g_e);

  std::pair<Mechanization_bRn2, KalmanFilter::State> integrate(
      const Mechanization_bRn2& mech, KalmanFilter::State state,
      const Vector& u, double dt);

  std::pair<Mechanization_bRn2, KalmanFilter::State> aid(
      const Mechanization_bRn2& mech, KalmanFilter::State state,
      const Vector& f, bool Farrell=0);

  std::pair<Mechanization_bRn2, KalmanFilter::State> aidGeneral(
      const Mechanization_bRn2& mech, KalmanFilter::State state,
      const Vector& f, const Vector& f_expected, const Rot3& R_previous);

  /// print
  void print(const std::string& s = "") const;

  virtual ~AHRS();
};

} /* namespace gtsam */
#endif /* AHRS_H_ */
