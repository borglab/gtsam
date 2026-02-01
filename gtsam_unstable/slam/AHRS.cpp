/*
 * @file AHRS.cpp
 * @brief Attitude and Heading Reference System implementation
 *  Created on: Jan 26, 2012
 *      Author: cbeall3
 */

#include "AHRS.h"
#include <cmath>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
Matrix3 AHRS::Cov(const Vector3s& m) {
  const double num_observations = m.cols();
  const Vector3 mean = m.rowwise().sum() / num_observations;
  Vector3s D = m.colwise() - mean;
  return D * trans(D) / (num_observations - 1);
}

/* ************************************************************************* */
AHRS::AHRS(const Matrix& stationaryU, const Matrix& stationaryF, double g_e,
    bool flat) :
    KF_(9) {

  // Initial state
  mech0_ = Mechanization_bRn2::initialize(stationaryU, stationaryF, g_e, flat);

  size_t T = stationaryU.cols();

  // estimate standard deviation on gyroscope readings
  Pg_ = Cov(stationaryU);
  Vector3 sigmas_v_g = (Pg_.diagonal() * T).cwiseSqrt();

  // estimate standard deviation on accelerometer readings
  Pa_ = Cov(stationaryF);

  // Quantities needed for integration

  // dynamics, Chris' email September 23, 2011 3:38:05 PM EDT
  double tau_g = 730; // correlation time for gyroscope
  double tau_a = 730; // correlation time for accelerometer

  F_g_ = -I_3x3 / tau_g;
  F_a_ = -I_3x3 / tau_a;
  Vector3 var_omega_w = 0 * Vector::Ones(3); // TODO
  Vector3 var_omega_g = (0.0034 * 0.0034) * Vector::Ones(3);
  Vector3 var_omega_a = (0.034 * 0.034) * Vector::Ones(3);
  Vector3 sigmas_v_g_sq = sigmas_v_g.array().square();
  var_w_ << var_omega_w, var_omega_g, sigmas_v_g_sq, var_omega_a;

  // Quantities needed for aiding
  sigmas_v_a_ = (T * Pa_.diagonal()).cwiseSqrt();

  // gravity in nav frame
  n_g_ = (Vector(3) << 0.0, 0.0, g_e).finished();
  n_g_cross_ = skewSymmetric(n_g_);  // nav frame has Z down !!!
}

/* ************************************************************************* */
std::pair<Mechanization_bRn2, KalmanFilter::State> AHRS::initialize(double g_e) {

  // Calculate Omega_T, formula 2.80 in Farrell08book
  double cp = cos(mech0_.bRn().inverse().pitch());
  double sp = sin(mech0_.bRn().inverse().pitch());
  double cy = cos(0.0);
  double sy = sin(0.0);
  Matrix Omega_T = (Matrix(3, 3) << cy * cp, -sy, 0.0, sy * cp, cy, 0.0, -sp, 0.0, 1.0).finished();

  // Calculate Jacobian of roll/pitch/yaw wrpt (g1,g2,g3), see doc/ypr.nb
  Vector b_g = mech0_.b_g(g_e);
  double g1 = b_g(0);
  double g2 = b_g(1);
  double g3 = b_g(2);
  double g23 = g2 * g2 + g3 * g3;
  double g123 = g1 * g1 + g23;
  double f = 1 / (std::sqrt(g23) * g123);
  Matrix H_g = (Matrix(3, 3) <<
      0.0, g3 / g23, -(g2 / g23),                            // roll
      std::sqrt(g23) / g123, -f * (g1 * g2), -f * (g1 * g3), // pitch
      0.0, 0.0, 0.0).finished();                             // we don't know anything on yaw

  // Calculate the initial covariance matrix for the error state dx, Farrell08book eq. 10.66
  Matrix Pa = 0.025 * 0.025 * I_3x3;
  Matrix P11 = Omega_T * (H_g * (Pa + Pa_) * trans(H_g)) * trans(Omega_T);
  P11(2, 2) = 0.0001;
  Matrix P12 = -Omega_T * H_g * Pa;

  Matrix P_plus_k2 = Matrix(9, 9);
  P_plus_k2.block<3,3>(0, 0) = P11;
  P_plus_k2.block<3,3>(0, 3) = Z_3x3;
  P_plus_k2.block<3,3>(0, 6) = P12;

  P_plus_k2.block<3,3>(3, 0) = Z_3x3;
  P_plus_k2.block<3,3>(3, 3) = Pg_;
  P_plus_k2.block<3,3>(3, 6) = Z_3x3;

  P_plus_k2.block<3,3>(6, 0) = trans(P12);
  P_plus_k2.block<3,3>(6, 3) = Z_3x3;
  P_plus_k2.block<3,3>(6, 6) = Pa;

  Vector dx = Z_9x1;
  KalmanFilter::State state = KF_.init(dx, P_plus_k2);
  return std::make_pair(mech0_, state);
}

/* ************************************************************************* */
std::pair<Mechanization_bRn2, KalmanFilter::State> AHRS::integrate(
    const Mechanization_bRn2& mech, KalmanFilter::State state,
    const Vector& u, double dt) {

  // Integrate full state
  Mechanization_bRn2 newMech = mech.integrate(u, dt);

  // Integrate error state Kalman filter
  // FIXME:
  //if nargout>1
  Matrix bRn = mech.bRn().matrix();

  Matrix9 F_k; F_k.setZero();
  F_k.block<3,3>(0, 3) = -bRn;
  F_k.block<3,3>(3, 3) = F_g_;
  F_k.block<3,3>(6, 6) = F_a_;

  typedef Eigen::Matrix<double,9,12> Matrix9_12;
  Matrix9_12 G_k; G_k.setZero();
  G_k.block<3,3>(0, 0) = -bRn;
  G_k.block<3,3>(0, 6) = -bRn;
  G_k.block<3,3>(3, 3) = I_3x3;
  G_k.block<3,3>(6, 9) = I_3x3;

  Matrix9 Q_k = G_k * var_w_.asDiagonal() * G_k.transpose();
  Matrix9 Psi_k = I_9x9 + dt * F_k; // +dt*dt*F_k*F_k/2; // transition matrix

  // This implements update in section 10.6
  Matrix9 B; B.setZero();
  Vector9 u2; u2.setZero();
  // TODO predictQ should be templated to also take fixed size matrices.
  KalmanFilter::State newState = KF_.predictQ(state, Psi_k,B,u2,dt*Q_k);
  return make_pair(newMech, newState);
}

/* ************************************************************************* */
bool AHRS::isAidingAvailable(const Mechanization_bRn2& mech,
    const gtsam::Vector& f, const gtsam::Vector& u, double ge) const {

  // Subtract the biases
  Vector f_ = f - mech.x_a();
  Vector u_ = u - mech.x_g();

  double mu_f = f_.norm() - ge; // accelerometer same magnitude as local gravity ?
  double mu_u = u_.norm(); // gyro says we are not maneuvering ?
  return (std::abs(mu_f)<0.5 && mu_u < 5.0 / 180.0 * M_PI);
}

/* ************************************************************************* */
std::pair<Mechanization_bRn2, KalmanFilter::State> AHRS::aid(
    const Mechanization_bRn2& mech, KalmanFilter::State state,
    const Vector& f, bool Farrell) const {

  Matrix bRn = mech.bRn().matrix();

  // get gravity in body from accelerometer
  Vector measured_b_g = mech.x_a() - f;

  Matrix R, H;
  Vector z;
  if (Farrell) {
    // calculate residual gravity measurement
    z = n_g_ - trans(bRn) * measured_b_g;
    H = collect(3, &n_g_cross_, &Z_3x3, &bRn);
    R = trans(bRn) * ((Vector3) sigmas_v_a_.array().square()).asDiagonal() * bRn;
  } else {
    // my measurement prediction (in body frame):
    // F(:,k) = bias - b_g
    // F(:,k) = mech.x_a + dx_a - bRn*n_g;
    // F(:,k) = mech.x_a + dx_a - bRn*(I+P)*n_g;
    // F(:,k) = mech.x_a + dx_a - b_g - bRn*(rho x n_g); // P = [rho]_x
  // Hence, the measurement z = b_g - (mech.x_a - F(:,k)) is predicted
  // from the filter state (dx_a, rho) as  dx_a + bRn*(n_g x rho)
    // z = b_g - (mech.x_a - F(:,k)) = dx_a + bRn*(n_g x rho)
    z = bRn * n_g_ - measured_b_g;
    // Now the Jacobian H
    Matrix b_g = bRn * n_g_cross_;
    H = collect(3, &b_g, &Z_3x3, &I_3x3);
    // And the measurement noise, TODO: should be created once where sigmas_v_a is given
    R = ((Vector3) sigmas_v_a_.array().square()).asDiagonal();
  }

// update the Kalman filter
  KalmanFilter::State updatedState = KF_.updateQ(state, H, z, R);

// update full state (rotation and accelerometer bias)
  Mechanization_bRn2 newMech = mech.correct(updatedState->mean());

// reset KF state
  Vector dx = Z_9x1;
  updatedState = KF_.init(dx, updatedState->covariance());

  return make_pair(newMech, updatedState);
}

/* ************************************************************************* */
std::pair<Mechanization_bRn2, KalmanFilter::State> AHRS::aidGeneral(
    const Mechanization_bRn2& mech, KalmanFilter::State state,
    const Vector& f, const Vector& f_previous,
    const Rot3& R_previous) const {

  Matrix increment = R_previous.between(mech.bRn()).matrix();

  // expected - measured
  Vector z = f - increment * f_previous;
  //Vector z = increment * f_previous - f;
  Matrix b_g = skewSymmetric(increment* f_previous);
  Matrix H = collect(3, &b_g, &I_3x3, &Z_3x3);
//  Matrix R = diag(emul(sigmas_v_a_, sigmas_v_a_));
//  Matrix R = diag(Vector3(1.0, 0.2, 1.0)); // good for L_twice
  Matrix R = Vector3(0.01, 0.0001, 0.01).asDiagonal();

// update the Kalman filter
  KalmanFilter::State updatedState = KF_.updateQ(state, H, z, R);

// update full state (rotation and gyro bias)
  Mechanization_bRn2 newMech = mech.correct(updatedState->mean());

// reset KF state
  Vector dx = Z_9x1;
  updatedState = KF_.init(dx, updatedState->covariance());

  return make_pair(newMech, updatedState);
}

/* ************************************************************************* */
void AHRS::print(const std::string& s) const {
  mech0_.print(s + ".mech0_");
  gtsam::print((Matrix)F_g_, s + ".F_g");
  gtsam::print((Matrix)F_a_, s + ".F_a");
  gtsam::print((Vector)var_w_, s + ".var_w");

  gtsam::print((Vector)sigmas_v_a_, s + ".sigmas_v_a");
  gtsam::print((Vector)n_g_, s + ".n_g");
  gtsam::print((Matrix)n_g_cross_, s + ".n_g_cross");

  gtsam::print((Matrix)Pg_, s + ".P_g");
  gtsam::print((Matrix)Pa_, s + ".P_a");

}

/* ************************************************************************* */
AHRS::~AHRS() {
}

/* ************************************************************************* */

} /* namespace gtsam */
