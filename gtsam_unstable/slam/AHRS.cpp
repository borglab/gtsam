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

Matrix cov(const Matrix& m) {
  const double num_observations = m.cols();
  const Vector mean = m.rowwise().sum() / num_observations;
  Matrix D = m.colwise() - mean;
  Matrix DDt = D * trans(D);
  return DDt / (num_observations - 1);
}

Matrix I3 = eye(3);
Matrix Z3 = zeros(3, 3);

/* ************************************************************************* */
AHRS::AHRS(const Matrix& stationaryU, const Matrix& stationaryF, double g_e,
    bool flat) :
    KF_(9) {


  // Initial state
  mech0_ = Mechanization_bRn2::initialize(stationaryU, stationaryF, g_e, flat);

  size_t T = stationaryU.cols();

  // estimate standard deviation on gyroscope readings
  Pg_ = cov(stationaryU);
  Vector sigmas_v_g = esqrt(Pg_.diagonal() * T);

  // estimate standard deviation on accelerometer readings
  Pa_ = cov(stationaryF);

  // Quantities needed for integration

  // dynamics, Chris' email September 23, 2011 3:38:05 PM EDT
//  double tau_g = 730; // correlation time for gyroscope
//  double tau_a = 730; // correlation time for accelerometer
//  Matrix F_g = (Matrix(3,3)<<
//                        1./560.,     0,       0,
//                        0   ,   1./501.,    0,
//                        0,      0,      1./586.);        //730; // correlation time for gyroscope
//  Matrix F_a = (Matrix(3,3)<<
//                          1./490.,     0,      0,
//                          0,        1./500., 0,
//                          0,        0,      1./772.);         //730; // correlation time for accelerometer
  Matrix F_g = diag(Vector3(1./560,1./501,1./586));
  Matrix F_a = diag(Vector3(1./490,1./500,1./772));
  F_g_ = -I3 * F_g;
  F_a_ = -I3 * F_a;
  cout.precision(5);
  cout<<"F_g_ \n"<<F_g_<<endl;
  cout<<"F_a_\n"<<F_a_<<endl;

  Vector var_omega_w = 0 * ones(3); // TODO
  Vector var_omega_g = Vector3(8.7275e-4, 0.004, 0.004);        //(0.0034 * 0.0034) * ones(3);
  Vector var_omega_a = Vector3(0.0261, 0.0064, 0.034);          //(0.034 * 0.034) * ones(3);
  Vector sigmas_v_g_sq = emul(sigmas_v_g, sigmas_v_g);
  Vector vars = concatVectors(4, &var_omega_w, &var_omega_g, &sigmas_v_g_sq,
      &var_omega_a);
  var_w_ = diag(vars);

  // Quantities needed for aiding
  sigmas_v_a_ = esqrt(T * Pa_.diagonal());

  // gravity in nav frame
  n_g_ = (Vector(3) << 0.0, 0.0, g_e);
  n_g_cross_ = skewSymmetric(n_g_);  // nav frame has Z down !!!
}

/* ************************************************************************* */
std::pair<Mechanization_bRn2, KalmanFilter::State> AHRS::initialize(double g_e) {

  // Calculate Omega_T, formula 2.80 in Farrell08book
  double cp = cos(mech0_.bRn().inverse().pitch());
  double sp = sin(mech0_.bRn().inverse().pitch());
  double cy = cos(0.0);
  double sy = sin(0.0);
  Matrix Omega_T = (Matrix(3, 3) << cy * cp, -sy, 0.0, sy * cp, cy, 0.0, -sp, 0.0, 1.0);

  // Calculate Jacobian of roll/pitch/yaw wrpt (g1,g2,g3), see doc/ypr.nb
  Vector b_g = mech0_.b_g(g_e);
  std::cout<<"b_g : \n"<<b_g.transpose()<<std::endl;
  double g1 = b_g(0);
  double g2 = b_g(1);
  double g3 = b_g(2);
  double g23 = g2 * g2 + g3 * g3;
  double g123 = g1 * g1 + g23;
  double f = 1 / (std::sqrt(g23) * g123);
  Matrix H_g = (Matrix(3, 3) <<
      0.0, g3 / g23, -(g2 / g23),                       // roll
      std::sqrt(g23) / g123, -f * (g1 * g2), -f * (g1 * g3), // pitch
      0.0, 0.0, 0.0);                                   // we don't know anything on yaw

  // Calculate the initial covariance matrix for the error state dx, Farrell08book eq. 10.66
  Matrix Pa = 0.025 * 0.025 * eye(3);
  Matrix P11 = Omega_T * (H_g * (Pa + Pa_) * trans(H_g)) * trans(Omega_T);
  P11(2, 2) = 0.0001;
  Matrix P12 = -Omega_T * H_g * Pa;

  Matrix P_plus_k2 = Matrix(9, 9);
  P_plus_k2.block<3,3>(0, 0) = P11;
  P_plus_k2.block<3,3>(0, 3) = Z3;
  P_plus_k2.block<3,3>(0, 6) = P12;

  P_plus_k2.block<3,3>(3, 0) = Z3;
  P_plus_k2.block<3,3>(3, 3) = Pg_;
  P_plus_k2.block<3,3>(3, 6) = Z3;

  P_plus_k2.block<3,3>(6, 0) = trans(P12);
  P_plus_k2.block<3,3>(6, 3) = Z3;
  P_plus_k2.block<3,3>(6, 6) = Pa;
  std::cout<<"P_PLUS_K2 : \n"<<P_plus_k2<<std::endl;

  Vector dx = zero(9);
  KalmanFilter::State state = KF_.init(dx, P_plus_k2);
  state->print("KF initialize state: ");
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
  Matrix I3 = eye(3);
  Matrix Z3 = zeros(3, 3);

  Matrix F_k = zeros(9, 9);
  F_k.block<3,3>(0, 3) = -bRn;
  F_k.block<3,3>(3, 3) = F_g_;
  F_k.block<3,3>(6, 6) = F_a_;

  Matrix G_k = zeros(9, 12);
  G_k.block<3,3>(0, 0) = -bRn;
  G_k.block<3,3>(0, 6) = -bRn;
  G_k.block<3,3>(3, 3) = I3;
  G_k.block<3,3>(6, 9) = I3;

  Matrix Q_k = G_k * var_w_ * trans(G_k);
  Matrix Psi_k = eye(9) + dt * F_k; // +dt*dt*F_k*F_k/2; // transition matrix

  // This implements update in section 10.6
  Matrix B = zeros(9, 9);
  Vector u2 = zero(9);
  std::cout<<"dt : \n"<<dt<<std::endl;
  std::cout<<"F_k \n:"<<F_k<<std::endl;
  std::cout<<"G_k : \n"<<G_k<<std::endl;
  std::cout<<"Psi_k : \n"<<Psi_k<<std::endl;
  std::cout<<"Q_k : \n"<<Q_k<<std::endl;

  cout<<" before predict Q \n"<<endl;
  KalmanFilter::State newState = KF_.predictQ(state, Psi_k,B,u2,dt*Q_k);
  cout<<"after Predict Q \n"<<endl;
  cout<<"newState mean: \n"<<newState->mean();
  cout<<"newState covariance: \n"<<newState->covariance();
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
  return (fabs(mu_f)<0.5 && mu_u < 5.0 / 180.0 * M_PI);
}

/* ************************************************************************* */
std::pair<Mechanization_bRn2, KalmanFilter::State> AHRS::aid(
    const Mechanization_bRn2& mech, KalmanFilter::State state,
    const Vector& f, bool Farrell) const {
cout<<"in aid"<<endl;
  Matrix bRn = mech.bRn().matrix();

  // get gravity in body from accelerometer
  Vector measured_b_g = mech.x_a() - f;

  Matrix R, H;
  Vector z;
  if (Farrell) {
    // calculate residual gravity measurement
    z = n_g_ - trans(bRn) * measured_b_g;
    H = collect(3, &n_g_cross_, &Z3, &bRn);
    R = trans(bRn) * diag(emul(sigmas_v_a_, sigmas_v_a_)) * bRn;
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
    H = collect(3, &b_g, &Z3, &I3);
    // And the measurement noise, TODO: should be created once where sigmas_v_a is given
    R = diag(emul(sigmas_v_a_, sigmas_v_a_));

  }
  cout<<"R: \n"<<R<<endl;
  cout<<"H: \n"<<H<<endl;
  cout<<"z: \n"<<z<<endl;
  cout<<"state mean: \n"<<state->mean();
  cout<<"state covariance\n" <<state->covariance();
  cout<<"before updated state\n"<<endl;
// update the Kalman filter
  KalmanFilter::State updatedState = KF_.updateQ(state, H, z, R);
  cout<<"after updateQ \n"<<endl;
  cout<<"after updated state :\n"<<updatedState->mean()<<endl;
  cout<<"before correct\n"<<endl;
// update full state (rotation and accelerometer bias)
  Mechanization_bRn2 newMech = mech.correct(updatedState->mean());

// reset KF state
  Vector dx = zeros(9, 1);
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
  Matrix H = collect(3, &b_g, &I3, &Z3);
//  Matrix R = diag(emul(sigmas_v_a_, sigmas_v_a_));
//  Matrix R = diag((Vector(3) << 1.0, 0.2, 1.0)); // good for L_twice
  Matrix R = diag((Vector(3) << 0.01, 0.0001, 0.01));

// update the Kalman filter
  KalmanFilter::State updatedState = KF_.updateQ(state, H, z, R);

// update full state (rotation and gyro bias)
  Mechanization_bRn2 newMech = mech.correct(updatedState->mean());

// reset KF state
  Vector dx = zeros(9, 1);
  updatedState = KF_.init(dx, updatedState->covariance());

  return make_pair(newMech, updatedState);
}

/* ************************************************************************* */
void AHRS::print(const std::string& s) const {
  mech0_.print(s + ".mech0_");
  gtsam::print(F_g_, s + ".F_g");
  gtsam::print(F_a_, s + ".F_a");
  gtsam::print(var_w_, s + ".var_w");

  gtsam::print(sigmas_v_a_, s + ".sigmas_v_a");
  gtsam::print(n_g_, s + ".n_g");
  gtsam::print(n_g_cross_, s + ".n_g_cross");

  gtsam::print(Pg_, s + ".P_g");
  gtsam::print(Pa_, s + ".P_a");

}

/* ************************************************************************* */
AHRS::~AHRS() {
}

/* ************************************************************************* */

} /* namespace gtsam */
