/**
 * @file Mechanization_bRn2.cpp
 * @date Jan 25, 2012
 * @author Chris Beall
 * @author Frank Dellaert
 */

#include "Mechanization_bRn2.h"

namespace gtsam {

/* ************************************************************************* */
Mechanization_bRn2 Mechanization_bRn2::initializeVector(const std::list<Vector>& U,
    const std::list<Vector>& F, const double g_e, bool flat) {
  Matrix Umat = (Matrix(3, U.size()) << concatVectors(U)).finished();
  Matrix Fmat = (Matrix(3, F.size()) << concatVectors(F)).finished();

  return initialize(Umat, Fmat, g_e, flat);
}


/* ************************************************************************* */
Mechanization_bRn2 Mechanization_bRn2::initialize(const Matrix& U,
    const Matrix& F, const double g_e, bool flat) {

  // estimate gyro bias by averaging gyroscope readings (10.62)
  Vector3 x_g = U.rowwise().mean();
  Vector3 meanF = F.rowwise().mean();

  // estimate gravity
  Vector3 b_g;
  if(g_e == 0) {
    if (flat)
      // acceleration measured is  along the z-axis.
      b_g = (Vector3(3) << 0.0, 0.0, meanF.norm()).finished();
    else
      // acceleration measured is the opposite of gravity (10.13)
      b_g = -meanF;
  } else {
    if (flat)
      // gravity is downward along the z-axis since we are flat on the ground
      b_g = (Vector3(3) << 0.0,0.0,g_e).finished();
    else
      // normalize b_g and attribute remainder to biases
      b_g = - g_e * meanF/meanF.norm();
  }

  // estimate accelerometer bias
  Vector3 x_a = meanF + b_g;

  // initialize roll, pitch (eqns. 10.14, 10.15)
  double g1=b_g(0);
  double g2=b_g(1);
  double g3=b_g(2);

  // Farrell08book eqn. 10.14
  double roll = atan2(g2, g3);
  // Farrell08book eqn. 10.15
  double pitch = atan2(-g1, sqrt(g2 * g2 + g3 * g3));
  double yaw = 0;
  // This returns body-to-nav nRb
  Rot3 bRn = Rot3::Ypr(yaw, pitch, roll).inverse();

  return Mechanization_bRn2(bRn, x_g, x_a);
}

/* ************************************************************************* */
Mechanization_bRn2 Mechanization_bRn2::correct(const Vector9& dx) const {
  Vector3 rho = dx.segment<3>(0);

  Rot3 delta_nRn = Rot3::Rodrigues(rho);
  Rot3 bRn = bRn_ * delta_nRn;

  Vector3 x_g = x_g_ + dx.segment<3>(3);
  Vector3 x_a = x_a_ + dx.segment<3>(6);

  return Mechanization_bRn2(bRn, x_g, x_a);
}

/* ************************************************************************* */
Mechanization_bRn2 Mechanization_bRn2::integrate(const Vector3& u,
    const double dt) const {
  // integrate rotation nRb based on gyro measurement u and bias x_g

#ifndef MODEL_NAV_FRAME_ROTATION
  // get body to inertial (measured in b) from gyro, subtract bias
  Vector3 b_omega_ib = u - x_g_;

  // nav to inertial due to Earth's rotation and our movement on Earth surface
  // TODO: figure out how to do this if we need it
  Vector3 b_omega_in; b_omega_in.setZero();

  // calculate angular velocity on bRn measured in body frame
  Vector3 b_omega_bn = b_omega_in - b_omega_ib;
#else
  // Assume a non-rotating nav frame (probably an approximation)
  // calculate angular velocity on bRn measured in body frame
  Vector3 b_omega_bn = x_g_ - u;
#endif

  // convert to navigation frame
  Rot3 nRb = bRn_.inverse();
  Vector3 n_omega_bn = nRb * b_omega_bn;

  // integrate bRn using exponential map, assuming constant over dt
  Rot3 delta_nRn = Rot3::Rodrigues(n_omega_bn*dt);
  Rot3 bRn = bRn_.compose(delta_nRn);

  // implicit updating of biases (variables just do not change)
  // x_g = x_g;
  // x_a = x_a;
  return Mechanization_bRn2(bRn, x_g_, x_a_);
}

} /* namespace gtsam */
