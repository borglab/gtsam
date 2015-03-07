/**
 * @file Mechanization_bRn.cpp
 * @date Jan 25, 2012
 * @author Chris Beall
 * @author Frank Dellaert
 */

#include "Mechanization_bRn2.h"
#include <boost/foreach.hpp>

namespace gtsam {

/* ************************************************************************* */
Mechanization_bRn2 Mechanization_bRn2::initializeVector(const std::list<Vector>& U,
    const std::list<Vector>& F, const double g_e, bool flat) {
  Matrix Umat = (Matrix(3, U.size()) << concatVectors(U));
  Matrix Fmat = (Matrix(3, F.size()) << concatVectors(F));

  return initialize(Umat, Fmat, g_e, flat);
}


/* ************************************************************************* */
Mechanization_bRn2 Mechanization_bRn2::initialize(const Matrix& U,
    const Matrix& F, const double g_e, bool flat) {

  // estimate gyro bias by averaging gyroscope readings (10.62)
  Vector x_g = U.rowwise().mean();
  Vector meanF = F.rowwise().mean();

  // estimate gravity
  Vector b_g;
  if(g_e == 0) {
    if (flat)
      // acceleration measured is  along the z-axis.
      b_g = (Vector(3) << 0.0, 0.0, norm_2(meanF));
    else
      // acceleration measured is the opposite of gravity (10.13)
      b_g = -meanF;
  } else {
    if (flat)
      // gravity is downward along the z-axis since we are flat on the ground
      b_g = (Vector(3) << 0.0,0.0,g_e);
    else
      // normalize b_g and attribute remainder to biases
      b_g = - g_e * meanF/meanF.norm();
  }

  // estimate accelerometer bias
  Vector x_a = meanF + b_g;

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
  Rot3 bRn = Rot3::ypr(yaw, pitch, roll).inverse();

  return Mechanization_bRn2(bRn, x_g, x_a);
}

/* ************************************************************************* */
Mechanization_bRn2 Mechanization_bRn2::correct(const Vector& dx) const {
  Vector rho = sub(dx, 0, 3);

  Rot3 delta_nRn = Rot3::rodriguez(rho);
  Rot3 bRn = bRn_ * delta_nRn;

  Vector x_g = x_g_ + sub(dx, 3, 6);
  Vector x_a = x_a_ + sub(dx, 6, 9);

  return Mechanization_bRn2(bRn, x_g, x_a);
}

/* ************************************************************************* */
Mechanization_bRn2 Mechanization_bRn2::integrate(const Vector& u,
    const double dt) const {
  // integrate rotation nRb based on gyro measurement u and bias x_g

#ifndef MODEL_NAV_FRAME_ROTATION
  // get body to inertial (measured in b) from gyro, subtract bias
  Vector b_omega_ib = u - x_g_;

  // nav to inertial due to Earth's rotation and our movement on Earth surface
  // TODO: figure out how to do this if we need it
  Vector b_omega_in = zero(3);

  // calculate angular velocity on bRn measured in body frame
  Vector b_omega_bn = b_omega_in - b_omega_ib;
#else
  // Assume a non-rotating nav frame (probably an approximation)
  // calculate angular velocity on bRn measured in body frame
  Vector b_omega_bn = x_g_ - u;
#endif

  // convert to navigation frame
  Rot3 nRb = bRn_.inverse();
  Vector n_omega_bn = (nRb*b_omega_bn).vector();

  // integrate bRn using exponential map, assuming constant over dt
  Rot3 delta_nRn = Rot3::rodriguez(n_omega_bn*dt);
  Rot3 bRn = bRn_.compose(delta_nRn);

  // implicit updating of biases (variables just do not change)
  // x_g = x_g;
  // x_a = x_a;
  return Mechanization_bRn2(bRn, x_g_, x_a_);
}

} /* namespace gtsam */
