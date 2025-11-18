/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SO4.cpp
 * @brief   4*4 matrix representation of SO(4)
 * @author  Frank Dellaert
 * @author  Luca Carlone
 */

#include <gtsam/base/concepts.h>
#include <gtsam/base/timing.h>
#include <gtsam/geometry/SO4.h>
#include <gtsam/geometry/Unit3.h>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

using namespace std;

namespace gtsam {

// TODO(frank): is this better than SOn::Random?
// /* *************************************************************************
// */ static Vector3 randomOmega(std::mt19937 &rng) {
//   static std::uniform_real_distribution<double> randomAngle(-M_PI, M_PI);
//   return Unit3::Random(rng).unitVector() * randomAngle(rng);
// }

// /* *************************************************************************
// */
// // Create random SO(4) element using direct product of lie algebras.
// SO4 SO4::Random(std::mt19937 &rng) {
//   Vector6 delta;
//   delta << randomOmega(rng), randomOmega(rng);
//   return SO4::Expmap(delta);
// }

//******************************************************************************
template <>
GTSAM_EXPORT
Matrix4 SO4::Hat(const Vector6& xi) {
  // skew symmetric matrix X = xi^
  // Unlike Luca, makes upper-left the SO(3) subgroup.
  Matrix4 Y = Z_4x4;
  Y(0, 1) = -xi(5);
  Y(0, 2) = +xi(4);
  Y(1, 2) = -xi(3);
  Y(0, 3) = +xi(2);
  Y(1, 3) = -xi(1);
  Y(2, 3) = +xi(0);
  return Y - Y.transpose();
}

//******************************************************************************
template <>
GTSAM_EXPORT
Vector6 SO4::Vee(const Matrix4& X) {
  Vector6 xi;
  xi(5) = -X(0, 1);
  xi(4) = +X(0, 2);
  xi(3) = -X(1, 2);
  xi(2) = +X(0, 3);
  xi(1) = -X(1, 3);
  xi(0) = +X(2, 3);
  return xi;
}

//******************************************************************************
/* Exponential map, porting MATLAB implementation by Luca, which follows
 * "SOME REMARKS ON THE EXPONENTIAL MAP ON THE GROUPS SO(n) AND SE(n)" by
 * Ramona-Andreaa Rohan */
template <>
GTSAM_EXPORT
SO4 SO4::Expmap(const Vector6& xi, ChartJacobian H) {
  using namespace std;
  if (H) throw std::runtime_error("SO4::Expmap Jacobian");

  // skew symmetric matrix X = xi^
  const Matrix4 X = Hat(xi);

  // do eigen-decomposition
  auto eig = Eigen::EigenSolver<Matrix4>(X);
  Eigen::Vector4cd e = eig.eigenvalues();
  using std::abs;
  sort(e.data(), e.data() + 4, [](complex<double> a, complex<double> b) {
    return abs(a.imag()) > abs(b.imag());
  });

  // Get a and b from eigenvalues +/i ai and +/- bi
  double a = e[0].imag(), b = e[2].imag();
  if (!e.real().isZero() || e[1].imag() != -a || e[3].imag() != -b) {
    throw runtime_error("SO4::Expmap: wrong eigenvalues.");
  }

  // Build expX = exp(xi^)
  using std::cos;
  using std::sin;
  const auto X2 = X * X;
  const auto X3 = X2 * X;
  double a2 = a * a, a3 = a2 * a, b2 = b * b, b3 = b2 * b;
  if (a != 0 && b == 0) {
    double c2 = (1 - cos(a)) / a2, c3 = (a - sin(a)) / a3;
    return SO4(I_4x4 + X + c2 * X2 + c3 * X3);
  } else if (a == b && b != 0) {
    double sin_a = sin(a), cos_a = cos(a);
    double c0 = (a * sin_a + 2 * cos_a) / 2,
           c1 = (3 * sin_a - a * cos_a) / (2 * a), c2 = sin_a / (2 * a),
           c3 = (sin_a - a * cos_a) / (2 * a3);
    return SO4(c0 * I_4x4 + c1 * X + c2 * X2 + c3 * X3);
  } else if (a != b) {
    double sin_a = sin(a), cos_a = cos(a);
    double sin_b = sin(b), cos_b = cos(b);
    double c0 = (b2 * cos_a - a2 * cos_b) / (b2 - a2),
           c1 = (b3 * sin_a - a3 * sin_b) / (a * b * (b2 - a2)),
           c2 = (cos_a - cos_b) / (b2 - a2),
           c3 = (b * sin_a - a * sin_b) / (a * b * (b2 - a2));
    return SO4(c0 * I_4x4 + c1 * X + c2 * X2 + c3 * X3);
  } else {
    return SO4();
  }
}

//******************************************************************************
template <>
GTSAM_EXPORT
SO4::VectorN2 SO4::vec(OptionalJacobian<16, 6> H) const {
  const Matrix& Q = matrix_;
  if (H) {
    H->setZero();
    H->block<4, 1>(0, 2) = -Q.col(3);
    H->block<4, 1>(0, 4) = -Q.col(2);
    H->block<4, 1>(0, 5) = Q.col(1);
    H->block<4, 1>(4, 1) = Q.col(3);
    H->block<4, 1>(4, 3) = Q.col(2);
    H->block<4, 1>(4, 5) = -Q.col(0);
    H->block<4, 1>(8, 0) = -Q.col(3);
    H->block<4, 1>(8, 3) = -Q.col(1);
    H->block<4, 1>(8, 4) = Q.col(0);
    H->block<4, 1>(12, 0) = Q.col(2);
    H->block<4, 1>(12, 1) = -Q.col(1);
    H->block<4, 1>(12, 2) = Q.col(0);
  }
  return Eigen::Map<const SO4::VectorN2>(Q.data());
}

///******************************************************************************
template <>
GTSAM_EXPORT
SO4 SO4::ChartAtOrigin::Retract(const Vector6& xi, ChartJacobian H) {
  if (H) throw std::runtime_error("SO4::ChartAtOrigin::Retract Jacobian");
  gttic(SO4_Retract);
  const Matrix4 X = Hat(xi / 2);
  return SO4((I_4x4 + X) * (I_4x4 - X).inverse());
}

//******************************************************************************
template <>
GTSAM_EXPORT
Vector6 SO4::ChartAtOrigin::Local(const SO4& Q, ChartJacobian H) {
  if (H) throw std::runtime_error("SO4::ChartAtOrigin::Retract Jacobian");
  const Matrix4& R = Q.matrix();
  const Matrix4 X = (I_4x4 - R) * (I_4x4 + R).inverse();
  return -2 * Vee(X);
}

//******************************************************************************
GTSAM_EXPORT Matrix3 topLeft(const SO4& Q, OptionalJacobian<9, 6> H) {
  const Matrix4& R = Q.matrix();
  const Matrix3 M = R.topLeftCorner<3, 3>();
  if (H) {
    const Vector3 m1 = M.col(0), m2 = M.col(1), m3 = M.col(2),
                  q = R.topRightCorner<3, 1>();
    *H << Z_3x1, Z_3x1, -q, Z_3x1, -m3, m2,  //
        Z_3x1, q, Z_3x1, m3, Z_3x1, -m1,     //
        -q, Z_3x1, Z_3x1, -m2, m1, Z_3x1;
  }
  return M;
}

//******************************************************************************
GTSAM_EXPORT Matrix43 stiefel(const SO4& Q, OptionalJacobian<12, 6> H) {
  const Matrix4& R = Q.matrix();
  const Matrix43 M = R.leftCols<3>();
  if (H) {
    const auto &m1 = R.col(0), m2 = R.col(1), m3 = R.col(2), q = R.col(3);
    *H << Z_4x1, Z_4x1, -q, Z_4x1, -m3, m2,  //
        Z_4x1, q, Z_4x1, m3, Z_4x1, -m1,     //
        -q, Z_4x1, Z_4x1, -m2, m1, Z_4x1;
  }
  return M;
}

//******************************************************************************

}  // end namespace gtsam
