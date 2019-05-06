/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SO4.h
 * @brief   4*4 matrix representation of SO(4)
 * @author  Frank Dellaert
 * @author  Luca Carlone
 * @date    March 2019
 */

#pragma once

#include <gtsam/geometry/SOn.h>

#include <gtsam/base/Group.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/Manifold.h>
#include <gtsam/base/Matrix.h>

#include <boost/random/mersenne_twister.hpp>

#include <iosfwd>
#include <string>

namespace gtsam {

using SO4 = SO<4>;

// /// Random SO(4) element (no big claims about uniformity)
// static SO4 Random(boost::mt19937 &rng);

// static Matrix4 Hat(const Vector6 &xi);  ///< make skew symmetric matrix
// static Vector6 Vee(const Matrix4 &X);   ///< inverse of Hat
// static SO4 Expmap(const Vector6 &xi,
//                   ChartJacobian H = boost::none);  ///< exponential map
// static Vector6 Logmap(const SO4 &Q,
//                       ChartJacobian H = boost::none);  ///< and its inverse
// Matrix6 AdjointMap() const;

//******************************************************************************
/* Exponential map, porting MATLAB implementation by Luca, which follows
 * "SOME REMARKS ON THE EXPONENTIAL MAP ON THE GROUPS SO(n) AND SE(n)" by
 * Ramona-Andreaa Rohan */
template <>
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
  Matrix4 expX;
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
static SO4::VectorN2 vec4(const Matrix4& Q) {
  return Eigen::Map<const SO4::VectorN2>(Q.data());
}

static const std::vector<const Matrix4> G4(
    {SO4::Hat(Vector6::Unit(0)), SO4::Hat(Vector6::Unit(1)),
     SO4::Hat(Vector6::Unit(2)), SO4::Hat(Vector6::Unit(3)),
     SO4::Hat(Vector6::Unit(4)), SO4::Hat(Vector6::Unit(5))});

static const Eigen::Matrix<double, 16, 6> P4 =
    (Eigen::Matrix<double, 16, 6>() << vec4(G4[0]), vec4(G4[1]), vec4(G4[2]),
     vec4(G4[3]), vec4(G4[4]), vec4(G4[5]))
        .finished();

//******************************************************************************
template <>
Matrix6 SO4::AdjointMap() const {
  // Elaborate way of calculating the AdjointMap
  // TODO(frank): find a closed form solution. In SO(3) is just R :-/
  const Matrix4& Q = matrix_;
  const Matrix4 Qt = Q.transpose();
  Matrix6 A;
  for (size_t i = 0; i < 6; i++) {
    // Calculate column i of linear map for coeffcient of Gi
    A.col(i) = SO4::Vee(Q * G4[i] * Qt);
  }
  return A;
}

//******************************************************************************
template <>
SO4::VectorN2 SO4::vec(OptionalJacobian<16, 6> H) const {
  const Matrix& Q = matrix_;
  if (H) {
    // As Luca calculated, this is (I4 \oplus Q) * P4
    *H << Q * P4.block<4, 6>(0, 0), Q * P4.block<4, 6>(4, 0),
        Q * P4.block<4, 6>(8, 0), Q * P4.block<4, 6>(12, 0);
  }
  return gtsam::vec4(Q);
}

// /// Vectorize
// Vector16 vec(OptionalJacobian<16, 6> H = boost::none) const;

// /// Project to top-left 3*3 matrix. Note this is *not* in general \in SO(3).
// Matrix3 topLeft(OptionalJacobian<9, 6> H = boost::none) const;

// /// Project to Stiefel manifold of 4*3 orthonormal 3-frames in R^4, i.e.,
// pi(Q) -> S \in St(3,4). Matrix43 stiefel(OptionalJacobian<12, 6> H =
// boost::none) const;

//  private:
//   /** Serialization function */
//   friend class boost::serialization::access;
//   template <class ARCHIVE>
//   void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
//     ar &boost::serialization::make_nvp("Q11", (*this)(0, 0));
//     ar &boost::serialization::make_nvp("Q12", (*this)(0, 1));
//     ar &boost::serialization::make_nvp("Q13", (*this)(0, 2));
//     ar &boost::serialization::make_nvp("Q14", (*this)(0, 3));

//     ar &boost::serialization::make_nvp("Q21", (*this)(1, 0));
//     ar &boost::serialization::make_nvp("Q22", (*this)(1, 1));
//     ar &boost::serialization::make_nvp("Q23", (*this)(1, 2));
//     ar &boost::serialization::make_nvp("Q24", (*this)(1, 3));

//     ar &boost::serialization::make_nvp("Q31", (*this)(2, 0));
//     ar &boost::serialization::make_nvp("Q32", (*this)(2, 1));
//     ar &boost::serialization::make_nvp("Q33", (*this)(2, 2));
//     ar &boost::serialization::make_nvp("Q34", (*this)(2, 3));

//     ar &boost::serialization::make_nvp("Q41", (*this)(3, 0));
//     ar &boost::serialization::make_nvp("Q42", (*this)(3, 1));
//     ar &boost::serialization::make_nvp("Q43", (*this)(3, 2));
//     ar &boost::serialization::make_nvp("Q44", (*this)(3, 3));
//   }

/*
 * Define the traits. internal::LieGroup provides both Lie group and Testable
 */

template <>
struct traits<SO4> : Testable<SO4>, internal::LieGroupTraits<SO4> {};

template <>
struct traits<const SO4> : Testable<SO4>, internal::LieGroupTraits<SO4> {};

}  // end namespace gtsam
