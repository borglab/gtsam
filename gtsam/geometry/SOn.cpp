/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SOn.cpp
 * @brief   Definitions of dynamic specializations of SO(n)
 * @author  Frank Dellaert
 * @author  Varun Agrawal
 * @date    March 2019
 */

#include <gtsam/geometry/SOn.h>

namespace gtsam {

template <>
GTSAM_EXPORT
Matrix SOn::Hat(const Vector& xi) {
  size_t n = AmbientDim(xi.size());
  if (n < 2) throw std::invalid_argument("SO<N>::Hat: n<2 not supported");

  Matrix X(n, n);  // allocate space for n*n skew-symmetric matrix
  X.setZero();
  if (n == 2) {
    // Handle SO(2) case as recursion bottom
    assert(xi.size() == 1);
    X << 0, -xi(0), xi(0), 0;
  } else {
    // Recursively call SO(n-1) call for top-left block
    const size_t dmin = (n - 1) * (n - 2) / 2;
    X.topLeftCorner(n - 1, n - 1) = Hat(xi.tail(dmin));

    // determine sign of last element (signs alternate)
    double sign = pow(-1.0, xi.size());
    // Now fill last row and column
    for (size_t i = 0; i < n - 1; i++) {
      const size_t j = n - 2 - i;
      X(n - 1, j) = -sign * xi(i);
      X(j, n - 1) = -X(n - 1, j);
      sign = -sign;
    }
  }
  return X;
}

template <>
GTSAM_EXPORT
Vector SOn::Vee(const Matrix& X) {
  const size_t n = X.rows();
  if (n < 2) throw std::invalid_argument("SO<N>::Hat: n<2 not supported");

  if (n == 2) {
    // Handle SO(2) case as recursion bottom
    Vector xi(1);
    xi(0) = X(1, 0);
    return xi;
  } else {
    // Calculate dimension and allocate space
    const size_t d = n * (n - 1) / 2;
    Vector xi(d);

    // Fill first n-1 spots from last row of X
    double sign = pow(-1.0, xi.size());
    for (size_t i = 0; i < n - 1; i++) {
      const size_t j = n - 2 - i;
      xi(i) = -sign * X(n - 1, j);
      sign = -sign;
    }

    // Recursively call Vee to fill remainder of x
    const size_t dmin = (n - 1) * (n - 2) / 2;
    xi.tail(dmin) = Vee(X.topLeftCorner(n - 1, n - 1));
    return xi;
  }
}

template <>
SOn LieGroup<SOn, Eigen::Dynamic>::compose(const SOn& g, DynamicJacobian H1,
                                           DynamicJacobian H2) const {
  if (H1) *H1 = g.inverse().AdjointMap();
  if (H2) *H2 = SOn::IdentityJacobian(g.rows());
  return derived() * g;
}

template <>
SOn LieGroup<SOn, Eigen::Dynamic>::between(const SOn& g, DynamicJacobian H1,
                                           DynamicJacobian H2) const {
  SOn result = derived().inverse() * g;
  if (H1) *H1 = -result.inverse().AdjointMap();
  if (H2) *H2 = SOn::IdentityJacobian(g.rows());
  return result;
}

}  // namespace gtsam
