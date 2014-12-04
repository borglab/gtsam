/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file AdaptAutoDiff.h
 * @date October 22, 2014
 * @author Frank Dellaert
 * @brief Adaptor for Ceres style auto-differentiated functions
 */

#pragma once

#include <gtsam/3rdparty/ceres/autodiff.h>
#include <gtsam/base/Manifold.h>
#include <gtsam/base/OptionalJacobian.h>

namespace gtsam {

/// Adapt ceres-style autodiff
template<typename F, typename T, typename A1, typename A2>
class AdaptAutoDiff {

  static const int N = traits::dimension<T>::value;
  static const int M1 = traits::dimension<A1>::value;
  static const int M2 = traits::dimension<A2>::value;

  typedef Eigen::Matrix<double, N, M1, Eigen::RowMajor> RowMajor1;
  typedef Eigen::Matrix<double, N, M2, Eigen::RowMajor> RowMajor2;

  typedef Canonical<T> CanonicalT;
  typedef Canonical<A1> Canonical1;
  typedef Canonical<A2> Canonical2;

  typedef typename CanonicalT::vector VectorT;
  typedef typename Canonical1::vector Vector1;
  typedef typename Canonical2::vector Vector2;

  // Instantiate function and charts
  CanonicalT chartT;
  Canonical1 chart1;
  Canonical2 chart2;
  F f;

public:

  T operator()(const A1& a1, const A2& a2, OptionalJacobian<N, M1> H1 = boost::none,
      OptionalJacobian<N, M2> H2 = boost::none) {

    using ceres::internal::AutoDiff;

    // Make arguments
    Vector1 v1 = chart1.local(a1);
    Vector2 v2 = chart2.local(a2);

    bool success;
    VectorT result;

    if (H1 || H2) {

      // Get derivatives with AutoDiff
      double *parameters[] = { v1.data(), v2.data() };
      double rowMajor1[N * M1], rowMajor2[N * M2]; // om the stack
      double *jacobians[] = { rowMajor1, rowMajor2 };
      success = AutoDiff<F, double, 9, 3>::Differentiate(f, parameters, 2,
          result.data(), jacobians);

      // Convert from row-major to columnn-major
      // TODO: if this is a bottleneck (probably not!) fix Autodiff to be Column-Major
      *H1 = Eigen::Map<RowMajor1>(rowMajor1);
      *H2 = Eigen::Map<RowMajor2>(rowMajor2);

    } else {
      // Apply the mapping, to get result
      success = f(v1.data(), v2.data(), result.data());
    }
    if (!success)
      throw std::runtime_error(
          "AdaptAutoDiff: function call resulted in failure");
    return chartT.retract(result);
  }

};

}
