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

#include <gtsam/base/VectorSpace.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/3rdparty/ceres/autodiff.h>

#include <boost/static_assert.hpp>
#include <boost/type_traits/is_base_of.hpp>

namespace gtsam {

namespace detail {

// By default, we assume an Identity element
template<typename T, typename structure_category>
struct Origin { T operator()() { return traits_x<T>::Identity();} };

// but dimple manifolds don't have one, so we just use the default constructor
template<typename T>
struct Origin<T, manifold_tag> { T operator()() { return T();} };

} // \ detail

/**
 * Canonical is a template that creates canonical coordinates for a given type.
 * A simple manifold type (i.e., not a Lie Group) has no concept of identity,
 * hence in that case we use the value given by the default constructor T() as
 * the origin of a "canonical coordinate" parameterization.
 */
template<typename T>
struct Canonical {

  GTSAM_CONCEPT_MANIFOLD_TYPE(T)

  typedef traits_x<T> Traits;
  enum { dimension = Traits::dimension };
  typedef typename Traits::TangentVector TangentVector;
  typedef typename Traits::structure_category Category;
  typedef detail::Origin<T, Category> Origin;

  static TangentVector Local(const T& other) {
    return Traits::Local(Origin()(), other);
  }

  static T Retract(const TangentVector& v) {
    return Traits::Retract(Origin()(), v);
  }
};

/// Adapt ceres-style autodiff
template<typename F, typename T, typename A1, typename A2>
class AdaptAutoDiff {

  static const int N = traits_x<T>::dimension;
  static const int M1 = traits_x<A1>::dimension;
  static const int M2 = traits_x<A2>::dimension;

  typedef Eigen::Matrix<double, N, M1, Eigen::RowMajor> RowMajor1;
  typedef Eigen::Matrix<double, N, M2, Eigen::RowMajor> RowMajor2;

  typedef Canonical<T> CanonicalT;
  typedef Canonical<A1> Canonical1;
  typedef Canonical<A2> Canonical2;

  typedef typename CanonicalT::TangentVector VectorT;
  typedef typename Canonical1::TangentVector Vector1;
  typedef typename Canonical2::TangentVector Vector2;

  F f;

public:

  T operator()(const A1& a1, const A2& a2, OptionalJacobian<N, M1> H1 = boost::none,
      OptionalJacobian<N, M2> H2 = boost::none) {

    using ceres::internal::AutoDiff;

    // Make arguments
    Vector1 v1 = Canonical1::Local(a1);
    Vector2 v2 = Canonical2::Local(a2);

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
    return CanonicalT::Retract(result);
  }

};

}
