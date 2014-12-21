/*
 * concepts.h
 *
 * @date Dec 4, 2014
 * @author Mike Bosse
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/base/Group.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/OptionalJacobian.h>

#include <boost/concept_check.hpp>
#include <boost/concept/requires.hpp>
#include <boost/static_assert.hpp>
#include <boost/type_traits/is_base_of.hpp>

// This is a helper to ease the transition to the new traits defined in this file.
// Uncomment this if you want all methods that are tagged as not implemented
// to cause compilation errors.
// #define COMPILE_ERROR_NOT_IMPLEMENTED

#ifdef COMPILE_ERROR_NOT_IMPLEMENTED
#define CONCEPT_NOT_IMPLEMENTED BOOST_STATIC_ASSERT_MSG(boost::false_type, \
"This method is required by the new concepts framework but has not been implemented yet.");
#else
#define CONCEPT_NOT_IMPLEMENTED \
  throw std::runtime_error("This method is required by the new concepts framework but has not been implemented yet.");
#endif

namespace gtsam {

template <typename T> struct traits_x;

template<typename ManifoldType>
struct Canonical {
  BOOST_STATIC_ASSERT_MSG(
      (boost::is_base_of<group_tag, typename traits_x<ManifoldType>::structure_category>::value),
      "This type's trait does not assert it is a manifold (or derived)");
  typedef traits_x<ManifoldType> Traits;
  typedef typename Traits::TangentVector TangentVector;
  enum { dimension = Traits::dimension };
  typedef OptionalJacobian<dimension, dimension> ChartJacobian;

  static TangentVector Local(const ManifoldType& other) {
    return Traits::Local(Traits::Identity(), other);
  }

  static ManifoldType Retract(const TangentVector& v) {
    return Traits::Retract(Traits::Identity(), v);
  }

  static TangentVector Local(const ManifoldType& other,
                             ChartJacobian Hother) {
    return Traits::Local(Traits::Identity(), other, boost::none, Hother);
  }

  static ManifoldType Retract(const ManifoldType& origin,
                              const TangentVector& v,
                              ChartJacobian Horigin,
                              ChartJacobian Hv) {
    return Traits::Retract(Traits::Identity(), v, boost::none, Hv);
  }
};

} // namespace gtsam

