/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file BearingRange.h
 * @date July, 2015
 * @author Frank Dellaert
 * @brief Bearing-Range product
 */

#pragma once

#include <gtsam/base/Manifold.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/OptionalJacobian.h>
#include <boost/concept/assert.hpp>

namespace gtsam {

// forward declaration of Bearing functor, assumed partially specified
template <typename A1, typename A2>
struct Bearing;

// forward declaration of Range functor, assumed partially specified
template <typename A1, typename A2>
struct Range;

/**
 * Bearing-Range product for a particular A1,A2 combination
 */
template <typename A1, typename A2>
struct BearingRange
    : public ProductManifold<typename Bearing<A1, A2>::result_type,
                             typename Range<A1, A2>::result_type> {
  typedef typename Bearing<A1, A2>::result_type B;
  typedef typename Range<A1, A2>::result_type R;
  typedef ProductManifold<B, R> Base;

  BearingRange() {}
  BearingRange(const ProductManifold<B, R>& br) : Base(br) {}
  BearingRange(const B& b, const R& r) : Base(b, r) {}

  /// Prediction function that stacks measurements
  static BearingRange Measure(
      const A1& a1, const A2& a2,
      OptionalJacobian<Base::dimension, traits<A1>::dimension> H1 = boost::none,
      OptionalJacobian<Base::dimension, traits<A2>::dimension> H2 =
          boost::none) {
    typename MakeJacobian<B, A1>::type HB1;
    typename MakeJacobian<B, A2>::type HB2;
    typename MakeJacobian<R, A1>::type HR1;
    typename MakeJacobian<R, A2>::type HR2;

    B b = Bearing<A1, A2>()(a1, a2, H1 ? &HB1 : 0, H2 ? &HB2 : 0);
    R r = Range<A1, A2>()(a1, a2, H1 ? &HR1 : 0, H2 ? &HR2 : 0);

    if (H1) *H1 << HB1, HR1;
    if (H2) *H2 << HB2, HR2;
    return BearingRange(b, r);
  }

  void print(const std::string& str = "") const {
    std::cout << str;
    traits<B>::Print(this->first, "bearing ");
    traits<R>::Print(this->second, "range ");
  }
  bool equals(const BearingRange<A1, A2>& m2, double tol = 1e-8) const {
    return traits<B>::Equals(this->first, m2.first, tol) &&
           traits<R>::Equals(this->second, m2.second, tol);
  }

 private:
  /// Serialization function
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp("bearing", this->first);
    ar& boost::serialization::make_nvp("range", this->second);
  }

  friend class boost::serialization::access;
};

template <typename A1, typename A2>
struct traits<BearingRange<A1, A2> >
    : Testable<BearingRange<A1, A2> >,
      internal::ManifoldTraits<BearingRange<A1, A2> > {};

}  // namespace gtsam
