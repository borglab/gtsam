/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file BearingRangeFactor.h
 * @date Apr 1, 2010
 * @author Kai Ni
 * @brief a single factor contains both the bearing and the range to prevent
 * handle to pair bearing and range factors
 */

#pragma once

#include <gtsam/nonlinear/SerializableExpressionFactor.h>
#include <gtsam/base/Testable.h>
#include <boost/concept/assert.hpp>

namespace gtsam {

// forward declaration of Bearing functor, assumed partially specified
template <typename A1, typename A2>
struct Bearing;

// forward declaration of Range functor, assumed partially specified
template <typename A1, typename A2>
struct Range;

template <typename B, typename R>
struct BearingRange : public ProductManifold<B, R> {
  BearingRange() {}
  BearingRange(const ProductManifold<B, R>& br) : ProductManifold<B, R>(br) {}
  BearingRange(const B& b, const R& r) : ProductManifold<B, R>(b, r) {}

 private:
  /// Serialization function
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp("bearing", this->first);
    ar& boost::serialization::make_nvp("range", this->second);
  }

  friend class boost::serialization::access;
};

template <typename B, typename R>
struct traits<BearingRange<B, R> >
    : internal::ManifoldTraits<BearingRange<B, R> > {
  static void Print(const BearingRange<B, R>& m, const std::string& str = "") {
    traits<B>::Print(m.first, str);
    traits<R>::Print(m.second, str);
  }
  static bool Equals(const BearingRange<B, R>& m1, const BearingRange<B, R>& m2,
                     double tol = 1e-8) {
    return traits<B>::Equals(m1.first, m2.first, tol) &&
           traits<R>::Equals(m1.second, m2.second, tol);
  }
};

/**
 * Binary factor for a bearing/range measurement
 * @addtogroup SLAM
 */
template <typename A1, typename A2,
          typename B = typename Bearing<A1, A2>::result_type,
          typename R = typename Range<A1, A2>::result_type>
class BearingRangeFactor
    : public SerializableExpressionFactor2<BearingRange<B, R>, A1, A2> {
 public:
  typedef BearingRange<B, R> T;
  typedef SerializableExpressionFactor2<T, A1, A2> Base;
  typedef BearingRangeFactor<A1, A2> This;
  typedef boost::shared_ptr<This> shared_ptr;

 private:
  /** concept check by type */
  BOOST_CONCEPT_ASSERT((IsTestable<B>));
  BOOST_CONCEPT_ASSERT((IsTestable<R>));

 public:
  /// default constructor
  BearingRangeFactor() {}

  /// primary constructor
  BearingRangeFactor(Key key1, Key key2, const B& measuredBearing,
                     const R measuredRange, const SharedNoiseModel& model)
      : Base(key1, key2, model, T(measuredBearing, measuredRange)) {
    this->initialize(expression(key1, key2));
  }

  virtual ~BearingRangeFactor() {}

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  // Return measurement expression
  virtual Expression<T> expression(Key key1, Key key2) const {
    Expression<A1> a1_(key1);
    Expression<A2> a2_(key2);
    return Expression<T>(This::Predict, a1_, a2_);
  }

  /** Print */
  virtual void print(
      const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
    std::cout << s << "BearingRangeFactor(" << keyFormatter(this->keys_[0])
              << "," << keyFormatter(this->keys_[1]) << ")\n";
    traits<B>::Print(this->measured_.first, "measured bearing: ");
    traits<R>::Print(this->measured_.second, "measured range: ");
    this->noiseModel_->print("noise model:\n");
  }

  /// Prediction function that stacks measurements
  static T Predict(const A1& pose, const A2& point,
                   typename MakeOptionalJacobian<T, A1>::type H1,
                   typename MakeOptionalJacobian<T, A2>::type H2) {
    typename MakeJacobian<B, A1>::type HB1;
    typename MakeJacobian<B, A2>::type HB2;
    typename MakeJacobian<R, A1>::type HR1;
    typename MakeJacobian<R, A2>::type HR2;

    B b = Bearing<A1, A2>()(pose, point, H1 ? &HB1 : 0, H2 ? &HB2 : 0);
    R r = Range<A1, A2>()(pose, point, H1 ? &HR1 : 0, H2 ? &HR2 : 0);

    if (H1) *H1 << HB1, HR1;
    if (H2) *H2 << HB2, HR2;
    return T(b, r);
  }

};  // BearingRangeFactor

/// traits
template <typename A1, typename A2, typename B, typename R>
struct traits<BearingRangeFactor<A1, A2, B, R> >
    : public Testable<BearingRangeFactor<A1, A2, B, R> > {};

}  // namespace gtsam
