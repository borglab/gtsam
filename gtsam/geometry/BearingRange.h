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
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/nvp.hpp>
#endif
#include <iostream>

namespace gtsam {

// Forward declaration of Bearing functor which should be of A1*A2 -> return_type
// For example Bearing<Pose3,Point3>(pose,point), defined in Pose3.h will return Unit3
// At time of writing only Pose2 and Pose3 specialize this functor.
template <typename A1, typename A2>
struct Bearing;

// Forward declaration of Range functor which should be of A1*A2 -> return_type
// For example Range<Pose2,Pose2>(T1,T2), defined in Pose2.h  will return double
// At time of writing Pose2, Pose3, and several Camera variants specialize this for several types
template <typename A1, typename A2>
struct Range;

/**
 * Bearing-Range product for a particular A1,A2 combination will use the functors above to create
 * a similar functor of type A1*A2 -> pair<Bearing::return_type,Range::return_type>
 * For example BearingRange<Pose2,Point2>(pose,point) will return pair<Rot2,double>
 * and BearingRange<Pose3,Point3>(pose,point) will return pair<Unit3,double>
 */
template <typename A1, typename A2,
          typename B = typename Bearing<A1, A2>::result_type,
          typename R = typename Range<A1, A2>::result_type>
struct BearingRange {
private:
  B bearing_;
  R range_;

public:
  enum { dimB = traits<B>::dimension };
  enum { dimR = traits<R>::dimension };
  enum { dimension = dimB + dimR };

  /// @name Standard Constructors
  /// @{

  BearingRange() {}
  BearingRange(const B& b, const R& r) : bearing_(b), range_(r) {}

  /// @}
  /// @name Standard Interface
  /// @{

  /// Return bearing measurement
  const B& bearing() const { return bearing_; }

  /// Return range measurement
  const R& range() const { return range_; }

  /// Prediction function that stacks measurements
  static BearingRange Measure(
    const A1& a1, const A2& a2,
    OptionalJacobian<dimension, traits<A1>::dimension> H1 = {},
    OptionalJacobian<dimension, traits<A2>::dimension> H2 = {}) {
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

  /// Predict bearing
  static B MeasureBearing(const A1& a1, const A2& a2) {
    return Bearing<A1, A2>()(a1, a2);
  }

  /// Predict range
  static R MeasureRange(const A1& a1, const A2& a2) {
    return Range<A1, A2>()(a1, a2);
  }

  /// @}
  /// @name Testable
  /// @{

  void print(const std::string& str = "") const {
    std::cout << str;
    traits<B>::Print(bearing_, "bearing ");
    traits<R>::Print(range_, "range ");
  }
  bool equals(const BearingRange<A1, A2>& m2, double tol = 1e-8) const {
    return traits<B>::Equals(bearing_, m2.bearing_, tol) &&
      traits<R>::Equals(range_, m2.range_, tol);
  }

  /// @}
  /// @name Manifold
  /// @{

  inline static size_t Dim() { return dimension; }
  inline size_t dim() const { return dimension; }

  typedef Eigen::Matrix<double, dimension, 1> TangentVector;
  typedef OptionalJacobian<dimension, dimension> ChartJacobian;

  /// Retract delta to manifold
  BearingRange retract(const TangentVector& xi) const {
    B m1 = traits<B>::Retract(bearing_, xi.template head<dimB>());
    R m2 = traits<R>::Retract(range_, xi.template tail<dimR>());
    return BearingRange(m1, m2);
  }

  /// Compute the coordinates in the tangent space
  TangentVector localCoordinates(const BearingRange& other) const {
    typename traits<B>::TangentVector v1 = traits<B>::Local(bearing_, other.bearing_);
    typename traits<R>::TangentVector v2 = traits<R>::Local(range_, other.range_);
    TangentVector v;
    v << v1, v2;
    return v;
  }

  /// @}
  /// @name Advanced Interface
  /// @{

private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /// Serialization function
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp("bearing", bearing_);
    ar& boost::serialization::make_nvp("range", range_);
  }

  friend class boost::serialization::access;
#endif

  /// @}

  // Alignment, see https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
  enum {
    NeedsToAlign = (sizeof(B) % 16) == 0 || (sizeof(R) % 16) == 0
  };
public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
};

// Declare this to be both Testable and a Manifold
template <typename A1, typename A2>
struct traits<BearingRange<A1, A2> >
  : Testable<BearingRange<A1, A2> >,
  internal::ManifoldTraits<BearingRange<A1, A2> > {};

// Helper class for to implement Range traits for classes with a bearing method
// For example, to specialize Bearing to Pose3 and Point3, using Pose3::bearing, it suffices to say
//   template <> struct Bearing<Pose3, Point3> : HasBearing<Pose3, Point3, Unit3> {};
// where the third argument is used to indicate the return type
template <class A1, typename A2, class RT>
struct HasBearing {
  typedef RT result_type;
  RT operator()(
      const A1& a1, const A2& a2,
      OptionalJacobian<traits<RT>::dimension, traits<A1>::dimension> H1={},
      OptionalJacobian<traits<RT>::dimension, traits<A2>::dimension> H2={}) {
    return a1.bearing(a2, H1, H2);
  }
};

// Similar helper class for to implement Range traits for classes with a range method
// For classes with overloaded range methods, such as PinholeCamera, this can even be templated:
//   template <typename T> struct Range<PinholeCamera, T> : HasRange<PinholeCamera, T, double> {};
template <class A1, typename A2, class RT>
struct HasRange {
  typedef RT result_type;
  RT operator()(
      const A1& a1, const A2& a2,
      OptionalJacobian<traits<RT>::dimension, traits<A1>::dimension> H1={},
      OptionalJacobian<traits<RT>::dimension, traits<A2>::dimension> H2={}) {
    return a1.range(a2, H1, H2);
  }
};

}  // namespace gtsam
