/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file StereoPoint2.h
 * @brief A 2D stereo point (uL,uR,v)
 * @date Jan 26, 2010
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/geometry/Point2.h>
#include <gtsam/base/VectorSpace.h>
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/nvp.hpp>
#endif

namespace gtsam {

/**
 * A 2D stereo point, v will be same for rectified images
 * @ingroup geometry
 * \nosubgrouping
 */
class GTSAM_EXPORT StereoPoint2 {
private:

  double uL_, uR_, v_;

public:
  enum { dimension = 3 };
  /// @name Standard Constructors
  /// @{

  /** default constructor */
  StereoPoint2() :
      uL_(0), uR_(0), v_(0) {
  }

  /** constructor */
  StereoPoint2(double uL, double uR, double v) :
      uL_(uL), uR_(uR), v_(v) {
  }

  /// construct from 3D vector
  explicit StereoPoint2(const Vector3& v) :
    uL_(v(0)), uR_(v(1)), v_(v(2)) {}

  /// @}
  /// @name Testable
  /// @{

  /** print */
  void print(const std::string& s = "") const;

  /** equals */
  bool equals(const StereoPoint2& q, double tol = 1e-9) const {
    return (std::abs(uL_ - q.uL_) < tol && std::abs(uR_ - q.uR_) < tol
        && std::abs(v_ - q.v_) < tol);
  }

  /// @}
  /// @name Group
  /// @{

  /// identity
  inline static StereoPoint2 Identity() {
    return StereoPoint2();
  }

  /// inverse
  StereoPoint2 operator-() const {
    return StereoPoint2(-uL_, -uR_, -v_);
  }

  /// add vector on right
  inline StereoPoint2 operator +(const Vector3& v) const {
    return StereoPoint2(uL_ + v[0], uR_ + v[1], v_ + v[2]);
  }

  /// add
  inline StereoPoint2 operator +(const StereoPoint2& b) const {
    return StereoPoint2(uL_ + b.uL_, uR_ + b.uR_, v_ + b.v_);
  }

  /// subtract
  inline StereoPoint2 operator -(const StereoPoint2& b) const {
    return StereoPoint2(uL_ - b.uL_, uR_ - b.uR_, v_ - b.v_);
  }

  /// @}
  /// @name Standard Interface
  /// @{

  /// equality
  inline bool operator ==(const StereoPoint2& q) const {return uL_== q.uL_ && uR_==q.uR_ && v_ == q.v_;}

  /// get uL
  inline double uL() const {return uL_;}

  /// get uR
  inline double uR() const {return uR_;}

  /// get v
  inline double v() const {return v_;}

  /** convert to vector */
  Vector3 vector() const {
    return Vector3(uL_, uR_, v_);
  }

  /** convenient function to get a Point2 from the left image */
  Point2 point2() const {
    return Point2(uL_, v_);
  }

  /** convenient function to get a Point2 from the right image */
  Point2 right() const {
    return Point2(uR_, v_);
  }

  /// @name Deprecated
  /// @{
  inline StereoPoint2 inverse() const { return StereoPoint2()- (*this);}
  inline StereoPoint2 compose(const StereoPoint2& p1) const { return *this + p1;}
  inline StereoPoint2 between(const StereoPoint2& p2) const { return p2 - *this; }
  inline Vector localCoordinates(const StereoPoint2& t2) const { return Logmap(between(t2)); }
  inline StereoPoint2 retract(const Vector& v) const { return compose(Expmap(v)); }
  static inline Vector Logmap(const StereoPoint2& p) { return p.vector(); }
  static inline StereoPoint2 Expmap(const Vector& d) { return StereoPoint2(d(0), d(1), d(2)); }
  /// @}

  /// Streaming
  GTSAM_EXPORT friend std::ostream &operator<<(std::ostream &os, const StereoPoint2& p);

private:

  /// @}
  /// @name Advanced Interface
  /// @{

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(uL_);
    ar & BOOST_SERIALIZATION_NVP(uR_);
    ar & BOOST_SERIALIZATION_NVP(v_);
  }
#endif

  /// @}

};

typedef std::vector<StereoPoint2> StereoPoint2Vector;

template<>
struct traits<StereoPoint2> : public internal::VectorSpace<StereoPoint2> {};

template<>
struct traits<const StereoPoint2> : public internal::VectorSpace<StereoPoint2> {};
}
