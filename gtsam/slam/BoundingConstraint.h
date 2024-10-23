/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file BoundingConstraint.h
 * @brief Provides partially implemented constraints to implement bounds
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/base/Lie.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/constraint/NonlinearInequalityConstraint.h>

namespace gtsam {

/**
 * Unary inequality constraint forcing a scalar to be
 * greater/less than a fixed threshold.  The function
 * will need to have its value function implemented to return
 * a scalar for comparison.
 * @ingroup slam
 */
template<class VALUE>
struct BoundingConstraint1: public NonlinearInequalityConstraint {
  typedef VALUE X;
  typedef NonlinearInequalityConstraint Base;
  typedef std::shared_ptr<BoundingConstraint1<VALUE> > shared_ptr;

  double threshold_;
  bool isGreaterThan_; /// flag for greater/less than

  BoundingConstraint1(Key key, double threshold,
      bool isGreaterThan, double mu = 1000.0) :
        Base(noiseModel::Constrained::All(1, mu), KeyVector{key}),
        threshold_(threshold), isGreaterThan_(isGreaterThan) {
  }

  ~BoundingConstraint1() override {}

  inline double threshold() const { return threshold_; }
  inline bool isGreaterThan() const { return isGreaterThan_; }
  inline Key key() const { return keys().front(); }

  /**
   * function producing a scalar value to compare to the threshold
   * Must have optional argument for derivative with 1xN matrix, where
   * N = X::dim()
   */
  virtual double value(const X& x, OptionalMatrixType H =
      OptionalNone) const = 0;

  Vector unwhitenedExpr(const Values& x, OptionalMatrixVecType H = nullptr) const override {
    if (H) {
      double d = value(x.at<X>(this->key()), &(H->front()));
      if (isGreaterThan_) {
        H->front() *= -1;
        return Vector1(threshold_ - d);
      } else {
        return Vector1(d - threshold_);
      }
    } else {
      double d = value(x.at<X>(this->key()));
      return Vector1((isGreaterThan_) ? threshold_ - d : d - threshold_);
    }
  }

  /// TODO: This should be deprecated.
  Vector evaluateError(const X& x, OptionalMatrixType H = nullptr) const {
    Matrix D;
    double error = value(x, &D) - threshold_;
    if (H) {
      if (isGreaterThan_) *H = D;
      else *H = -1.0 * D;
    }

    if (isGreaterThan_)
      return (Vector(1) << error).finished();
    else
      return -1.0 * (Vector(1) << error).finished();
  }

private:

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    // NoiseModelFactor1 instead of NoiseModelFactorN for backward compatibility
    ar & boost::serialization::make_nvp("NoiseModelFactor1",
        boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(threshold_);
    ar & BOOST_SERIALIZATION_NVP(isGreaterThan_);
  }
#endif
};

/**
 * Binary scalar inequality constraint, with a similar value() function
 * to implement for specific systems
 */
template<class VALUE1, class VALUE2>
struct BoundingConstraint2: public NonlinearInequalityConstraint {
  typedef VALUE1 X1;
  typedef VALUE2 X2;

  typedef NonlinearInequalityConstraint Base;
  typedef std::shared_ptr<BoundingConstraint2<VALUE1, VALUE2> > shared_ptr;

  double threshold_;
  bool isGreaterThan_; /// flag for greater/less than

  BoundingConstraint2(Key key1, Key key2, double threshold,
      bool isGreaterThan, double mu = 1000.0)
  : Base(noiseModel::Constrained::All(1, mu), KeyVector{key1, key2}),
    threshold_(threshold), isGreaterThan_(isGreaterThan) {}

  ~BoundingConstraint2() override {}

  inline double threshold() const { return threshold_; }
  inline bool isGreaterThan() const { return isGreaterThan_; }

  /**
   * function producing a scalar value to compare to the threshold
   * Must have optional argument for derivatives)
   */
  virtual double value(const X1& x1, const X2& x2,
      OptionalMatrixType H1 = OptionalNone,
      OptionalMatrixType H2 = OptionalNone) const = 0;

  Vector unwhitenedExpr(const Values& x, OptionalMatrixVecType H = nullptr) const override {
    X1 x1 = x.at<X1>(keys().front());
    X2 x2 = x.at<X2>(keys().back());
    if (H) {
      double d = value(x1, x2, &(H->front()), &(H->back()));
      if (isGreaterThan_) {
        H->front() *= -1;
        H->back() *= -1;
        return Vector1(threshold_ - d);
      } else {
        return Vector1(d - threshold_);
      }
    } else {
      double d = value(x1, x2);
      return Vector1((isGreaterThan_) ? threshold_ - d : d - threshold_);
    }
  }

  /// TODO: This should be deprecated.
  Vector evaluateError(const X1& x1, const X2& x2,
      OptionalMatrixType H1 = nullptr, OptionalMatrixType H2 = nullptr) const {
    Matrix D1, D2;
    double error = value(x1, x2, &D1, &D2) - threshold_;
    if (H1) {
      if (isGreaterThan_)  *H1 = D1;
      else *H1 = -1.0 * D1;
    }
    if (H2) {
      if (isGreaterThan_) *H2 = D2;
      else *H2 = -1.0 * D2;
    }

    if (isGreaterThan_)
      return (Vector(1) << error).finished();
    else
      return -1.0 * (Vector(1) << error).finished();
  }

private:

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    // NoiseModelFactor2 instead of NoiseModelFactorN for backward compatibility
    ar & boost::serialization::make_nvp("NoiseModelFactor2",
        boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(threshold_);
    ar & BOOST_SERIALIZATION_NVP(isGreaterThan_);
  }
#endif
};

} // \namespace gtsam
