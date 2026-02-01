/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file NonlinearEquality.h
 * @brief Factor to handle equality constraints
 * @author Alex Cunningham
 * @author Frank Dellaert
 * @author Yetong Zhang
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Manifold.h>

#include <limits>
#include <iostream>
#include <cmath>

namespace gtsam {

/**
 * Equality constraint that pins a single variable to a constant value.
 *
 * Behavior is controlled by allow_error_:
 *   - Exact mode (default): throws at linearization if the current value is not
 *     equal to the feasible point (within compare_), and returns infinite
 * error.
 *   - Allow-error mode: returns a smooth squared error scaled by error_gain_.
 *
 * \nosubgrouping
 */
template<class VALUE>
class NonlinearEquality: public NonlinearEqualityConstraint {

 public:
  using T = VALUE;

 private:

  // feasible value
  T feasible_;

  // error handling flag
  bool allow_error_;

  // error gain in allow error case
  double error_gain_;

  // typedef to this class
  using This = NonlinearEquality<VALUE>;

  // typedef to base class
  using Base = NonlinearEqualityConstraint;

  GTSAM_CONCEPT_MANIFOLD_TYPE(T)
  GTSAM_CONCEPT_TESTABLE_TYPE(T)

 public:

  /// Function that compares two values.
  using CompareFunction = std::function<bool(const T&, const T&)>;
  CompareFunction compare_;

  /// Default constructor - only for serialization
  NonlinearEquality() {}

  ~NonlinearEquality() override {}

  /// @name Standard Constructors
  /// @{

  /**
   * Constructor - exact equality, throws on infeasible linearization points.
   */
  NonlinearEquality(Key j, const T& feasible,
      const CompareFunction &_compare = std::bind(traits<T>::Equals,
          std::placeholders::_1, std::placeholders::_2, 1e-9)) :
      Base(noiseModel::Constrained::All(traits<T>::GetDimension(feasible)),
          KeyVector{j}), feasible_(feasible), allow_error_(false), error_gain_(0.0), //
      compare_(_compare) {
  }

  /**
   * Constructor - allows inexact evaluation with a smooth penalty.
   */
  NonlinearEquality(Key j, const T& feasible, double error_gain,
      const CompareFunction &_compare = std::bind(traits<T>::Equals,
          std::placeholders::_1, std::placeholders::_2, 1e-9)) :
      Base(noiseModel::Constrained::All(traits<T>::GetDimension(feasible)),
          KeyVector{j}), feasible_(feasible), allow_error_(true), error_gain_(error_gain), //
      compare_(_compare) {
  }

  Key key() const { return keys().front(); }

  /// @}
  /// @name Testable
  /// @{

  void print(const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << (s.empty() ? s : s + " ") << "Constraint: on ["
              << keyFormatter(this->key()) << "]\n";
    traits<VALUE>::Print(feasible_, "Feasible Point:\n");
    std::cout << "Variable Dimension: " << traits<T>::GetDimension(feasible_)
              << std::endl;
  }

  /** Check if two factors are equal */
  bool equals(const NonlinearFactor& f, double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&f);
    return e && Base::equals(f) && traits<T>::Equals(feasible_,e->feasible_, tol)
        && std::abs(error_gain_ - e->error_gain_) < tol;
  }

  /// @}
  /// @name Standard Interface
  /// @{

  /// Actual error function calculation
  double error(const Values& c) const override {
    const T& xj = c.at<T>(this->key());
    Vector e = this->unwhitenedError(c);
    if (allow_error_ || !compare_(xj, feasible_)) {
      return error_gain_ * dot(e, e);
    } else {
      return 0.0;
    }
  }

  /// Whether this constraint should be treated as hard.
  bool isHardConstraint() const override { return !allow_error_; }

  /// Error function
  Vector evaluateError(const T& xj, OptionalMatrixType H = nullptr) const {
    const size_t nj = traits<T>::GetDimension(feasible_);
    if (H) *H = Matrix::Identity(nj, nj);
    if (allow_error_) {
      return traits<T>::Local(xj, feasible_);
    } else if (compare_(feasible_, xj)) {
      return Vector::Zero(nj);  // set error to zero if equal
    } else {
      if (H)
        throw std::invalid_argument("Linearization point not feasible for " +
                                    DefaultKeyFormatter(this->key()) + "!");
      // set error to infinity if not equal
      return Vector::Constant(nj, std::numeric_limits<double>::infinity());
    }
  }

  Vector unwhitenedError(const Values& x, OptionalMatrixVecType H = nullptr) const override {
    VALUE x1 = x.at<VALUE>(key());
    if (H) {
      return evaluateError(x1, &(H->front()));
    } else {
      return evaluateError(x1);
    }
  }

  /// Linearize is over-written, because base linearization tries to whiten
  GaussianFactor::shared_ptr linearize(const Values& x) const override {
    const T& xj = x.at<T>(this->key());
    Matrix A;
    Vector b = evaluateError(xj, &A);
    SharedDiagonal model = noiseModel::Constrained::All(b.size());
    return GaussianFactor::shared_ptr(
        new JacobianFactor(this->key(), A, b, model));
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// @}

  GTSAM_MAKE_ALIGNED_OPERATOR_NEW

 private:

#if GTSAM_ENABLE_BOOST_SERIALIZATION  ///
  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    // NoiseModelFactor1 instead of NoiseModelFactorN for backward compatibility
    ar
        & boost::serialization::make_nvp("NoiseModelFactor1",
            boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(feasible_);
    ar & BOOST_SERIALIZATION_NVP(allow_error_);
    ar & BOOST_SERIALIZATION_NVP(error_gain_);
  }
#endif

};
// \class NonlinearEquality

template <typename VALUE>
struct traits<NonlinearEquality<VALUE>> : Testable<NonlinearEquality<VALUE>> {};

/* ************************************************************************* */
/**
 * Simple unary equality constraint - fixes a value for a variable.
 *
 * @deprecated Use NonlinearEquality (exact or allow-error mode) for
 * constraints, or use a PriorFactor for a soft constraint with a conventional
 * noise model.
 */
#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43
template<class VALUE>
class NonlinearEquality1: public NonlinearEqualityConstraint {
 public:
  typedef VALUE T;

 protected:
  typedef NonlinearEqualityConstraint Base;
  typedef NonlinearEquality1<VALUE> This;

  /// Default constructor to allow for serialization
  NonlinearEquality1() {
  }

  T value_; /// fixed value for variable

  GTSAM_CONCEPT_MANIFOLD_TYPE(T)
  GTSAM_CONCEPT_TESTABLE_TYPE(T)

 public:

  typedef std::shared_ptr<NonlinearEquality1<VALUE> > shared_ptr;

  /**
   * Constructor
   * @param value feasible value that values(key) should be equal to
   * @param key the key for the unknown variable to be constrained
   * @param mu a parameter which really turns this into a strong prior
   */
  NonlinearEquality1(const T& value, Key key, double mu = 1000.0)
      : Base(noiseModel::Constrained::All(traits<T>::GetDimension(value),
                                          std::abs(mu)),
             KeyVector{key}),
        value_(value) {}

  ~NonlinearEquality1() override {
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  Key key() const { return keys().front(); }

  /// g(x) with optional derivative
  Vector evaluateError(const T& x1, OptionalMatrixType H = nullptr) const {
    if (H)
      (*H) = Matrix::Identity(traits<T>::GetDimension(x1),traits<T>::GetDimension(x1));
    // manifold equivalent of h(x)-z -> log(z,h(x))
    return traits<T>::Local(value_,x1);
  }

  Vector unwhitenedError(const Values& x, OptionalMatrixVecType H = nullptr) const override {
    T x1 = x.at<T>(key());
    if (H) {
      return evaluateError(x1, &(H->front()));
    } else {
      return evaluateError(x1);
    }
  }

  /// Print
  void print(const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << ": NonlinearEquality1(" << keyFormatter(this->key())
              << ")," << "\n";
    this->noiseModel_->print();
    traits<T>::Print(value_, "Value");
  }

  GTSAM_MAKE_ALIGNED_OPERATOR_NEW

 private:

#if GTSAM_ENABLE_BOOST_SERIALIZATION  ///
  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    // NoiseModelFactor1 instead of NoiseModelFactorN for backward compatibility
    ar
        & boost::serialization::make_nvp("NoiseModelFactor1",
            boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(value_);
  }
#endif
};
// \NonlinearEquality1

template <typename VALUE>
struct traits<NonlinearEquality1<VALUE> >
    : Testable<NonlinearEquality1<VALUE> > {};
#endif

/* ************************************************************************* */
/**
 * Simple binary equality constraint - this constraint forces two variables to
 * be the same.
 */
template <class T>
class NonlinearEquality2 : public NonlinearEqualityConstraint {
 protected:
  typedef NonlinearEqualityConstraint Base;
  typedef NonlinearEquality2<T> This;

  GTSAM_CONCEPT_MANIFOLD_TYPE(T)
  GTSAM_CONCEPT_TESTABLE_TYPE(T)

  /// Default constructor to allow for serialization
  NonlinearEquality2() {}

 public:
  typedef std::shared_ptr<NonlinearEquality2<T>> shared_ptr;

  /**
   * Constructor
   * @param key1 the key for the first unknown variable to be constrained
   * @param key2 the key for the second unknown variable to be constrained
   * @param mu a parameter which really turns this into a strong prior
   */
  NonlinearEquality2(Key key1, Key key2, double mu = 1e4)
      : Base(noiseModel::Constrained::All(traits<T>::dimension, std::abs(mu)),
             KeyVector{key1, key2}) {}

  ~NonlinearEquality2() override {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// g(x) with optional derivative2
  Vector evaluateError(
      const T& x1, const T& x2, OptionalMatrixType H1 = nullptr, OptionalMatrixType H2 = nullptr) const {
    static const size_t p = traits<T>::dimension;
    if (H1) *H1 = -Matrix::Identity(p, p);
    if (H2) *H2 = Matrix::Identity(p, p);
    return traits<T>::Local(x1, x2);
  }

  Vector unwhitenedError(const Values& x, OptionalMatrixVecType H = nullptr) const override {
    T x1 = x.at<T>(keys().front());
    T x2 = x.at<T>(keys().back());
    if (H) {
      return evaluateError(x1, x2, &(H->front()), &(H->back()));
    } else {
      return evaluateError(x1, x2);
    }
  }

  GTSAM_MAKE_ALIGNED_OPERATOR_NEW

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    // NoiseModelFactor2 instead of NoiseModelFactorN for backward compatibility
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor2", boost::serialization::base_object<Base>(*this));
  }
#endif
};
// \NonlinearEquality2

template <typename VALUE>
struct traits<NonlinearEquality2<VALUE>> : Testable<NonlinearEquality2<VALUE>> {
};

}// namespace gtsam
