/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file NonlinearEquality.h
 * @brief Factor to handle enforced equality between factors
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Manifold.h>

#include <boost/bind/bind.hpp>

#include <limits>
#include <iostream>
#include <cmath>

namespace gtsam {

/**
 * An equality factor that forces either one variable to a constant,
 * or a set of variables to be equal to each other.
 *
 * Depending on flag, throws an error at linearization if the constraints are not met.
 *
 * Switchable implementation:
 *   - ALLLOW_ERROR : if we allow that there can be nonzero error, does not throw, and uses gain
 *   - ONLY_EXACT   : throws error at linearization if not at exact feasible point, and infinite error
 *
 * \nosubgrouping
 */
template<class VALUE>
class NonlinearEquality: public NoiseModelFactorN<VALUE> {

public:
  typedef VALUE T;

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
  using Base = NoiseModelFactorN<VALUE>;

public:

  /// Function that compares two values.
  using CompareFunction = std::function<bool(const T&, const T&)>;
  CompareFunction compare_;

  /// Default constructor - only for serialization
  NonlinearEquality() {
  }

  ~NonlinearEquality() override {
  }

  /// @name Standard Constructors
  /// @{

  /**
   * Constructor - forces exact evaluation
   */
  NonlinearEquality(Key j, const T& feasible,
      const CompareFunction &_compare = std::bind(traits<T>::Equals,
          std::placeholders::_1, std::placeholders::_2, 1e-9)) :
      Base(noiseModel::Constrained::All(traits<T>::GetDimension(feasible)),
          j), feasible_(feasible), allow_error_(false), error_gain_(0.0), //
      compare_(_compare) {
  }

  /**
   * Constructor - allows inexact evaluation
   */
  NonlinearEquality(Key j, const T& feasible, double error_gain,
      const CompareFunction &_compare = std::bind(traits<T>::Equals,
          std::placeholders::_1, std::placeholders::_2, 1e-9)) :
      Base(noiseModel::Constrained::All(traits<T>::GetDimension(feasible)),
          j), feasible_(feasible), allow_error_(true), error_gain_(error_gain), //
      compare_(_compare) {
  }

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

  /// Error function
  Vector evaluateError(const T& xj,
      OptionalMatrixType H = OptionalNone) const override {
    const size_t nj = traits<T>::GetDimension(feasible_);
    if (allow_error_) {
      if (H)
        *H = Matrix::Identity(nj,nj); // FIXME: this is not the right linearization for nonlinear compare
      return traits<T>::Local(xj,feasible_);
    } else if (compare_(feasible_, xj)) {
      if (H)
        *H = Matrix::Identity(nj,nj);
      return Vector::Zero(nj); // set error to zero if equal
    } else {
      if (H)
        throw std::invalid_argument(
            "Linearization point not feasible for "
                + DefaultKeyFormatter(this->key()) + "!");
      return Vector::Constant(nj, std::numeric_limits<double>::infinity()); // set error to infinity if not equal
    }
  }

  /// Linearize is over-written, because base linearization tries to whiten
  GaussianFactor::shared_ptr linearize(const Values& x) const override {
    const T& xj = x.at<T>(this->key());
    Matrix A;
    Vector b = evaluateError(xj, A);
    SharedDiagonal model = noiseModel::Constrained::All(b.size());
    return GaussianFactor::shared_ptr(
        new JacobianFactor(this->key(), A, b, model));
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// @}

  GTSAM_MAKE_ALIGNED_OPERATOR_NEW

private:

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

};
// \class NonlinearEquality

template <typename VALUE>
struct traits<NonlinearEquality<VALUE>> : Testable<NonlinearEquality<VALUE>> {};

/* ************************************************************************* */
/**
 * Simple unary equality constraint - fixes a value for a variable
 */
template<class VALUE>
class NonlinearEquality1: public NoiseModelFactorN<VALUE> {

public:
  typedef VALUE X;

protected:
  typedef NoiseModelFactorN<VALUE> Base;
  typedef NonlinearEquality1<VALUE> This;

  /// Default constructor to allow for serialization
  NonlinearEquality1() {
  }

  X value_; /// fixed value for variable

  GTSAM_CONCEPT_MANIFOLD_TYPE(X)
  GTSAM_CONCEPT_TESTABLE_TYPE(X)

public:

  typedef boost::shared_ptr<NonlinearEquality1<VALUE> > shared_ptr;

  /**
   * Constructor
   * @param value feasible value that values(key) shouild be equal to
   * @param key the key for the unknown variable to be constrained
   * @param mu a parameter which really turns this into a strong prior
   */
  NonlinearEquality1(const X& value, Key key, double mu = 1000.0)
      : Base(noiseModel::Constrained::All(traits<X>::GetDimension(value),
                                          std::abs(mu)),
             key),
        value_(value) {}

  ~NonlinearEquality1() override {
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// g(x) with optional derivative
  Vector evaluateError(const X& x1,
      OptionalMatrixType H = OptionalNone) const override {
    if (H)
      (*H) = Matrix::Identity(traits<X>::GetDimension(x1),traits<X>::GetDimension(x1));
    // manifold equivalent of h(x)-z -> log(z,h(x))
    return traits<X>::Local(value_,x1);
  }

  /// Print
  void print(const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << ": NonlinearEquality1(" << keyFormatter(this->key())
        << ")," << "\n";
    this->noiseModel_->print();
    traits<X>::Print(value_, "Value");
  }

  GTSAM_MAKE_ALIGNED_OPERATOR_NEW

private:

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
};
// \NonlinearEquality1

template <typename VALUE>
struct traits<NonlinearEquality1<VALUE> >
    : Testable<NonlinearEquality1<VALUE> > {};

/* ************************************************************************* */
/**
 * Simple binary equality constraint - this constraint forces two variables to
 * be the same.
 */
template <class T>
class NonlinearEquality2 : public NoiseModelFactorN<T, T> {
 protected:
  using Base = NoiseModelFactorN<T, T>;
  using This = NonlinearEquality2<T>;

  GTSAM_CONCEPT_MANIFOLD_TYPE(T)

  /// Default constructor to allow for serialization
  NonlinearEquality2() {}

 public:
  typedef boost::shared_ptr<NonlinearEquality2<T>> shared_ptr;

  /**
   * Constructor
   * @param key1 the key for the first unknown variable to be constrained
   * @param key2 the key for the second unknown variable to be constrained
   * @param mu a parameter which really turns this into a strong prior
   */
  NonlinearEquality2(Key key1, Key key2, double mu = 1e4)
      : Base(noiseModel::Constrained::All(traits<T>::dimension, std::abs(mu)),
             key1, key2) {}
  ~NonlinearEquality2() override {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// g(x) with optional derivative2
  Vector evaluateError(
      const T& x1, const T& x2, OptionalMatrixType H1 = OptionalNone,
      OptionalMatrixType H2 = OptionalNone) const override {
    static const size_t p = traits<T>::dimension;
    if (H1) *H1 = -Matrix::Identity(p, p);
    if (H2) *H2 = Matrix::Identity(p, p);
    return traits<T>::Local(x1, x2);
  }

  GTSAM_MAKE_ALIGNED_OPERATOR_NEW

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    // NoiseModelFactor2 instead of NoiseModelFactorN for backward compatibility
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor2", boost::serialization::base_object<Base>(*this));
  }
};
// \NonlinearEquality2

template <typename VALUE>
struct traits<NonlinearEquality2<VALUE>> : Testable<NonlinearEquality2<VALUE>> {
};

}// namespace gtsam
