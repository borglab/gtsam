/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearFactor.h
 * @brief   Non-linear factor base classes
 * @author  Frank Dellaert
 * @author  Richard Roberts
 */

// \callgraph

#pragma once

#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/inference/Factor.h>
#include <gtsam/base/OptionalJacobian.h>

#include <boost/serialization/base_object.hpp>
#include <boost/assign/list_of.hpp>

namespace gtsam {

using boost::assign::cref_list_of;

/* ************************************************************************* */

/**
 * Nonlinear factor base class
 *
 * \nosubgrouping
 */
class GTSAM_EXPORT NonlinearFactor: public Factor {

protected:

  // Some handy typedefs
  typedef Factor Base;
  typedef NonlinearFactor This;

public:

  typedef boost::shared_ptr<This> shared_ptr;

  /// @name Standard Constructors
  /// @{

  /** Default constructor for I/O only */
  NonlinearFactor() {}

  /**
   * Constructor from a collection of the keys involved in this factor
   */
  template<typename CONTAINER>
  NonlinearFactor(const CONTAINER& keys) :
    Base(keys) {}

  /// @}
  /// @name Testable
  /// @{

  /** print */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;

  /** Check if two factors are equal */
  virtual bool equals(const NonlinearFactor& f, double tol = 1e-9) const;
  /// @}
  /// @name Standard Interface
  /// @{

  /** Destructor */
  virtual ~NonlinearFactor() {}


  /**
   * Calculate the error of the factor
   * This is typically equal to log-likelihood, e.g. \f$ 0.5(h(x)-z)^2/sigma^2 \f$ in case of Gaussian.
   * You can override this for systems with unusual noise models.
   */
  virtual double error(const Values& c) const = 0;

  /** get the dimension of the factor (number of rows on linearization) */
  virtual size_t dim() const = 0;

  /**
   * Checks whether a factor should be used based on a set of values.
   * This is primarily used to implement inequality constraints that
   * require a variable active set. For all others, the default implementation
   * returning true solves this problem.
   *
   * In an inequality/bounding constraint, this active() returns true
   * when the constraint is *NOT* fulfilled.
   * @return true if the constraint is active
   */
  virtual bool active(const Values& /*c*/) const { return true; }

  /** linearize to a GaussianFactor */
  virtual boost::shared_ptr<GaussianFactor>
  linearize(const Values& c) const = 0;

  /**
   * Creates a shared_ptr clone of the factor - needs to be specialized to allow
   * for subclasses
   *
   * By default, throws exception if subclass does not implement the function.
   */
  virtual shared_ptr clone() const {
    // TODO: choose better exception to throw here
    throw std::runtime_error("NonlinearFactor::clone(): Attempting to clone factor with no clone() implemented!");
    return shared_ptr();
  }

  /**
   * Creates a shared_ptr clone of the
   * factor with different keys using
   * a map from old->new keys
   */
  virtual shared_ptr rekey(const std::map<Key,Key>& rekey_mapping) const;

  /**
   * Clones a factor and fully replaces its keys
   * @param new_keys is the full replacement set of keys
   */
  virtual shared_ptr rekey(const KeyVector& new_keys) const;

  /**
   * Should the factor be evaluated in the same thread as the caller
   * This is to enable factors that has shared states (like the Python GIL lock)
   */
   virtual bool sendable() const {
    return true;
  }

}; // \class NonlinearFactor

/// traits
template<> struct traits<NonlinearFactor> : public Testable<NonlinearFactor> {
};

/* ************************************************************************* */
/**
 * A nonlinear sum-of-squares factor with a zero-mean noise model
 * implementing the density \f$ P(z|x) \propto exp -0.5*|z-h(x)|^2_C \f$
 * Templated on the parameter type X and the values structure Values
 * There is no return type specified for h(x). Instead, we require
 * the derived class implements \f$ \mathtt{error\_vector}(x) = h(x)-z \approx A \delta x - b \f$
 * This allows a graph to have factors with measurements of mixed type.

 * The noise model is typically Gaussian, but robust and constrained error models are also supported.
 */
class GTSAM_EXPORT NoiseModelFactor: public NonlinearFactor {

protected:

  // handy typedefs
  typedef NonlinearFactor Base;
  typedef NoiseModelFactor This;

  SharedNoiseModel noiseModel_; /** Noise model */

public:

  typedef boost::shared_ptr<This> shared_ptr;

  /** Default constructor for I/O only */
  NoiseModelFactor() {}

  /** Destructor */
  ~NoiseModelFactor() override {}

  /**
   * Constructor
   */
  template<typename CONTAINER>
  NoiseModelFactor(const SharedNoiseModel& noiseModel, const CONTAINER& keys) :
    Base(keys), noiseModel_(noiseModel) {}

protected:

  /**
   * Constructor - only for subclasses, as this does not set keys.
   */
  NoiseModelFactor(const SharedNoiseModel& noiseModel) : noiseModel_(noiseModel) {}

public:

  /** Print */
  void print(const std::string& s = "",
    const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

  /** Check if two factors are equal */
  bool equals(const NonlinearFactor& f, double tol = 1e-9) const override;

  /** get the dimension of the factor (number of rows on linearization) */
  size_t dim() const override {
    return noiseModel_->dim();
  }

  /// access to the noise model
  const SharedNoiseModel& noiseModel() const {
    return noiseModel_;
  }

  /**
   * Error function *without* the NoiseModel, \f$ z-h(x) \f$.
   * Override this method to finish implementing an N-way factor.
   * If the optional arguments is specified, it should compute
   * both the function evaluation and its derivative(s) in H.
   */
  virtual Vector unwhitenedError(const Values& x,
      boost::optional<std::vector<Matrix>&> H = boost::none) const = 0;

  /**
   * Vector of errors, whitened
   * This is the raw error, i.e., i.e. \f$ (h(x)-z)/\sigma \f$ in case of a Gaussian
   */
  Vector whitenedError(const Values& c) const;

  /**
   * Vector of errors, whitened, but unweighted by any loss function
   */
  Vector unweightedWhitenedError(const Values& c) const;

  /**
   * Compute the effective weight of the factor from the noise model.
   */
  double weight(const Values& c) const;

  /**
   * Calculate the error of the factor.
   * This is the log-likelihood, e.g. \f$ 0.5(h(x)-z)^2/\sigma^2 \f$ in case of Gaussian.
   * In this class, we take the raw prediction error \f$ h(x)-z \f$, ask the noise model
   * to transform it to \f$ (h(x)-z)^2/\sigma^2 \f$, and then multiply by 0.5.
   */
  double error(const Values& c) const override;

  /**
   * Linearize a non-linearFactorN to get a GaussianFactor,
   * \f$ Ax-b \approx h(x+\delta x)-z = h(x) + A \delta x - z \f$
   * Hence \f$ b = z - h(x) = - \mathtt{error\_vector}(x) \f$
   */
  boost::shared_ptr<GaussianFactor> linearize(const Values& x) const override;

  /**
   * Creates a shared_ptr clone of the
   * factor with a new noise model
   */
  shared_ptr cloneWithNewNoiseModel(const SharedNoiseModel newNoise) const;

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("NonlinearFactor",
         boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(noiseModel_);
  }

}; // \class NoiseModelFactor


/* ************************************************************************* */

/**
 * A convenient base class for creating your own NoiseModelFactor with 1
 * variable.  To derive from this class, implement evaluateError().
 *
 * Templated on a values structure type. The values structures are typically
 * more general than just vectors, e.g., Rot3 or Pose3,
 * which are objects in non-linear manifolds (Lie groups).
 */
template<class VALUE>
class NoiseModelFactor1: public NoiseModelFactor {

public:

  // typedefs for value types pulled from keys
  typedef VALUE X;

protected:

  typedef NoiseModelFactor Base;
  typedef NoiseModelFactor1<VALUE> This;

public:
  /// @name Constructors
  /// @{

  /** Default constructor for I/O only */
  NoiseModelFactor1() {}

  ~NoiseModelFactor1() override {}

  inline Key key() const { return keys_[0]; }

  /**
   *  Constructor
   *  @param noiseModel shared pointer to noise model
   *  @param key1 by which to look up X value in Values
   */
  NoiseModelFactor1(const SharedNoiseModel &noiseModel, Key key1)
      : Base(noiseModel, cref_list_of<1>(key1)) {}

  /// @}
  /// @name NoiseModelFactor methods
  /// @{

  /**
   * Calls the 1-key specific version of evaluateError below, which is pure
   * virtual so must be implemented in the derived class.
   */
  Vector unwhitenedError(
      const Values &x,
      boost::optional<std::vector<Matrix> &> H = boost::none) const override {
    if (this->active(x)) {
      const X &x1 = x.at<X>(keys_[0]);
      if (H) {
        return evaluateError(x1, (*H)[0]);
      } else {
        return evaluateError(x1);
      }
    } else {
      return Vector::Zero(this->dim());
    }
  }

  /// @}
  /// @name Virtual methods
  /// @{

  /**
   *  Override this method to finish implementing a unary factor.
   *  If the optional Matrix reference argument is specified, it should compute
   *  both the function evaluation and its derivative in X.
   */
  virtual Vector
  evaluateError(const X &x,
                boost::optional<Matrix &> H = boost::none) const = 0;

  /// @}

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("NoiseModelFactor",
        boost::serialization::base_object<Base>(*this));
  }
};// \class NoiseModelFactor1


/* ************************************************************************* */
/** A convenient base class for creating your own NoiseModelFactor with 2
 * variables.  To derive from this class, implement evaluateError(). */
template<class VALUE1, class VALUE2>
class NoiseModelFactor2: public NoiseModelFactor {

public:

  // typedefs for value types pulled from keys
  typedef VALUE1 X1;
  typedef VALUE2 X2;

protected:

  typedef NoiseModelFactor Base;
  typedef NoiseModelFactor2<VALUE1, VALUE2> This;

public:

  /**
   * Default Constructor for I/O
   */
  NoiseModelFactor2() {}

  /**
   * Constructor
   * @param noiseModel shared pointer to noise model
   * @param j1 key of the first variable
   * @param j2 key of the second variable
   */
  NoiseModelFactor2(const SharedNoiseModel& noiseModel, Key j1, Key j2) :
    Base(noiseModel, cref_list_of<2>(j1)(j2)) {}

  ~NoiseModelFactor2() override {}

  /** methods to retrieve both keys */
  inline Key key1() const { return keys_[0];  }
  inline Key key2() const {  return keys_[1];  }

  /** Calls the 2-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  Vector unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H = boost::none) const override {
    if(this->active(x)) {
      const X1& x1 = x.at<X1>(keys_[0]);
      const X2& x2 = x.at<X2>(keys_[1]);
      if(H) {
        return evaluateError(x1, x2, (*H)[0], (*H)[1]);
      } else {
        return evaluateError(x1, x2);
      }
    } else {
      return Vector::Zero(this->dim());
    }
  }

  /**
   *  Override this method to finish implementing a binary factor.
   *  If any of the optional Matrix reference arguments are specified, it should compute
   *  both the function evaluation and its derivative(s) in X1 (and/or X2).
   */
  virtual Vector
  evaluateError(const X1&, const X2&, boost::optional<Matrix&> H1 =
      boost::none, boost::optional<Matrix&> H2 = boost::none) const = 0;

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("NoiseModelFactor",
        boost::serialization::base_object<Base>(*this));
  }
}; // \class NoiseModelFactor2

/* ************************************************************************* */
/** A convenient base class for creating your own NoiseModelFactor with 3
 * variables.  To derive from this class, implement evaluateError(). */
template<class VALUE1, class VALUE2, class VALUE3>
class NoiseModelFactor3: public NoiseModelFactor {

public:

  // typedefs for value types pulled from keys
  typedef VALUE1 X1;
  typedef VALUE2 X2;
  typedef VALUE3 X3;

protected:

  typedef NoiseModelFactor Base;
  typedef NoiseModelFactor3<VALUE1, VALUE2, VALUE3> This;

public:

  /**
   * Default Constructor for I/O
   */
  NoiseModelFactor3() {}

  /**
   * Constructor
   * @param noiseModel shared pointer to noise model
   * @param j1 key of the first variable
   * @param j2 key of the second variable
   * @param j3 key of the third variable
   */
  NoiseModelFactor3(const SharedNoiseModel& noiseModel, Key j1, Key j2, Key j3) :
    Base(noiseModel, cref_list_of<3>(j1)(j2)(j3)) {}

  ~NoiseModelFactor3() override {}

  /** methods to retrieve keys */
  inline Key key1() const { return keys_[0]; }
  inline Key key2() const { return keys_[1]; }
  inline Key key3() const { return keys_[2]; }

  /** Calls the 3-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  Vector unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H = boost::none) const override {
    if(this->active(x)) {
      if(H)
        return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), (*H)[0], (*H)[1], (*H)[2]);
      else
        return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]));
    } else {
      return Vector::Zero(this->dim());
    }
  }

  /**
   *  Override this method to finish implementing a trinary factor.
   *  If any of the optional Matrix reference arguments are specified, it should compute
   *  both the function evaluation and its derivative(s) in X1 (and/or X2, X3).
   */
  virtual Vector
  evaluateError(const X1&, const X2&, const X3&,
      boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none,
      boost::optional<Matrix&> H3 = boost::none) const = 0;

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("NoiseModelFactor",
        boost::serialization::base_object<Base>(*this));
  }
}; // \class NoiseModelFactor3

/* ************************************************************************* */
/** A convenient base class for creating your own NoiseModelFactor with 4
 * variables.  To derive from this class, implement evaluateError(). */
template<class VALUE1, class VALUE2, class VALUE3, class VALUE4>
class NoiseModelFactor4: public NoiseModelFactor {

public:

  // typedefs for value types pulled from keys
  typedef VALUE1 X1;
  typedef VALUE2 X2;
  typedef VALUE3 X3;
  typedef VALUE4 X4;

protected:

  typedef NoiseModelFactor Base;
  typedef NoiseModelFactor4<VALUE1, VALUE2, VALUE3, VALUE4> This;

public:

  /**
   * Default Constructor for I/O
   */
  NoiseModelFactor4() {}

  /**
   * Constructor
   * @param noiseModel shared pointer to noise model
   * @param j1 key of the first variable
   * @param j2 key of the second variable
   * @param j3 key of the third variable
   * @param j4 key of the fourth variable
   */
  NoiseModelFactor4(const SharedNoiseModel& noiseModel, Key j1, Key j2, Key j3, Key j4) :
    Base(noiseModel, cref_list_of<4>(j1)(j2)(j3)(j4)) {}

  ~NoiseModelFactor4() override {}

  /** methods to retrieve keys */
  inline Key key1() const { return keys_[0]; }
  inline Key key2() const { return keys_[1]; }
  inline Key key3() const { return keys_[2]; }
  inline Key key4() const { return keys_[3]; }

  /** Calls the 4-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  Vector unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H = boost::none) const override {
    if(this->active(x)) {
      if(H)
        return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]), (*H)[0], (*H)[1], (*H)[2], (*H)[3]);
      else
        return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]));
    } else {
      return Vector::Zero(this->dim());
    }
  }

  /**
   *  Override this method to finish implementing a 4-way factor.
   *  If any of the optional Matrix reference arguments are specified, it should compute
   *  both the function evaluation and its derivative(s) in X1 (and/or X2, X3).
   */
  virtual Vector
  evaluateError(const X1&, const X2&, const X3&, const X4&,
      boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none,
      boost::optional<Matrix&> H3 = boost::none,
      boost::optional<Matrix&> H4 = boost::none) const = 0;

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("NoiseModelFactor",
        boost::serialization::base_object<Base>(*this));
  }
}; // \class NoiseModelFactor4

/* ************************************************************************* */
/** A convenient base class for creating your own NoiseModelFactor with 5
 * variables.  To derive from this class, implement evaluateError(). */
template<class VALUE1, class VALUE2, class VALUE3, class VALUE4, class VALUE5>
class NoiseModelFactor5: public NoiseModelFactor {

public:

  // typedefs for value types pulled from keys
  typedef VALUE1 X1;
  typedef VALUE2 X2;
  typedef VALUE3 X3;
  typedef VALUE4 X4;
  typedef VALUE5 X5;

protected:

  typedef NoiseModelFactor Base;
  typedef NoiseModelFactor5<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5> This;

public:

  /**
   * Default Constructor for I/O
   */
  NoiseModelFactor5() {}

  /**
   * Constructor
   * @param noiseModel shared pointer to noise model
   * @param j1 key of the first variable
   * @param j2 key of the second variable
   * @param j3 key of the third variable
   * @param j4 key of the fourth variable
   * @param j5 key of the fifth variable
   */
  NoiseModelFactor5(const SharedNoiseModel& noiseModel, Key j1, Key j2, Key j3, Key j4, Key j5) :
    Base(noiseModel, cref_list_of<5>(j1)(j2)(j3)(j4)(j5)) {}

  ~NoiseModelFactor5() override {}

  /** methods to retrieve keys */
  inline Key key1() const { return keys_[0]; }
  inline Key key2() const { return keys_[1]; }
  inline Key key3() const { return keys_[2]; }
  inline Key key4() const { return keys_[3]; }
  inline Key key5() const { return keys_[4]; }

  /** Calls the 5-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  Vector unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H = boost::none) const override {
    if(this->active(x)) {
      if(H)
        return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]), x.at<X5>(keys_[4]), (*H)[0], (*H)[1], (*H)[2], (*H)[3], (*H)[4]);
      else
        return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]), x.at<X5>(keys_[4]));
    } else {
      return Vector::Zero(this->dim());
    }
  }

  /**
   *  Override this method to finish implementing a 5-way factor.
   *  If any of the optional Matrix reference arguments are specified, it should compute
   *  both the function evaluation and its derivative(s) in X1 (and/or X2, X3).
   */
  virtual Vector
  evaluateError(const X1&, const X2&, const X3&, const X4&, const X5&,
      boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none,
      boost::optional<Matrix&> H3 = boost::none,
      boost::optional<Matrix&> H4 = boost::none,
      boost::optional<Matrix&> H5 = boost::none) const = 0;

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("NoiseModelFactor",
        boost::serialization::base_object<Base>(*this));
  }
}; // \class NoiseModelFactor5

/* ************************************************************************* */
/** A convenient base class for creating your own NoiseModelFactor with 6
 * variables.  To derive from this class, implement evaluateError(). */
template<class VALUE1, class VALUE2, class VALUE3, class VALUE4, class VALUE5, class VALUE6>
class NoiseModelFactor6: public NoiseModelFactor {

public:

  // typedefs for value types pulled from keys
  typedef VALUE1 X1;
  typedef VALUE2 X2;
  typedef VALUE3 X3;
  typedef VALUE4 X4;
  typedef VALUE5 X5;
  typedef VALUE6 X6;

protected:

  typedef NoiseModelFactor Base;
  typedef NoiseModelFactor6<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5, VALUE6> This;

public:

  /**
   * Default Constructor for I/O
   */
  NoiseModelFactor6() {}

  /**
   * Constructor
   * @param noiseModel shared pointer to noise model
   * @param j1 key of the first variable
   * @param j2 key of the second variable
   * @param j3 key of the third variable
   * @param j4 key of the fourth variable
   * @param j5 key of the fifth variable
   * @param j6 key of the fifth variable
   */
  NoiseModelFactor6(const SharedNoiseModel& noiseModel, Key j1, Key j2, Key j3, Key j4, Key j5, Key j6) :
    Base(noiseModel, cref_list_of<6>(j1)(j2)(j3)(j4)(j5)(j6)) {}

  ~NoiseModelFactor6() override {}

  /** methods to retrieve keys */
  inline Key key1() const { return keys_[0]; }
  inline Key key2() const { return keys_[1]; }
  inline Key key3() const { return keys_[2]; }
  inline Key key4() const { return keys_[3]; }
  inline Key key5() const { return keys_[4]; }
  inline Key key6() const { return keys_[5]; }

  /** Calls the 6-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  Vector unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H = boost::none) const override {
    if(this->active(x)) {
      if(H)
        return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]), x.at<X5>(keys_[4]), x.at<X6>(keys_[5]), (*H)[0], (*H)[1], (*H)[2], (*H)[3], (*H)[4], (*H)[5]);
      else
        return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]), x.at<X5>(keys_[4]), x.at<X6>(keys_[5]));
    } else {
      return Vector::Zero(this->dim());
    }
  }

  /**
   *  Override this method to finish implementing a 6-way factor.
   *  If any of the optional Matrix reference arguments are specified, it should compute
   *  both the function evaluation and its derivative(s) in X1 (and/or X2, X3).
   */
  virtual Vector
  evaluateError(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&,
      boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none,
      boost::optional<Matrix&> H3 = boost::none,
      boost::optional<Matrix&> H4 = boost::none,
      boost::optional<Matrix&> H5 = boost::none,
      boost::optional<Matrix&> H6 = boost::none) const = 0;

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("NoiseModelFactor",
        boost::serialization::base_object<Base>(*this));
  }
}; // \class NoiseModelFactor6

/* ************************************************************************* */

} // \namespace gtsam
