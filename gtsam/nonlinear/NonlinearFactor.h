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

#include <gtsam/inference/HypoTree.h>

#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/inference/Factor.h>
#include <gtsam/base/OptionalJacobian.h>

#include <boost/serialization/shared_ptr.hpp>

#include <boost/serialization/base_object.hpp>
#include <boost/assign/list_of.hpp>

/**
 * Macro to add a standard clone function to a derived factor
 * @deprecated: will go away shortly - just add the clone function directly
 */
#define ADD_CLONE_NONLINEAR_FACTOR(Derived) \
  virtual gtsam::NonlinearFactor::shared_ptr clone() const { \
  return boost::static_pointer_cast<gtsam::NonlinearFactor>( \
      gtsam::NonlinearFactor::shared_ptr(new Derived(*this))); }

namespace gtsam {

class HypoNode;
class HypoLayer;
class HypoTree;


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

  typedef std::list<HypoNode*> HypoList; //used in this.cpp
    

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
  virtual void print(const std::string& s = "",
    const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

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
   * This is primarily used to implment inequality constraints that
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
  linearize(const Values& c) const = 0; //NonlinearFactor
  
  //[MH-A]: use a different function name allows using original single-mode factors in MH...
  virtual boost::shared_ptr<GaussianFactor>
  mhLinearize(const Values& c) const { //NonlinearFactor
    std::cout << "NonlinearFactor::mhLinearize() return NULL" << std::endl;
    return NULL;
  }

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
  shared_ptr rekey(const std::map<Key,Key>& rekey_mapping) const;

  /**
   * Clones a factor and fully replaces its keys
   * @param new_keys is the full replacement set of keys
   */
  shared_ptr rekey(const std::vector<Key>& new_keys) const;

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
  virtual ~NoiseModelFactor() {}

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
  virtual void print(const std::string& s = "",
    const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /** Check if two factors are equal */
  virtual bool equals(const NonlinearFactor& f, double tol = 1e-9) const;

  /** get the dimension of the factor (number of rows on linearization) */
  virtual size_t dim() const {
    return noiseModel_->dim();
  }

  /// access to the noise model
  const SharedNoiseModel& noiseModel() const {
    return noiseModel_;
  }

  /// @deprecated access to the noise model
  SharedNoiseModel get_noiseModel() const {
    return noiseModel_;
  }

  /**
   * Error function *without* the NoiseModel, \f$ z-h(x) \f$.
   * Override this method to finish implementing an N-way factor.
   * If the optional arguments is specified, it should compute
   * both the function evaluation and its derivative(s) in H.
   */
  virtual Vector unwhitenedError(const Values& x,
      boost::optional<std::vector<Matrix>&> H = boost::none) const = 0; //NoiseModelFactor

  /**
   * Vector of errors, whitened
   * This is the raw error, i.e., i.e. \f$ (h(x)-z)/\sigma \f$ in case of a Gaussian
   */
  Vector whitenedError(const Values& c) const;

  /**
   * Calculate the error of the factor.
   * This is the log-likelihood, e.g. \f$ 0.5(h(x)-z)^2/\sigma^2 \f$ in case of Gaussian.
   * In this class, we take the raw prediction error \f$ h(x)-z \f$, ask the noise model
   * to transform it to \f$ (h(x)-z)^2/\sigma^2 \f$, and then multiply by 0.5.
   */
  virtual double error(const Values& c) const;

  /**
   * Linearize a non-linearFactorN to get a GaussianFactor,
   * \f$ Ax-b \approx h(x+\delta x)-z = h(x) + A \delta x - z \f$
   * Hence \f$ b = z - h(x) = - \mathtt{error\_vector}(x) \f$
   */
  boost::shared_ptr<GaussianFactor> linearize(const Values& x) const; //NoiseModelFactor
  
  //[MH-A]: NOT to use single-mode factor in MH...
  boost::shared_ptr<GaussianFactor> mhLinearize(const Values& x) const {//NoiseModelFactor
    std::cout << "NoiseModelFactor::mhLinearize() return NULL" << std::endl;
    return NULL;
  }
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

  /** Default constructor for I/O only */
  NoiseModelFactor1() {}

  virtual ~NoiseModelFactor1() {}

  inline Key key() const { return keys_[0]; }

  /**
   *  Constructor
   *  @param noiseModel shared pointer to noise model
   *  @param key1 by which to look up X value in Values
   */
  NoiseModelFactor1(const SharedNoiseModel& noiseModel, Key key1) :
    Base(noiseModel, cref_list_of<1>(key1)) {}

  /** Calls the 1-key specific version of evaluateError, which is pure virtual
   *  so must be implemented in the derived class.
   */
  virtual Vector unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H = boost::none) const { //mhsiao: NoiseModelFactor1
    if(this->active(x)) {
      const X& x1 = x.at<X>(keys_[0]);
      if(H) {
        return evaluateError(x1, (*H)[0]);
      } else {
        return evaluateError(x1);
      }
    } else {
      return Vector::Zero(this->dim());
    }
  }

  /**
   *  Override this method to finish implementing a unary factor.
   *  If the optional Matrix reference argument is specified, it should compute
   *  both the function evaluation and its derivative in X.
   */
  virtual Vector evaluateError(const X& x, boost::optional<Matrix&> H =
      boost::none) const = 0;

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

  virtual ~NoiseModelFactor2() {}

  /** methods to retrieve both keys */
  inline Key key1() const { return keys_[0];  }
  inline Key key2() const {  return keys_[1];  }

  /** Calls the 2-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  virtual Vector unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H = boost::none) const { //mhsiao: NoiseModelFactor2
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

  virtual ~NoiseModelFactor3() {}

  /** methods to retrieve keys */
  inline Key key1() const { return keys_[0]; }
  inline Key key2() const { return keys_[1]; }
  inline Key key3() const { return keys_[2]; }

  /** Calls the 3-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  virtual Vector unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H = boost::none) const { //mhsiao: NoiseModelFactor3
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

  virtual ~NoiseModelFactor4() {}

  /** methods to retrieve keys */
  inline Key key1() const { return keys_[0]; }
  inline Key key2() const { return keys_[1]; }
  inline Key key3() const { return keys_[2]; }
  inline Key key4() const { return keys_[3]; }

  /** Calls the 4-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  virtual Vector unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H = boost::none) const { //mhsiao: NoiseModelFactor4
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

  virtual ~NoiseModelFactor5() {}

  /** methods to retrieve keys */
  inline Key key1() const { return keys_[0]; }
  inline Key key2() const { return keys_[1]; }
  inline Key key3() const { return keys_[2]; }
  inline Key key4() const { return keys_[3]; }
  inline Key key5() const { return keys_[4]; }

  /** Calls the 5-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  virtual Vector unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H = boost::none) const { //mhsiao: NoiseModelFactor5
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

  virtual ~NoiseModelFactor6() {}

  /** methods to retrieve keys */
  inline Key key1() const { return keys_[0]; }
  inline Key key2() const { return keys_[1]; }
  inline Key key3() const { return keys_[2]; }
  inline Key key4() const { return keys_[3]; }
  inline Key key5() const { return keys_[4]; }
  inline Key key6() const { return keys_[5]; }

  /** Calls the 6-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  virtual Vector unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H = boost::none) const { //mhsiao: NoiseModelFactor6
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
//======================================== MHNoiseModelFactor ===============================================

class GTSAM_EXPORT MHNoiseModelFactor: public NonlinearFactor {

protected:

  // handy typedefs
  typedef NonlinearFactor Base;
  typedef MHNoiseModelFactor This;

  /** Noise model */ 
  //[MH-G] mhsiao: Both format can be applied:
  // 1) All hypos share the same NoiseModel (only use front()...)
  // 2) Each mode has its own NoiseModel (use entire arr_...)
  std::vector<SharedNoiseModel> noiseModel_arr_;

public:  
  //[MH-A]: The 1st reason why we need a MHNoiseModelFactor
  HypoLayer* creating_layer_; //creating_layer_ is NULL when there's only one mode of this factor... //TODO: should we modify existing single mode factors?

  mutable int max_key_idx_; //record the key_idx that corresponds to the value that has the most # hypo currently

public:

  typedef boost::shared_ptr<This> shared_ptr;

  /** Default constructor for I/O only */
  MHNoiseModelFactor() : creating_layer_(NULL) {}

  /** Destructor */
  virtual ~MHNoiseModelFactor() {}

  /**
   * Constructor
   */
  template<typename CONTAINER>
  MHNoiseModelFactor(const SharedNoiseModel& noiseModel, const CONTAINER& keys) :
    Base(keys), noiseModel_arr_(1, noiseModel), creating_layer_(NULL) {} //the union of all Keys
  
  //[MH-G]: each mode has its own NoiseModel
  template<typename CONTAINER>
  MHNoiseModelFactor(const std::vector<SharedNoiseModel>& noiseModel_arr, const CONTAINER& keys) :
    Base(keys), noiseModel_arr_(noiseModel_arr.begin(), noiseModel_arr.end()), creating_layer_(NULL) {} //the union of all Keys

protected:

  /**
   * Constructor - only for subclasses, as this does not set keys.
   */
  MHNoiseModelFactor(const SharedNoiseModel& noiseModel) : noiseModel_arr_(1, noiseModel), creating_layer_(NULL) {}

public:

  void setCreatingHypoLayer(HypoLayer* hypo_layer) {
    creating_layer_ = hypo_layer;
  }

  /** Print */
  //[MH-A]:
  virtual void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
    
    std::cout << "MHNMF::print() NOT implemented yet" << std::endl;
  }

  /** Check if two factors are equal */
  //[MH-A]:
  virtual bool equals(const NonlinearFactor& f, double tol = 1e-9) const {
        
    std::cout << "MHNMF::equals() NOT implemented yet" << std::endl;
    return false; 
  }

  /** get the dimension of the factor (number of rows on linearization) */
  virtual size_t dim() const {
    if (noiseModel_arr_.size() != 0) {
      return noiseModel_arr_.front()->dim();
    } else {
      std::cout << "ERROR: NO NoiseModel exists when calling MHNoiseModelFactor::dim()" << std::endl;
      return -1;
    }
  }

  /// access to the noise model
  const SharedNoiseModel& noiseModel() const {
    std::cout << "WARNING: MHNoiseModelFactor::noiseModel() only returns front()..." << std::endl;
    return noiseModel_arr_.front();
  }

  /// @deprecated access to the noise model
  SharedNoiseModel get_noiseModel() const {
    std::cout << "WARNING: MHNoiseModelFactor::get_noiseModel() only returns front()..." << std::endl;
    return noiseModel_arr_.front();
  }

  //[MH-G]: Only called when each mode has its own NoiseModel
  SharedNoiseModel getNoiseModelAt(const size_t& mode_id) const {
    if (mode_id < noiseModel_arr_.size()) {
      return noiseModel_arr_[mode_id];
    } else {
      return noiseModel_arr_.front(); //mhsiao: should only be used by detached
    }
  }
  
  /**
   * Error function *without* the NoiseModel, \f$ z-h(x) \f$.
   * Override this method to finish implementing an N-way factor.
   * If the optional arguments is specified, it should compute
   * both the function evaluation and its derivative(s) in H.
   */
   // Depends on how linearize() uses this
  virtual std::vector<Vector> mhUnwhitenedError(const Values& x, boost::optional<std::vector< std::vector<Matrix> >&> H = boost::none, boost::optional<std::vector<SharedNoiseModel>&> corresp_NM_arr = boost::none) const = 0; //MHNoiseModelFactor

  virtual void setMaxKeyIdx(const int& idx) const {
    max_key_idx_ = idx;
  }

  /**
   * Vector of errors, whitened
   * This is the raw error, i.e., i.e. \f$ (h(x)-z)/\sigma \f$ in case of a Gaussian
   */
  //Vector whitenedError(const Values& c) const;

  /**
   * Calculate the error of the factor.
   * This is the log-likelihood, e.g. \f$ 0.5(h(x)-z)^2/\sigma^2 \f$ in case of Gaussian.
   * In this class, we take the raw prediction error \f$ h(x)-z \f$, ask the noise model
   * to transform it to \f$ (h(x)-z)^2/\sigma^2 \f$, and then multiply by 0.5.
   */
  
  //[MH-A]:
  virtual double error(const Values& c) const {
    std::cout << "MHNMF::error() NOT implemented yet" << std::endl;
    return 0.0;
  }

  /**
   * Linearize a non-linearFactorN to get a GaussianFactor,
   * \f$ Ax-b \approx h(x+\delta x)-z = h(x) + A \delta x - z \f$
   * Hence \f$ b = z - h(x) = - \mathtt{error\_vector}(x) \f$
   */
  
  boost::shared_ptr<GaussianFactor> linearize(const Values& x) const { //MHNoiseModelFactor
    std::cout << "MHNoiseModelFactor::linearize() return NULL" << std::endl;
    return NULL;
  }
   //[MH-A]: The 2nd reason why we need a MHNoiseModelFactor. Loop through all hypos/modes
  boost::shared_ptr<GaussianFactor> mhLinearize(const Values& x) const; //MHNoiseModelFactor

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("MHNonlinearFactor",
        boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(noiseModel_arr_);
  }

}; // \class MHNoiseModelFactor

//======================================== END MHNoiseModelFactor ===============================================

//======================================== MHNoiseModelFactor1 ===============================================
template<class VALUE>
class MHNoiseModelFactor1: public MHNoiseModelFactor {

public:
  
  typedef std::list<HypoNode*> HypoList; //for virtual getHypoList()
  typedef typename HypoList::iterator HypoListIter;
  typedef typename HypoList::const_iterator HypoListCstIter;

  // typedefs for value types pulled from keys
  typedef VALUE X;

  typedef GenericValue<X> GenericX;
 
  typedef typename boost::shared_ptr<GenericX> sharedGenericX;

protected:

  typedef MHNoiseModelFactor Base;
  typedef MHNoiseModelFactor1<VALUE> This;

public:

  /** Default constructor for I/O only */
  MHNoiseModelFactor1() {}

  virtual ~MHNoiseModelFactor1() {}

  inline Key key() const { return keys_[0]; }

  /**
   *  Constructor
   *  @param noiseModel shared pointer to noise model
   *  @param key1 by which to look up X value in Values
   */
  MHNoiseModelFactor1(const SharedNoiseModel& noiseModel, Key key1) :
    Base(noiseModel, cref_list_of<1>(key1)) {}

  /** Calls the 1-key specific version of evaluateError, which is pure virtual
   *  so must be implemented in the derived class.
   */
  //[MH-A]:
  virtual std::vector<Vector> mhUnwhitenedError(const Values& x, boost::optional<std::vector< std::vector<Matrix> >&> H = boost::none, boost::optional<std::vector<SharedNoiseModel>&> corresp_NM_arr = boost::none) const { //MHNoiseModelFactor1 (for MHPriorFactor)
    
    const HypoList& hypo_list = x.at(keys()[max_key_idx_]).getHypoList(); //make sure the max_key_idx_ is updated before calling this function...
    std::vector<Vector> out_vec_arr(H->size());
    
    //[MH-G]:
    const bool is_set_NM = (noiseModel_arr_.size() != 1);

    if(this->active(x)) {
      int hypo_count = 0;
      for (HypoListCstIter hit = hypo_list.begin(); hit != hypo_list.end(); ++hit) { //use hypo_list_ of variable to infer all others...
        
        size_t mode_id = 0;
        if (creating_layer_ != NULL) {

          int fac_layer = creating_layer_->getLayerIdx();

          mode_id = ((*hit)->findAncestor(fac_layer))->mode_id_;
        }
        
        X& x1 = boost::static_pointer_cast<GenericX>((*hit)->key_value_map_.find(keys_[0])->second)->value(); 
          
        Vector error;
        
        if(H) {
          error = evaluateSingleError(x1, mode_id, (*H)[hypo_count][0]);
        } else {
          error = evaluateSingleError(x1, mode_id);
        }

        out_vec_arr[hypo_count] = error;

        //[MH-G]:
        if (is_set_NM) {
          (*corresp_NM_arr)[hypo_count] = getNoiseModelAt(mode_id);
        }

        hypo_count++;

      } //END for
      return out_vec_arr;
   
    } else { 
      //TODO: Do NOT consider hard constraints here yet
      for (size_t i = 0 ; i < out_vec_arr.size(); ++i) {
        out_vec_arr[i] = Vector::Zero(this->dim()); 
      }
      return out_vec_arr;
    }
  }

  /**
   *  Override this method to finish implementing a unary factor.
   *  If the optional Matrix reference argument is specified, it should compute
   *  both the function evaluation and its derivative in X.
   */
  //[MH-A]:
  virtual Vector
  evaluateSingleError(const X& x, const size_t& mode_id, boost::optional<Matrix&> H = boost::none) const = 0;

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("MHNoiseModelFactor",
        boost::serialization::base_object<Base>(*this));
  }
}; // MHNoiseModelFactor1
//======================================== END MHNoiseModelFactor1 ===============================================

//======================================== MHNoiseModelFactor2 ===============================================
//[MH-A]: MH-Factor between two same type of variables and share the same NoiseModel
//TODO: Should NOT share the same NosieModel
template<class VALUE1, class VALUE2>
class MHNoiseModelFactor2: public MHNoiseModelFactor {

public:

  typedef std::list<HypoNode*> HypoList; //for virtual getHypoList()
  typedef typename HypoList::iterator HypoListIter;
  typedef typename HypoList::const_iterator HypoListCstIter;

  // typedefs for value types pulled from keys
  typedef VALUE1 X1;
  typedef VALUE2 X2;
 
  typedef GenericValue<X1> GenericX1;
  typedef GenericValue<X2> GenericX2;
  
  typedef typename boost::shared_ptr<GenericX1> sharedGenericX1;
  typedef typename boost::shared_ptr<GenericX2> sharedGenericX2;

protected:

  typedef MHNoiseModelFactor Base;
  typedef MHNoiseModelFactor2<VALUE1, VALUE2> This;

public:

  /**
   * Default Constructor for I/O
   */
  MHNoiseModelFactor2() {}

  /**
   * Constructor
   * @param noiseModel shared pointer to noise model
   * @param j1 key of the first variable
   * @param j2 key of the second variable
   */
  MHNoiseModelFactor2(const SharedNoiseModel& noiseModel, Key j1, Key j2) :
    Base(noiseModel, cref_list_of<2>(j1)(j2)) {}
  
  //[MH-G]: each mode has its own NoiseModel
  MHNoiseModelFactor2(const std::vector<SharedNoiseModel>& noiseModel_arr, Key j1, Key j2) :
    Base(noiseModel_arr, cref_list_of<2>(j1)(j2)) {}

  virtual ~MHNoiseModelFactor2() {}

  /** methods to retrieve both keys */
  inline Key key1() const { return keys_[0];  }
  inline Key key2() const {  return keys_[1];  }

  /** Calls the 2-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  //[MH-A]: Associate modes and hypos of factors and values through HypoTree
  virtual std::vector<Vector> mhUnwhitenedError(const Values& x, boost::optional<std::vector< std::vector<Matrix> >&> H = boost::none, boost::optional<std::vector<SharedNoiseModel>&> corresp_NM_arr = boost::none) const { //MHNoiseModelFactor2 (for MHBetweenFactor)

    const HypoList& hypo_list = x.at(keys()[max_key_idx_]).getHypoList(); //make sure the max_key_idx_ is updated before calling this function...
    std::vector<Vector> out_vec_arr(H->size());
    
    //[MH-G]: 
    const bool is_set_NM = (noiseModel_arr_.size() != 1);

    if(this->active(x)) {

      HypoTree* this_tree = hypo_list.front()->belong_layer_ptr_->getBelongTreePtr();

      int hypo_count = 0;
      if (max_key_idx_ == 0) { //keys_[0] has the largest #hypo

        for (HypoListCstIter hit = hypo_list.begin(); hit != hypo_list.end(); ++hit) { //use hypo_list_ of variable to infer all others...
          
          size_t mode_id = 0;
          if (creating_layer_ != NULL) {
            int fac_layer = creating_layer_->getLayerIdx();
         
            mode_id = ((*hit)->findAncestor(fac_layer))->mode_id_;;
          }
          
          X1& x1 = boost::dynamic_pointer_cast<GenericX1>((*hit)->key_value_map_.find(keys_[0])->second)->value();
          
          int val_layer = this_tree->key_layer_map_.find(keys_[1])->second->getLayerIdx();
          // Can be either the same or diffrerent HypoLayer
          X2& x2 = boost::dynamic_pointer_cast<GenericX2>(((*hit)->findAncestor(val_layer))->key_value_map_.find(keys_[1])->second)->value();
          
          Vector error;
          
          if(H) {
            error = evaluateSingleError(x1, x2, mode_id, (*H)[hypo_count][0], (*H)[hypo_count][1]);
          } else {
            error = evaluateSingleError(x1, x2, mode_id);
          }
          
          out_vec_arr[hypo_count] = error;
          
          //[MH-G]:
          if (is_set_NM) {
            (*corresp_NM_arr)[hypo_count] = getNoiseModelAt(mode_id);
          }
          
          hypo_count++;
        } //END for

      } else { //max_key_idx_ == 1 //keys_[1] has the largest #hypo

        for (HypoListCstIter hit = hypo_list.begin(); hit != hypo_list.end(); ++hit) {
          
          size_t mode_id = 0;
          if (creating_layer_ != NULL) {

            int fac_layer = creating_layer_->getLayerIdx();
         
            mode_id = ((*hit)->findAncestor(fac_layer))->mode_id_;;
          }
          
          int val_layer = this_tree->key_layer_map_.find(keys_[0])->second->getLayerIdx();
          // Can be the same or diffrerent HypoLayer
          X1& x1 = boost::dynamic_pointer_cast<GenericX1>(((*hit)->findAncestor(val_layer))->key_value_map_.find(keys_[0])->second)->value();
          X2& x2 = boost::dynamic_pointer_cast<GenericX2>((*hit)->key_value_map_.find(keys_[1])->second)->value();
          
          Vector error;
          
          if(H) {
            error = evaluateSingleError(x1, x2, mode_id, (*H)[hypo_count][0], (*H)[hypo_count][1]);

          } else {
            error = evaluateSingleError(x1, x2, mode_id);
          }

          out_vec_arr[hypo_count] = error;
          
          //[MH-G]:
          if (is_set_NM) {
            (*corresp_NM_arr)[hypo_count] = getNoiseModelAt(mode_id);
          }

          hypo_count++;
        } //END for
      
      }
      return out_vec_arr;
   
    } else { 
      //TODO: Do NOT consider hard constraints here yet
      for (size_t i = 0 ; i < out_vec_arr.size(); ++i) {
        out_vec_arr[i] = Vector::Zero(this->dim()); 
      }
      return out_vec_arr;
    }

  }

  /**
   *  Override this method to finish implementing a binary factor.
   *  If any of the optional Matrix reference arguments are specified, it should compute
   *  both the function evaluation and its derivative(s) in X1 (and/or X2).
   */
   //[MH-A]: Each evaluateSingleError() only work on one hypo/mode
  virtual Vector
  evaluateSingleError(const X1&, const X2&, const size_t& mode_id, boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const = 0;

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("MHNoiseModelFactor",
        boost::serialization::base_object<Base>(*this));
  }
}; // \class MHNoiseModelFactor2

//======================================== END MHNoiseModelFactor2 ===============================================

//======================================== MHNoiseModelFactor6 ===============================================
template<class VALUE1, class VALUE2, class VALUE3, class VALUE4, class VALUE5, class VALUE6>
class MHNoiseModelFactor6: public MHNoiseModelFactor {

public:

  typedef std::list<HypoNode*> HypoList; //for virtual getHypoList()
  typedef typename HypoList::iterator HypoListIter;
  typedef typename HypoList::const_iterator HypoListCstIter;

  // typedefs for value types pulled from keys
  typedef VALUE1 X1;
  typedef VALUE2 X2;
  typedef VALUE3 X3;
  typedef VALUE4 X4;
  typedef VALUE5 X5;
  typedef VALUE6 X6;
 
  typedef GenericValue<X1> GenericX1;
  typedef GenericValue<X2> GenericX2;
  typedef GenericValue<X3> GenericX3;
  typedef GenericValue<X4> GenericX4;
  typedef GenericValue<X5> GenericX5;
  typedef GenericValue<X6> GenericX6;
  
  typedef typename boost::shared_ptr<GenericX1> sharedGenericX1;
  typedef typename boost::shared_ptr<GenericX2> sharedGenericX2;
  typedef typename boost::shared_ptr<GenericX3> sharedGenericX3;
  typedef typename boost::shared_ptr<GenericX4> sharedGenericX4;
  typedef typename boost::shared_ptr<GenericX5> sharedGenericX5;
  typedef typename boost::shared_ptr<GenericX6> sharedGenericX6;

protected:

  typedef MHNoiseModelFactor Base;
  typedef MHNoiseModelFactor6<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5, VALUE6> This;

public:

  /**
   * Default Constructor for I/O
   */
  MHNoiseModelFactor6() {}

  /**
   * Constructor
   * @param noiseModel shared pointer to noise model
   * @param j1 key of the first variable
   * @param j2 key of the second variable
   */
  MHNoiseModelFactor6(const SharedNoiseModel& noiseModel, Key j1, Key j2, Key j3, Key j4, Key j5, Key j6) :
    Base(noiseModel, cref_list_of<6>(j1)(j2)(j3)(j4)(j5)(j6)) {}

  virtual ~MHNoiseModelFactor6() {}

  /** methods to retrieve both keys */
  inline Key key1() const { return keys_[0];  }
  inline Key key2() const { return keys_[1];  }
  inline Key key3() const { return keys_[2];  }
  inline Key key4() const { return keys_[3];  }
  inline Key key5() const { return keys_[4];  }
  inline Key key6() const { return keys_[5];  }

  /** Calls the 6-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  //[MH-A]: Associate modes and hypos of factors and values through HypoTree
  virtual std::vector<Vector> mhUnwhitenedError(const Values& x, boost::optional<std::vector< std::vector<Matrix> >&> H = boost::none, boost::optional<std::vector<SharedNoiseModel>&> corresp_NM_arr = boost::none) const { //MHNoiseModelFactor6 (for MHCombinedImuFactor)

    const HypoList& hypo_list = x.at(keys()[max_key_idx_]).getHypoList(); //make sure the max_key_idx_ is updated before calling this function...
    std::vector<Vector> out_vec_arr(H->size());
    
    //[MH-G]: 
    const bool is_set_NM = (noiseModel_arr_.size() != 1);

    if(this->active(x)) {

      HypoTree* this_tree = hypo_list.front()->belong_layer_ptr_->getBelongTreePtr();

      int hypo_count = 0;
      if (max_key_idx_ == 0) { //keys_[0] has the largest #hypo

        for (HypoListCstIter hit = hypo_list.begin(); hit != hypo_list.end(); ++hit) { //use hypo_list_ of variable to infer all others...
          
          size_t mode_id = 0;
          if (creating_layer_ != NULL) {
            int fac_layer = creating_layer_->getLayerIdx();
         
            mode_id = ((*hit)->findAncestor(fac_layer))->mode_id_;;
          }
          
          X1& x1 = boost::dynamic_pointer_cast<GenericX1>((*hit)->key_value_map_.find(keys_[0])->second)->value();
          
          //int val_layer0 = this_tree->key_layer_map_.find(keys_[0])->second->getLayerIdx();
          //X1& x1 = boost::dynamic_pointer_cast<GenericX1>(((*hit)->findAncestor(val_layer0))->key_value_map_.find(keys_[0])->second)->value();
          int val_layer1 = this_tree->key_layer_map_.find(keys_[1])->second->getLayerIdx();
          X2& x2 = boost::dynamic_pointer_cast<GenericX2>(((*hit)->findAncestor(val_layer1))->key_value_map_.find(keys_[1])->second)->value();
          int val_layer2 = this_tree->key_layer_map_.find(keys_[2])->second->getLayerIdx();
          X3& x3 = boost::dynamic_pointer_cast<GenericX3>(((*hit)->findAncestor(val_layer2))->key_value_map_.find(keys_[2])->second)->value();
          int val_layer3 = this_tree->key_layer_map_.find(keys_[3])->second->getLayerIdx();
          X4& x4 = boost::dynamic_pointer_cast<GenericX4>(((*hit)->findAncestor(val_layer3))->key_value_map_.find(keys_[3])->second)->value();
          int val_layer4 = this_tree->key_layer_map_.find(keys_[4])->second->getLayerIdx();
          X5& x5 = boost::dynamic_pointer_cast<GenericX5>(((*hit)->findAncestor(val_layer4))->key_value_map_.find(keys_[4])->second)->value();
          int val_layer5 = this_tree->key_layer_map_.find(keys_[5])->second->getLayerIdx();
          X6& x6 = boost::dynamic_pointer_cast<GenericX6>(((*hit)->findAncestor(val_layer5))->key_value_map_.find(keys_[5])->second)->value();
          
          Vector error;
          if(H) {
            error = evaluateSingleError(x1, x2, x3, x4, x5, x6, mode_id, (*H)[hypo_count][0], (*H)[hypo_count][1], (*H)[hypo_count][2], (*H)[hypo_count][3], (*H)[hypo_count][4], (*H)[hypo_count][5]);
          } else {
            error = evaluateSingleError(x1, x2, x3, x4, x5, x6, mode_id);
          }
          out_vec_arr[hypo_count] = error;
          
          //[MH-G]:
          if (is_set_NM) {
            (*corresp_NM_arr)[hypo_count] = getNoiseModelAt(mode_id);
          }
          
          hypo_count++;
        } //END for
      
      } else if (max_key_idx_ == 1) { //keys_[1] has the largest #hypo

        for (HypoListCstIter hit = hypo_list.begin(); hit != hypo_list.end(); ++hit) { //use hypo_list_ of variable to infer all others...
          
          size_t mode_id = 0;
          if (creating_layer_ != NULL) {
            int fac_layer = creating_layer_->getLayerIdx();
         
            mode_id = ((*hit)->findAncestor(fac_layer))->mode_id_;;
          }
          
          X2& x2 = boost::dynamic_pointer_cast<GenericX2>((*hit)->key_value_map_.find(keys_[1])->second)->value();
          
          int val_layer0 = this_tree->key_layer_map_.find(keys_[0])->second->getLayerIdx();
          X1& x1 = boost::dynamic_pointer_cast<GenericX1>(((*hit)->findAncestor(val_layer0))->key_value_map_.find(keys_[0])->second)->value();
          //int val_layer1 = this_tree->key_layer_map_.find(keys_[1])->second->getLayerIdx();
          //X2& x2 = boost::dynamic_pointer_cast<GenericX2>(((*hit)->findAncestor(val_layer1))->key_value_map_.find(keys_[1])->second)->value();
          int val_layer2 = this_tree->key_layer_map_.find(keys_[2])->second->getLayerIdx();
          X3& x3 = boost::dynamic_pointer_cast<GenericX3>(((*hit)->findAncestor(val_layer2))->key_value_map_.find(keys_[2])->second)->value();
          int val_layer3 = this_tree->key_layer_map_.find(keys_[3])->second->getLayerIdx();
          X4& x4 = boost::dynamic_pointer_cast<GenericX4>(((*hit)->findAncestor(val_layer3))->key_value_map_.find(keys_[3])->second)->value();
          int val_layer4 = this_tree->key_layer_map_.find(keys_[4])->second->getLayerIdx();
          X5& x5 = boost::dynamic_pointer_cast<GenericX5>(((*hit)->findAncestor(val_layer4))->key_value_map_.find(keys_[4])->second)->value();
          int val_layer5 = this_tree->key_layer_map_.find(keys_[5])->second->getLayerIdx();
          X6& x6 = boost::dynamic_pointer_cast<GenericX6>(((*hit)->findAncestor(val_layer5))->key_value_map_.find(keys_[5])->second)->value();
          
          Vector error;
          if(H) {
            error = evaluateSingleError(x1, x2, x3, x4, x5, x6, mode_id, (*H)[hypo_count][0], (*H)[hypo_count][1], (*H)[hypo_count][2], (*H)[hypo_count][3], (*H)[hypo_count][4], (*H)[hypo_count][5]);
          } else {
            error = evaluateSingleError(x1, x2, x3, x4, x5, x6, mode_id);
          }
          out_vec_arr[hypo_count] = error;
          
          //[MH-G]:
          if (is_set_NM) {
            (*corresp_NM_arr)[hypo_count] = getNoiseModelAt(mode_id);
          }
          
          hypo_count++;
        } //END for

      } else if (max_key_idx_ == 2) { //keys_[2] has the largest #hypo

        for (HypoListCstIter hit = hypo_list.begin(); hit != hypo_list.end(); ++hit) { //use hypo_list_ of variable to infer all others...
          
          size_t mode_id = 0;
          if (creating_layer_ != NULL) {
            int fac_layer = creating_layer_->getLayerIdx();
         
            mode_id = ((*hit)->findAncestor(fac_layer))->mode_id_;;
          }
          
          X3& x3 = boost::dynamic_pointer_cast<GenericX3>((*hit)->key_value_map_.find(keys_[2])->second)->value();
          
          int val_layer0 = this_tree->key_layer_map_.find(keys_[0])->second->getLayerIdx();
          X1& x1 = boost::dynamic_pointer_cast<GenericX1>(((*hit)->findAncestor(val_layer0))->key_value_map_.find(keys_[0])->second)->value();
          int val_layer1 = this_tree->key_layer_map_.find(keys_[1])->second->getLayerIdx();
          X2& x2 = boost::dynamic_pointer_cast<GenericX2>(((*hit)->findAncestor(val_layer1))->key_value_map_.find(keys_[1])->second)->value();
          //int val_layer2 = this_tree->key_layer_map_.find(keys_[2])->second->getLayerIdx();
          //X3& x3 = boost::dynamic_pointer_cast<GenericX3>(((*hit)->findAncestor(val_layer2))->key_value_map_.find(keys_[2])->second)->value();
          int val_layer3 = this_tree->key_layer_map_.find(keys_[3])->second->getLayerIdx();
          X4& x4 = boost::dynamic_pointer_cast<GenericX4>(((*hit)->findAncestor(val_layer3))->key_value_map_.find(keys_[3])->second)->value();
          int val_layer4 = this_tree->key_layer_map_.find(keys_[4])->second->getLayerIdx();
          X5& x5 = boost::dynamic_pointer_cast<GenericX5>(((*hit)->findAncestor(val_layer4))->key_value_map_.find(keys_[4])->second)->value();
          int val_layer5 = this_tree->key_layer_map_.find(keys_[5])->second->getLayerIdx();
          X6& x6 = boost::dynamic_pointer_cast<GenericX6>(((*hit)->findAncestor(val_layer5))->key_value_map_.find(keys_[5])->second)->value();
          
          Vector error;
          if(H) {
            error = evaluateSingleError(x1, x2, x3, x4, x5, x6, mode_id, (*H)[hypo_count][0], (*H)[hypo_count][1], (*H)[hypo_count][2], (*H)[hypo_count][3], (*H)[hypo_count][4], (*H)[hypo_count][5]);
          } else {
            error = evaluateSingleError(x1, x2, x3, x4, x5, x6, mode_id);
          }
          out_vec_arr[hypo_count] = error;
          
          //[MH-G]:
          if (is_set_NM) {
            (*corresp_NM_arr)[hypo_count] = getNoiseModelAt(mode_id);
          }
          
          hypo_count++;
        } //END for

      } else if (max_key_idx_ == 3) { //keys_[1] has the largest #hypo

        for (HypoListCstIter hit = hypo_list.begin(); hit != hypo_list.end(); ++hit) { //use hypo_list_ of variable to infer all others...
          
          size_t mode_id = 0;
          if (creating_layer_ != NULL) {
            int fac_layer = creating_layer_->getLayerIdx();
         
            mode_id = ((*hit)->findAncestor(fac_layer))->mode_id_;;
          }
          
          X4& x4 = boost::dynamic_pointer_cast<GenericX4>((*hit)->key_value_map_.find(keys_[3])->second)->value();
          
          int val_layer0 = this_tree->key_layer_map_.find(keys_[0])->second->getLayerIdx();
          X1& x1 = boost::dynamic_pointer_cast<GenericX1>(((*hit)->findAncestor(val_layer0))->key_value_map_.find(keys_[0])->second)->value();
          int val_layer1 = this_tree->key_layer_map_.find(keys_[1])->second->getLayerIdx();
          X2& x2 = boost::dynamic_pointer_cast<GenericX2>(((*hit)->findAncestor(val_layer1))->key_value_map_.find(keys_[1])->second)->value();
          int val_layer2 = this_tree->key_layer_map_.find(keys_[2])->second->getLayerIdx();
          X3& x3 = boost::dynamic_pointer_cast<GenericX3>(((*hit)->findAncestor(val_layer2))->key_value_map_.find(keys_[2])->second)->value();
          //int val_layer3 = this_tree->key_layer_map_.find(keys_[3])->second->getLayerIdx();
          //X4& x4 = boost::dynamic_pointer_cast<GenericX4>(((*hit)->findAncestor(val_layer3))->key_value_map_.find(keys_[3])->second)->value();
          int val_layer4 = this_tree->key_layer_map_.find(keys_[4])->second->getLayerIdx();
          X5& x5 = boost::dynamic_pointer_cast<GenericX5>(((*hit)->findAncestor(val_layer4))->key_value_map_.find(keys_[4])->second)->value();
          int val_layer5 = this_tree->key_layer_map_.find(keys_[5])->second->getLayerIdx();
          X6& x6 = boost::dynamic_pointer_cast<GenericX6>(((*hit)->findAncestor(val_layer5))->key_value_map_.find(keys_[5])->second)->value();
          
          Vector error;
          if(H) {
            error = evaluateSingleError(x1, x2, x3, x4, x5, x6, mode_id, (*H)[hypo_count][0], (*H)[hypo_count][1], (*H)[hypo_count][2], (*H)[hypo_count][3], (*H)[hypo_count][4], (*H)[hypo_count][5]);
          } else {
            error = evaluateSingleError(x1, x2, x3, x4, x5, x6, mode_id);
          }
          out_vec_arr[hypo_count] = error;
          
          //[MH-G]:
          if (is_set_NM) {
            (*corresp_NM_arr)[hypo_count] = getNoiseModelAt(mode_id);
          }
          
          hypo_count++;
        } //END for

      } else if (max_key_idx_ == 4) { //keys_[1] has the largest #hypo

        for (HypoListCstIter hit = hypo_list.begin(); hit != hypo_list.end(); ++hit) { //use hypo_list_ of variable to infer all others...
          
          size_t mode_id = 0;
          if (creating_layer_ != NULL) {
            int fac_layer = creating_layer_->getLayerIdx();
         
            mode_id = ((*hit)->findAncestor(fac_layer))->mode_id_;;
          }
          
          X5& x5 = boost::dynamic_pointer_cast<GenericX5>((*hit)->key_value_map_.find(keys_[4])->second)->value();
          
          int val_layer0 = this_tree->key_layer_map_.find(keys_[0])->second->getLayerIdx();
          X1& x1 = boost::dynamic_pointer_cast<GenericX1>(((*hit)->findAncestor(val_layer0))->key_value_map_.find(keys_[0])->second)->value();
          int val_layer1 = this_tree->key_layer_map_.find(keys_[1])->second->getLayerIdx();
          X2& x2 = boost::dynamic_pointer_cast<GenericX2>(((*hit)->findAncestor(val_layer1))->key_value_map_.find(keys_[1])->second)->value();
          int val_layer2 = this_tree->key_layer_map_.find(keys_[2])->second->getLayerIdx();
          X3& x3 = boost::dynamic_pointer_cast<GenericX3>(((*hit)->findAncestor(val_layer2))->key_value_map_.find(keys_[2])->second)->value();
          int val_layer3 = this_tree->key_layer_map_.find(keys_[3])->second->getLayerIdx();
          X4& x4 = boost::dynamic_pointer_cast<GenericX4>(((*hit)->findAncestor(val_layer3))->key_value_map_.find(keys_[3])->second)->value();
          //int val_layer4 = this_tree->key_layer_map_.find(keys_[4])->second->getLayerIdx();
          //X5& x5 = boost::dynamic_pointer_cast<GenericX5>(((*hit)->findAncestor(val_layer4))->key_value_map_.find(keys_[4])->second)->value();
          int val_layer5 = this_tree->key_layer_map_.find(keys_[5])->second->getLayerIdx();
          X6& x6 = boost::dynamic_pointer_cast<GenericX6>(((*hit)->findAncestor(val_layer5))->key_value_map_.find(keys_[5])->second)->value();
          
          Vector error;
          if(H) {
            error = evaluateSingleError(x1, x2, x3, x4, x5, x6, mode_id, (*H)[hypo_count][0], (*H)[hypo_count][1], (*H)[hypo_count][2], (*H)[hypo_count][3], (*H)[hypo_count][4], (*H)[hypo_count][5]);
          } else {
            error = evaluateSingleError(x1, x2, x3, x4, x5, x6, mode_id);
          }
          out_vec_arr[hypo_count] = error;
          
          //[MH-G]:
          if (is_set_NM) {
            (*corresp_NM_arr)[hypo_count] = getNoiseModelAt(mode_id);
          }
          
          hypo_count++;
        } //END for

      } else { //max_key_idx_ == 5 //keys_[5] has the largest #hypo

        for (HypoListCstIter hit = hypo_list.begin(); hit != hypo_list.end(); ++hit) {
          
          size_t mode_id = 0;
          if (creating_layer_ != NULL) {

            int fac_layer = creating_layer_->getLayerIdx();
         
            mode_id = ((*hit)->findAncestor(fac_layer))->mode_id_;;
          }
          
          X6& x6 = boost::dynamic_pointer_cast<GenericX6>((*hit)->key_value_map_.find(keys_[5])->second)->value();
          
          int val_layer0 = this_tree->key_layer_map_.find(keys_[0])->second->getLayerIdx();
          X1& x1 = boost::dynamic_pointer_cast<GenericX1>(((*hit)->findAncestor(val_layer0))->key_value_map_.find(keys_[0])->second)->value();
          int val_layer1 = this_tree->key_layer_map_.find(keys_[1])->second->getLayerIdx();
          X2& x2 = boost::dynamic_pointer_cast<GenericX2>(((*hit)->findAncestor(val_layer1))->key_value_map_.find(keys_[1])->second)->value();
          int val_layer2 = this_tree->key_layer_map_.find(keys_[2])->second->getLayerIdx();
          X3& x3 = boost::dynamic_pointer_cast<GenericX3>(((*hit)->findAncestor(val_layer2))->key_value_map_.find(keys_[2])->second)->value();
          int val_layer3 = this_tree->key_layer_map_.find(keys_[3])->second->getLayerIdx();
          X4& x4 = boost::dynamic_pointer_cast<GenericX4>(((*hit)->findAncestor(val_layer3))->key_value_map_.find(keys_[3])->second)->value();
          int val_layer4 = this_tree->key_layer_map_.find(keys_[4])->second->getLayerIdx();
          X5& x5 = boost::dynamic_pointer_cast<GenericX5>(((*hit)->findAncestor(val_layer4))->key_value_map_.find(keys_[4])->second)->value();
          //int val_layer5 = this_tree->key_layer_map_.find(keys_[5])->second->getLayerIdx();
          //X6& x6 = boost::dynamic_pointer_cast<GenericX6>(((*hit)->findAncestor(val_layer5))->key_value_map_.find(keys_[5])->second)->value();
          
          Vector error;
          if(H) {
            error = evaluateSingleError(x1, x2, x3, x4, x5, x6, mode_id, (*H)[hypo_count][0], (*H)[hypo_count][1], (*H)[hypo_count][2], (*H)[hypo_count][3], (*H)[hypo_count][4], (*H)[hypo_count][5]);
          } else {
            error = evaluateSingleError(x1, x2, x3, x4, x5, x6, mode_id);
          }

          out_vec_arr[hypo_count] = error;
          
          //[MH-G]:
          if (is_set_NM) {
            (*corresp_NM_arr)[hypo_count] = getNoiseModelAt(mode_id);
          }

          hypo_count++;
        } //END for
      
      }
      return out_vec_arr;
   
    } else {
      //TODO: Do NOT consider hard constraints here yet
      for (size_t i = 0 ; i < out_vec_arr.size(); ++i) {
        out_vec_arr[i] = Vector::Zero(this->dim()); 
      }
      return out_vec_arr;
    }

  }

  /**
   *  Override this method to finish implementing a binary factor.
   *  If any of the optional Matrix reference arguments are specified, it should compute
   *  both the function evaluation and its derivative(s) in X1 (and/or X2).
   */
   //[MH-A]: Each evaluateSingleError() only work on one hypo/mode
  virtual Vector
  evaluateSingleError(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const size_t& mode_id, boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none, boost::optional<Matrix&> H3 = boost::none, boost::optional<Matrix&> H4 = boost::none, boost::optional<Matrix&> H5 = boost::none, boost::optional<Matrix&> H6 = boost::none) const = 0;

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("MHNoiseModelFactor",
        boost::serialization::base_object<Base>(*this));
  }
}; // \class MHNoiseModelFactor6
//======================================== END MHNoiseModelFactor6 ===============================================


//======================================== MHNoiseModelFactor_1toK ===============================================
//[MH-A]: MH-Factor between pose and landmarks and share the same NoiseModel
template<class VALUE1, class VALUE2> //<pose, landmarks>
class MHNoiseModelFactor_1toK: public MHNoiseModelFactor {

public:

  typedef std::list<HypoNode*> HypoList; //for virtual getHypoList()
  typedef typename HypoList::iterator HypoListIter;
  typedef typename HypoList::const_iterator HypoListCstIter;

  // typedefs for value types pulled from keys
  typedef VALUE1 X1;
  typedef VALUE2 X2;
 
  typedef GenericValue<X1> GenericX1;
  typedef GenericValue<X2> GenericX2;
  
  typedef typename boost::shared_ptr<GenericX1> sharedGenericX1;
  typedef typename boost::shared_ptr<GenericX2> sharedGenericX2;

  typedef std::list<Key> KeyList;

protected:

  typedef MHNoiseModelFactor Base;
  typedef MHNoiseModelFactor_1toK<VALUE1, VALUE2> This;

public:

  /**
   * Default Constructor for I/O
   */
  MHNoiseModelFactor_1toK() {}

  /**
   * Constructor
   * @param noiseModel shared pointer to noise model
   * @param j1 key of the first variable
   * @param j2 key of the second variable
   */

  // Notice: How to construct correctly????
  MHNoiseModelFactor_1toK(const SharedNoiseModel& noiseModel, KeyList& j1_j2_list) : Base(noiseModel, j1_j2_list) {
    // Do nothing here...
  }

  virtual ~MHNoiseModelFactor_1toK() {}

  /** methods to retrieve both keys */
  inline Key key1() const { return keys_[0];  }
  inline Key key2At(size_t idx) const {  return keys_[idx + 1];  } //careful about index

  /** Calls the 2-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  //[MH-A]: Associate modes and hypos of factors and values through HypoTree
  virtual std::vector<Vector> mhUnwhitenedError(const Values& x, boost::optional<std::vector< std::vector<Matrix> >&> H = boost::none, boost::optional<std::vector<SharedNoiseModel>&> corresp_NM_arr = boost::none) const { //MHNoiseModelFactor_1toK (for MHPose2_Point2_Factor)

    const HypoList& hypo_list = x.at(keys()[max_key_idx_]).getHypoList(); //make sure the max_key_idx_ is updated before calling this function...
    std::vector<Vector> out_vec_arr(H->size());
    
    //[MH-G]: 
    const bool is_set_NM = (noiseModel_arr_.size() != 1);

    if(this->active(x)) {

      HypoTree* this_tree = hypo_list.front()->belong_layer_ptr_->getBelongTreePtr();

      int hypo_count = 0;
      if (max_key_idx_ == 0) { //keys_[0] has the largest #hypo

        for (HypoListCstIter hit = hypo_list.begin(); hit != hypo_list.end(); ++hit) { //use hypo_list_ of variable to infer all others...
          
          size_t mode_id = 0;
          if (creating_layer_ != NULL) {
            int fac_layer = creating_layer_->getLayerIdx();
         
            mode_id = ((*hit)->findAncestor(fac_layer))->mode_id_;;
          }
          
          X1& x1 = boost::dynamic_pointer_cast<GenericX1>((*hit)->key_value_map_.find(keys_[0])->second)->value();
          std::vector<X2> x2_arr(size() - 1);
         
          for (size_t i = 0; i < (size() - 1); ++i) {
            
            int val_layer = this_tree->key_layer_map_.find(keys_[i + 1])->second->getLayerIdx();
            // Can be either the same or diffrerent HypoLayer
            x2_arr[i] = boost::dynamic_pointer_cast<GenericX2>(((*hit)->findAncestor(val_layer))->key_value_map_.find(keys_[i + 1])->second)->value();
          }
          
          Vector error;
          
          if(H) {
            error = evaluateSingleError(x1, x2_arr, mode_id, (*H)[hypo_count]);
          } else {
            error = evaluateSingleError(x1, x2_arr, mode_id);
          }
          
          out_vec_arr[hypo_count] = error;
          
          //[MH-G]:
          if (is_set_NM) {
            (*corresp_NM_arr)[hypo_count] = getNoiseModelAt(mode_id);
          }
          
          hypo_count++;
        } //END for

      } else { //max_key_idx_ != 0 //one of keys_[1~K] has the largest #hypo

        //TODO: Just a copy from if( max_key_idx_ == 0){...}, so should be made into a function that outputs x1 and x2 directly
        for (HypoListCstIter hit = hypo_list.begin(); hit != hypo_list.end(); ++hit) {
          
          size_t mode_id = 0;
          if (creating_layer_ != NULL) {

            int fac_layer = creating_layer_->getLayerIdx();
         
            mode_id = ((*hit)->findAncestor(fac_layer))->mode_id_;;
          }
         
          int val_layer_1 = this_tree->key_layer_map_.find(keys_[0])->second->getLayerIdx();
          // Can be the same or diffrerent HypoLayer
          X1& x1 = boost::dynamic_pointer_cast<GenericX1>(((*hit)->findAncestor(val_layer_1))->key_value_map_.find(keys_[0])->second)->value();
          std::vector<X2> x2_arr(size() - 1);
          
          for (size_t i = 0; i < (size() - 1); ++i) {
            
            int val_layer = this_tree->key_layer_map_.find(keys_[i + 1])->second->getLayerIdx();
            // Can be either the same or diffrerent HypoLayer
            x2_arr[i] = boost::dynamic_pointer_cast<GenericX2>(((*hit)->findAncestor(val_layer))->key_value_map_.find(keys_[i + 1])->second)->value();
          }
          
          Vector error;
          
          if(H) {
            error = evaluateSingleError(x1, x2_arr, mode_id, (*H)[hypo_count]);
          } else {
            error = evaluateSingleError(x1, x2_arr, mode_id);
          }

          out_vec_arr[hypo_count] = error;
          
          //[MH-G]:
          if (is_set_NM) {
            (*corresp_NM_arr)[hypo_count] = getNoiseModelAt(mode_id);
          }

          hypo_count++;
        } //END for
      
      }
      return out_vec_arr;
   
    } else { 
      //TODO: Do NOT consider hard constraints here yet
      for (size_t i = 0 ; i < out_vec_arr.size(); ++i) {
        out_vec_arr[i] = Vector::Zero(this->dim()); 
      }
      return out_vec_arr;
    }

  }

  /**
   *  Override this method to finish implementing a binary factor.
   *  If any of the optional Matrix reference arguments are specified, it should compute
   *  both the function evaluation and its derivative(s) in X1 (and/or X2).
   */
  //[MH-A]: each evaluateSingleError() only work on one hypo/mode
  virtual Vector
  evaluateSingleError(const X1&, const std::vector<X2>&, const size_t& mode_id, boost::optional<std::vector<Matrix>&> H_arr = boost::none) const = 0;

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("MHNoiseModelFactor",
        boost::serialization::base_object<Base>(*this));
  }
}; // \class MHNoiseModelFactor_1toK

//======================================== END MHNoiseModelFactor_1toK ===============================================

//======================================== MHNoiseModelFactor_2toK ===============================================
//[MH-A]: MH-Factor between pose and landmarks and share the same NoiseModel
template<class VALUE1, class VALUE2, class VALUE3> //<pose, landmarks>
class MHNoiseModelFactor_2toK: public MHNoiseModelFactor {

public:

  typedef std::list<HypoNode*> HypoList; //for virtual getHypoList()
  typedef typename HypoList::iterator HypoListIter;
  typedef typename HypoList::const_iterator HypoListCstIter;

  // typedefs for value types pulled from keys
  typedef VALUE1 X1;
  typedef VALUE2 X2;
  typedef VALUE3 X3;
 
  typedef GenericValue<X1> GenericX1;
  typedef GenericValue<X2> GenericX2;
  typedef GenericValue<X3> GenericX3;
  
  typedef typename boost::shared_ptr<GenericX1> sharedGenericX1;
  typedef typename boost::shared_ptr<GenericX2> sharedGenericX2;
  typedef typename boost::shared_ptr<GenericX3> sharedGenericX3;

  typedef std::list<Key> KeyList;

protected:

  typedef MHNoiseModelFactor Base;
  typedef MHNoiseModelFactor_2toK<VALUE1, VALUE2, VALUE3> This;

public:

  /**
   * Default Constructor for I/O
   */
  MHNoiseModelFactor_2toK() {}

  /**
   * Constructor
   * @param noiseModel shared pointer to noise model
   * @param j1 key of the first variable
   * @param j2 key of the second variable
   */

  // Notice: How to construct correctly????
  MHNoiseModelFactor_2toK(const SharedNoiseModel& noiseModel, KeyList& j1_j2_j3_list) : Base(noiseModel, j1_j2_j3_list) {}

  virtual ~MHNoiseModelFactor_2toK() {}

  /** methods to retrieve both keys */
  inline Key key1() const { return keys_[0];  }
  inline Key key2() const { return keys_[1];  }
  inline Key key3At(size_t idx) const {  return keys_[idx + 2];  } //careful about index

  /** Calls the 2-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  //[MH-A]: Associate modes and hypos of factors and values through HypoTree
  virtual std::vector<Vector> mhUnwhitenedError(const Values& x, boost::optional<std::vector< std::vector<Matrix> >&> H = boost::none, boost::optional<std::vector<SharedNoiseModel>&> corresp_NM_arr = boost::none) const { //MHNoiseModelFactor_2toK (for NONE)

    const HypoList& hypo_list = x.at(keys()[max_key_idx_]).getHypoList(); //make sure the max_key_idx_ is updated before calling this function...
    std::vector<Vector> out_vec_arr(H->size());
    
    //[MH-G]: 
    const bool is_set_NM = (noiseModel_arr_.size() != 1);

    if(this->active(x)) {

      HypoTree* this_tree = hypo_list.front()->belong_layer_ptr_->getBelongTreePtr();

      int hypo_count = 0;
      if (max_key_idx_ == 0) { //keys_[0] has the largest #hypo

        for (HypoListCstIter hit = hypo_list.begin(); hit != hypo_list.end(); ++hit) { //use hypo_list_ of variable to infer all others...
          
          size_t mode_id = 0;
          if (creating_layer_ != NULL) {
            int fac_layer = creating_layer_->getLayerIdx();
         
            mode_id = ((*hit)->findAncestor(fac_layer))->mode_id_;;
          }
          
          X1& x1 = boost::dynamic_pointer_cast<GenericX1>((*hit)->key_value_map_.find(keys_[0])->second)->value();
          
          int val_layer1 = this_tree->key_layer_map_.find(keys_[1])->second->getLayerIdx();
          X2& x2 = boost::dynamic_pointer_cast<GenericX2>(((*hit)->findAncestor(val_layer1))->key_value_map_.find(keys_[1])->second)->value();
          
          std::vector<X3> x3_arr(size() - 2);
         
          for (size_t i = 0; i < (size() - 2); ++i) {
            
            int val_layer2 = this_tree->key_layer_map_.find(keys_[i + 2])->second->getLayerIdx();
            // Can be either the same or diffrerent HypoLayer
            x3_arr[i] = boost::dynamic_pointer_cast<GenericX3>(((*hit)->findAncestor(val_layer2))->key_value_map_.find(keys_[i + 2])->second)->value();
          }
          
          Vector error;
          if(H) {
            error = evaluateSingleError(x1, x2, x3_arr, mode_id, (*H)[hypo_count]);
          } else {
            error = evaluateSingleError(x1, x2, x3_arr, mode_id);
          }
          out_vec_arr[hypo_count] = error;
          
          //[MH-G]:
          if (is_set_NM) {
            (*corresp_NM_arr)[hypo_count] = getNoiseModelAt(mode_id);
          }
          
          hypo_count++;
        } //END for
      
      } else if (max_key_idx_ == 1) { //keys_[1] has the largest #hypo

        for (HypoListCstIter hit = hypo_list.begin(); hit != hypo_list.end(); ++hit) { //use hypo_list_ of variable to infer all others...
          
          size_t mode_id = 0;
          if (creating_layer_ != NULL) {
            int fac_layer = creating_layer_->getLayerIdx();
         
            mode_id = ((*hit)->findAncestor(fac_layer))->mode_id_;;
          }
          
          int val_layer0 = this_tree->key_layer_map_.find(keys_[0])->second->getLayerIdx();
          X1& x1 = boost::dynamic_pointer_cast<GenericX1>(((*hit)->findAncestor(val_layer0))->key_value_map_.find(keys_[0])->second)->value();
          
          X2& x2 = boost::dynamic_pointer_cast<GenericX2>((*hit)->key_value_map_.find(keys_[1])->second)->value();
          
          std::vector<X3> x3_arr(size() - 2);
         
          for (size_t i = 0; i < (size() - 2); ++i) {
            
            int val_layer2 = this_tree->key_layer_map_.find(keys_[i + 2])->second->getLayerIdx();
            // Can be either the same or diffrerent HypoLayer
            x3_arr[i] = boost::dynamic_pointer_cast<GenericX3>(((*hit)->findAncestor(val_layer2))->key_value_map_.find(keys_[i + 2])->second)->value();
          }
          
          Vector error;
          if(H) {
            error = evaluateSingleError(x1, x2, x3_arr, mode_id, (*H)[hypo_count]);
          } else {
            error = evaluateSingleError(x1, x2, x3_arr, mode_id);
          }
          out_vec_arr[hypo_count] = error;
          
          //[MH-G]:
          if (is_set_NM) {
            (*corresp_NM_arr)[hypo_count] = getNoiseModelAt(mode_id);
          }
          
          hypo_count++;
        } //END for

      } else { //max_key_idx_ != {0, 1} //one of keys_[2~K] has the largest #hypo

        for (HypoListCstIter hit = hypo_list.begin(); hit != hypo_list.end(); ++hit) {
          
          size_t mode_id = 0;
          if (creating_layer_ != NULL) {

            int fac_layer = creating_layer_->getLayerIdx();
         
            mode_id = ((*hit)->findAncestor(fac_layer))->mode_id_;;
          }
         
          int val_layer0 = this_tree->key_layer_map_.find(keys_[0])->second->getLayerIdx();
          X1& x1 = boost::dynamic_pointer_cast<GenericX1>(((*hit)->findAncestor(val_layer0))->key_value_map_.find(keys_[0])->second)->value();
          
          int val_layer1 = this_tree->key_layer_map_.find(keys_[1])->second->getLayerIdx();
          X2& x2 = boost::dynamic_pointer_cast<GenericX2>(((*hit)->findAncestor(val_layer1))->key_value_map_.find(keys_[1])->second)->value();
          
          std::vector<X3> x3_arr(size() - 2);
          
          for (size_t i = 0; i < (size() - 2); ++i) {
            
            int val_layer2 = this_tree->key_layer_map_.find(keys_[i + 2])->second->getLayerIdx();
            // Can be either the same or diffrerent HypoLayer
            x3_arr[i] = boost::dynamic_pointer_cast<GenericX3>(((*hit)->findAncestor(val_layer2))->key_value_map_.find(keys_[i + 2])->second)->value();
          }
          
          Vector error;
          
          if(H) {
            error = evaluateSingleError(x1, x2, x3_arr, mode_id, (*H)[hypo_count]);
          } else {
            error = evaluateSingleError(x1, x2, x3_arr, mode_id);
          }

          out_vec_arr[hypo_count] = error;
          
          //[MH-G]:
          if (is_set_NM) {
            (*corresp_NM_arr)[hypo_count] = getNoiseModelAt(mode_id);
          }

          hypo_count++;
        } //END for
      
      }
      return out_vec_arr;
   
    } else { 
      //TODO: Do NOT consider hard constraints here yet
      for (size_t i = 0 ; i < out_vec_arr.size(); ++i) {
        out_vec_arr[i] = Vector::Zero(this->dim()); 
      }
      return out_vec_arr;
    }

  }

  /**
   *  Override this method to finish implementing a binary factor.
   *  If any of the optional Matrix reference arguments are specified, it should compute
   *  both the function evaluation and its derivative(s) in X1 (and/or X2).
   */
   //[MH-A]: Each evaluateSingleError() only work on one hypo/mode
  virtual Vector
  evaluateSingleError(const X1&, const X2&, const std::vector<X3>&, const size_t& mode_id, boost::optional<std::vector<Matrix>&> H_arr = boost::none) const = 0;

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("MHNoiseModelFactor",
        boost::serialization::base_object<Base>(*this));
  }
}; // \class MHNoiseModelFactor_2toK
//======================================== END MHNoiseModelFactor_2toK ===============================================

//======================================== MHNoiseModelFactor_1to2K ===============================================
//[MH-A]: MH-Factor between pose and landmarks and share the same NoiseModel
template<class VALUE1, class VALUE2, class VALUE3> //<pose, landmarks>
class MHNoiseModelFactor_1to2K: public MHNoiseModelFactor {

public:

  typedef std::list<HypoNode*> HypoList; //for virtual getHypoList()
  typedef typename HypoList::iterator HypoListIter;
  typedef typename HypoList::const_iterator HypoListCstIter;

  // typedefs for value types pulled from keys
  typedef VALUE1 X1;
  typedef VALUE2 X2;
  typedef VALUE3 X3;
 
  typedef GenericValue<X1> GenericX1;
  typedef GenericValue<X2> GenericX2;
  typedef GenericValue<X3> GenericX3;
  
  typedef typename boost::shared_ptr<GenericX1> sharedGenericX1;
  typedef typename boost::shared_ptr<GenericX2> sharedGenericX2;
  typedef typename boost::shared_ptr<GenericX3> sharedGenericX3;

  typedef std::list<Key> KeyList;

protected:

  typedef MHNoiseModelFactor Base;
  typedef MHNoiseModelFactor_1to2K<VALUE1, VALUE2, VALUE3> This;

public:

  /**
   * Default Constructor for I/O
   */
  MHNoiseModelFactor_1to2K() {}

  /**
   * Constructor
   * @param noiseModel shared pointer to noise model
   * @param j1 key of the first variable
   * @param j2 key of the second variable
   */

  // Notice: How to construct correctly????
  MHNoiseModelFactor_1to2K(const SharedNoiseModel& noiseModel, KeyList& j1_j2_j3_list) : Base(noiseModel, j1_j2_j3_list) {}

  virtual ~MHNoiseModelFactor_1to2K() {}

  /** methods to retrieve both keys */
  inline Key key1() const { return keys_[0]; }
  inline Key key2At(size_t idx) const { return keys_[idx*2 + 1]; } //careful about index
  inline Key key3At(size_t idx) const { return keys_[idx*2 + 2]; } //careful about index

  /** Calls the 2-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  //[MH-A]: Associate modes and hypos of factors and values through HypoTree
  virtual std::vector<Vector> mhUnwhitenedError(const Values& x, boost::optional<std::vector< std::vector<Matrix> >&> H = boost::none, boost::optional<std::vector<SharedNoiseModel>&> corresp_NM_arr = boost::none) const { //MHNoiseModelFactor_1to2K (for MH_Pose3_Base3_Plane3d_Factor)

    const HypoList& hypo_list = x.at(keys()[max_key_idx_]).getHypoList(); //make sure the max_key_idx_ is updated before calling this function...
    std::vector<Vector> out_vec_arr(H->size());
    
    //[MH-G]: 
    const bool is_set_NM = (noiseModel_arr_.size() != 1);

    if(this->active(x)) {

      HypoTree* this_tree = hypo_list.front()->belong_layer_ptr_->getBelongTreePtr();

      int hypo_count = 0;
      if (max_key_idx_ == 0) { //keys_[0] has the largest #hypo

        for (HypoListCstIter hit = hypo_list.begin(); hit != hypo_list.end(); ++hit) { //use hypo_list_ of variable to infer all others...
          
          size_t mode_id = 0;
          if (creating_layer_ != NULL) {
            int fac_layer = creating_layer_->getLayerIdx();
         
            mode_id = ((*hit)->findAncestor(fac_layer))->mode_id_;;
          }
          
          X1& x1 = boost::dynamic_pointer_cast<GenericX1>((*hit)->key_value_map_.find(keys_[0])->second)->value();
         
          const size_t K = (size() - 1)/2;
          
          std::vector<X2> x2_arr(K);
          for (size_t i = 0; i < K; ++i) {
            
            int val_layer1 = this_tree->key_layer_map_.find(keys_[i*2 + 1])->second->getLayerIdx();
            // Can be either the same or diffrerent HypoLayer
            x2_arr[i] = boost::dynamic_pointer_cast<GenericX2>(((*hit)->findAncestor(val_layer1))->key_value_map_.find(keys_[i*2 + 1])->second)->value();
          }
          
          std::vector<X3> x3_arr(K);
          for (size_t i = 0; i < K; ++i) {
            
            int val_layer2 = this_tree->key_layer_map_.find(keys_[i*2 + 2])->second->getLayerIdx();
            // Can be either the same or diffrerent HypoLayer
            x3_arr[i] = boost::dynamic_pointer_cast<GenericX3>(((*hit)->findAncestor(val_layer2))->key_value_map_.find(keys_[i*2 + 2])->second)->value();
          }
          
          Vector error;
          if(H) {
            error = evaluateSingleError(x1, x2_arr, x3_arr, mode_id, (*H)[hypo_count]);
          } else {
            error = evaluateSingleError(x1, x2_arr, x3_arr, mode_id);
          }
          out_vec_arr[hypo_count] = error;
          
          //[MH-G]:
          if (is_set_NM) {
            (*corresp_NM_arr)[hypo_count] = getNoiseModelAt(mode_id);
          }
          
          hypo_count++;
        } //END for
      
      } else if (max_key_idx_%2 == 1) { //one of keys_[k*2 + 1] has the largest #hypo

        for (HypoListCstIter hit = hypo_list.begin(); hit != hypo_list.end(); ++hit) { //use hypo_list_ of variable to infer all others...
          
          size_t mode_id = 0;
          if (creating_layer_ != NULL) {
            int fac_layer = creating_layer_->getLayerIdx();
         
            mode_id = ((*hit)->findAncestor(fac_layer))->mode_id_;;
          }
          
          int val_layer0 = this_tree->key_layer_map_.find(keys_[0])->second->getLayerIdx();
          X1& x1 = boost::dynamic_pointer_cast<GenericX1>(((*hit)->findAncestor(val_layer0))->key_value_map_.find(keys_[0])->second)->value();
          
          const size_t K = (size() - 1)/2;
          
          //X2& x2 = boost::dynamic_pointer_cast<GenericX2>((*hit)->key_value_map_.find(keys_[1])->second)->value();
          std::vector<X2> x2_arr(K);
          for (size_t i = 0; i < K; ++i) {
            
            int val_layer1 = this_tree->key_layer_map_.find(keys_[i*2 + 1])->second->getLayerIdx();
            // Can be either the same or diffrerent HypoLayer
            x2_arr[i] = boost::dynamic_pointer_cast<GenericX2>(((*hit)->findAncestor(val_layer1))->key_value_map_.find(keys_[i*2 + 1])->second)->value();
          }
          
          std::vector<X3> x3_arr(K);
          for (size_t i = 0; i < K; ++i) {
            
            int val_layer2 = this_tree->key_layer_map_.find(keys_[i*2 + 2])->second->getLayerIdx();
            // Can be either the same or diffrerent HypoLayer
            x3_arr[i] = boost::dynamic_pointer_cast<GenericX3>(((*hit)->findAncestor(val_layer2))->key_value_map_.find(keys_[i*2 + 2])->second)->value();
          }
          
          Vector error;
          if(H) {
            error = evaluateSingleError(x1, x2_arr, x3_arr, mode_id, (*H)[hypo_count]);
          } else {
            error = evaluateSingleError(x1, x2_arr, x3_arr, mode_id);
          }
          out_vec_arr[hypo_count] = error;
          
          //[MH-G]:
          if (is_set_NM) {
            (*corresp_NM_arr)[hypo_count] = getNoiseModelAt(mode_id);
          }
          
          hypo_count++;
        } //END for

      } else { //one of keys_[k*2 + 2] has the largest #hypo

        for (HypoListCstIter hit = hypo_list.begin(); hit != hypo_list.end(); ++hit) {
          
          size_t mode_id = 0;
          if (creating_layer_ != NULL) {

            int fac_layer = creating_layer_->getLayerIdx();
         
            mode_id = ((*hit)->findAncestor(fac_layer))->mode_id_;;
          }
         
          int val_layer0 = this_tree->key_layer_map_.find(keys_[0])->second->getLayerIdx();
          X1& x1 = boost::dynamic_pointer_cast<GenericX1>(((*hit)->findAncestor(val_layer0))->key_value_map_.find(keys_[0])->second)->value();
          
          const size_t K = (size() - 1)/2;
          
          //int val_layer1 = this_tree->key_layer_map_.find(keys_[1])->second->getLayerIdx();
          //X2& x2 = boost::dynamic_pointer_cast<GenericX2>(((*hit)->findAncestor(val_layer1))->key_value_map_.find(keys_[1])->second)->value();
          
          std::vector<X2> x2_arr(K);
          for (size_t i = 0; i < K; ++i) {
            
            int val_layer1 = this_tree->key_layer_map_.find(keys_[i*2 + 1])->second->getLayerIdx();
            // Can be either the same or diffrerent HypoLayer
            x2_arr[i] = boost::dynamic_pointer_cast<GenericX2>(((*hit)->findAncestor(val_layer1))->key_value_map_.find(keys_[i*2 + 1])->second)->value();
          }
          
          std::vector<X3> x3_arr(K);
          for (size_t i = 0; i < K; ++i) {
            
            int val_layer2 = this_tree->key_layer_map_.find(keys_[i*2 + 2])->second->getLayerIdx();
            // Can be either the same or diffrerent HypoLayer
            x3_arr[i] = boost::dynamic_pointer_cast<GenericX3>(((*hit)->findAncestor(val_layer2))->key_value_map_.find(keys_[i*2 + 2])->second)->value();
          }
          
          Vector error;
          if(H) {
            error = evaluateSingleError(x1, x2_arr, x3_arr, mode_id, (*H)[hypo_count]);
          } else {
            error = evaluateSingleError(x1, x2_arr, x3_arr, mode_id);
          }
          out_vec_arr[hypo_count] = error;
          
          //[MH-G]:
          if (is_set_NM) {
            (*corresp_NM_arr)[hypo_count] = getNoiseModelAt(mode_id);
          }
          
          hypo_count++;
        } //END for
      
      }
      return out_vec_arr;
   
    } else { 
      //TODO: Do NOT consider hard constraints here yet
      for (size_t i = 0 ; i < out_vec_arr.size(); ++i) {
        out_vec_arr[i] = Vector::Zero(this->dim()); 
      }
      return out_vec_arr;
    }

  }

  /**
   *  Override this method to finish implementing a binary factor.
   *  If any of the optional Matrix reference arguments are specified, it should compute
   *  both the function evaluation and its derivative(s) in X1 (and/or X2).
   */
   //[MH-A]: Each evaluateSingleError() only work on one hypo/mode
  virtual Vector
  evaluateSingleError(const X1&, const std::vector<X2>&, const std::vector<X3>&, const size_t& mode_id, boost::optional<std::vector<Matrix>&> H_arr = boost::none) const = 0;

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("MHNoiseModelFactor",
        boost::serialization::base_object<Base>(*this));
  }
}; // \class MHNoiseModelFactor_2toK

//======================================== END MHNoiseModelFactor_1to2K ===============================================

} // \namespace gtsam
