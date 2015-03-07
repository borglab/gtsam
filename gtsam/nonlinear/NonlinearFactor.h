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

#include <list>
#include <limits>

#include <boost/serialization/base_object.hpp>
#include <boost/function.hpp>

#include <gtsam/inference/Factor-inl.h>
#include <gtsam/inference/IndexFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/JacobianFactor.h>

#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Ordering.h>

/**
 * Macro to add a standard clone function to a derived factor
 * @deprecated: will go away shortly - just add the clone function directly
 */
#define ADD_CLONE_NONLINEAR_FACTOR(Derived) \
  virtual gtsam::NonlinearFactor::shared_ptr clone() const { \
  return boost::static_pointer_cast<gtsam::NonlinearFactor>( \
      gtsam::NonlinearFactor::shared_ptr(new Derived(*this))); }

namespace gtsam {

/* ************************************************************************* */
/**
 * Nonlinear factor base class
 *
 * Templated on a values structure type. The values structures are typically
 * more general than just vectors, e.g., Rot3 or Pose3,
 * which are objects in non-linear manifolds (Lie groups).
 * \nosubgrouping
 */
class NonlinearFactor: public Factor<Key> {

protected:

  // Some handy typedefs
  typedef Factor<Key> Base;
  typedef NonlinearFactor This;

public:

  typedef boost::shared_ptr<NonlinearFactor> shared_ptr;

	/// @name Standard Constructors
	/// @{

  /** Default constructor for I/O only */
  NonlinearFactor() {
  }

  /**
   * Constructor from a vector of the keys involved in this factor
   */
  NonlinearFactor(const std::vector<size_t>& keys) :
    Base(keys) {}

  /**
   * Constructor from iterators over the keys involved in this factor
   */
  template<class ITERATOR>
  NonlinearFactor(ITERATOR beginKeys, ITERATOR endKeys) :
    Base(beginKeys, endKeys) {}

  NonlinearFactor(Key key) : Base(key) {} ///< Convenience constructor for 1 key
  NonlinearFactor(Key key1, Key key2) : Base(key1, key2) {} ///< Convenience constructor for 2 keys
  NonlinearFactor(Key key1, Key key2, Key key3) : Base(key1, key2, key3) {} ///< Convenience constructor for 3 keys
  NonlinearFactor(Key key1, Key key2, Key key3, Key key4) : Base(key1, key2, key3, key4) {} ///< Convenience constructor for 4 keys
  NonlinearFactor(Key key1, Key key2, Key key3, Key key4, Key key5) : Base(key1, key2, key3, key4, key5) {} ///< Convenience constructor for 5 keys
  NonlinearFactor(Key key1, Key key2, Key key3, Key key4, Key key5, Key key6) : Base(key1, key2, key3, key4, key5, key6) {} ///< Convenience constructor for 6 keys

	/// @}
	/// @name Testable
	/// @{

  /** print */
  virtual void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
    std::cout << s << "  keys = { ";
    BOOST_FOREACH(Key key, this->keys()) { std::cout << keyFormatter(key) << " "; }
    std::cout << "}" << std::endl;
  }

  /** Check if two factors are equal */
  virtual bool equals(const NonlinearFactor& f, double tol = 1e-9) const {
    return Base::equals(f);
  }

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
	virtual bool active(const Values& c) const { return true; }

  /** linearize to a GaussianFactor */
  virtual boost::shared_ptr<GaussianFactor>
  linearize(const Values& c, const Ordering& ordering) const = 0;

  /**
   * Create a symbolic factor using the given ordering to determine the
   * variable indices.
   */
  virtual IndexFactor::shared_ptr symbolic(const Ordering& ordering) const {
    std::vector<Index> indices(this->size());
    for(size_t j=0; j<this->size(); ++j)
      indices[j] = ordering[this->keys()[j]];
    return IndexFactor::shared_ptr(new IndexFactor(indices));
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
   * Creates a shared_ptr clone of the factor with different keys using
   * a map from old->new keys
   */
  shared_ptr rekey(const std::map<Key,Key>& rekey_mapping) const {
  	shared_ptr new_factor = clone();
  	for (size_t i=0; i<new_factor->size(); ++i) {
  		Key& cur_key = new_factor->keys()[i];
  		std::map<Key,Key>::const_iterator mapping = rekey_mapping.find(cur_key);
  		if (mapping != rekey_mapping.end())
  			cur_key = mapping->second;
  	}
  	return new_factor;
  }

  /**
   * Clones a factor and fully replaces its keys
   * @param new_keys is the full replacement set of keys
   */
  shared_ptr rekey(const std::vector<Key>& new_keys) const {
  	assert(new_keys.size() == this->keys().size());
  	shared_ptr new_factor = clone();
  	new_factor->keys() = new_keys;
  	return new_factor;
  }


}; // \class NonlinearFactor

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
class NoiseModelFactor: public NonlinearFactor {

protected:

  // handy typedefs
  typedef NonlinearFactor Base;
  typedef NoiseModelFactor This;

  SharedNoiseModel noiseModel_; /** Noise model */

public:

  typedef boost::shared_ptr<NoiseModelFactor > shared_ptr;

  /** Default constructor for I/O only */
  NoiseModelFactor() {
  }

  /** Destructor */
  virtual ~NoiseModelFactor() {}

  /**
   * Constructor
   */
  template<class ITERATOR>
  NoiseModelFactor(const SharedNoiseModel& noiseModel, ITERATOR beginKeys, ITERATOR endKeys)
  : Base(beginKeys, endKeys), noiseModel_(noiseModel) {
  }

  NoiseModelFactor(const SharedNoiseModel& noiseModel, Key key) : Base(key), noiseModel_(noiseModel) {} ///< Convenience constructor for 1 key
  NoiseModelFactor(const SharedNoiseModel& noiseModel, Key key1, Key key2) : Base(key1, key2), noiseModel_(noiseModel) {} ///< Convenience constructor for 2 keys
  NoiseModelFactor(const SharedNoiseModel& noiseModel, Key key1, Key key2, Key key3) : Base(key1, key2, key3), noiseModel_(noiseModel) {} ///< Convenience constructor for 3 keys
  NoiseModelFactor(const SharedNoiseModel& noiseModel, Key key1, Key key2, Key key3, Key key4) : Base(key1, key2, key3, key4), noiseModel_(noiseModel) {} ///< Convenience constructor for 4 keys
  NoiseModelFactor(const SharedNoiseModel& noiseModel, Key key1, Key key2, Key key3, Key key4, Key key5) : Base(key1, key2, key3, key4, key5), noiseModel_(noiseModel) {} ///< Convenience constructor for 5 keys
  NoiseModelFactor(const SharedNoiseModel& noiseModel, Key key1, Key key2, Key key3, Key key4, Key key5, Key key6) : Base(key1, key2, key3, key4, key5, key6), noiseModel_(noiseModel) {} ///< Convenience constructor for 6 keys

protected:

  /**
   * Constructor - only for subclasses, as this does not set keys.
   */
  NoiseModelFactor(const SharedNoiseModel& noiseModel) : noiseModel_(noiseModel) {}

public:

  /** Print */
  virtual void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
    Base::print(s, keyFormatter);
    this->noiseModel_->print("  noise model: ");
  }

  /** Check if two factors are equal */
  virtual bool equals(const NonlinearFactor& f, double tol = 1e-9) const {
    const NoiseModelFactor* e = dynamic_cast<const NoiseModelFactor*>(&f);
    return e && Base::equals(f, tol) && noiseModel_->equals(*e->noiseModel_, tol);
  }

  /** get the dimension of the factor (number of rows on linearization) */
  virtual size_t dim() const {
    return noiseModel_->dim();
  }

  /** access to the noise model */
  SharedNoiseModel get_noiseModel() const {
    return noiseModel_;
  }

  /**
   * Error function *without* the NoiseModel, \f$ z-h(x) \f$.
   * Override this method to finish implementing an N-way factor.
   * If any of the optional Matrix reference arguments are specified, it should compute
   * both the function evaluation and its derivative(s) in X1 (and/or X2, X3...).
   */
  virtual Vector unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H = boost::none) const = 0;

  /**
   * Vector of errors, whitened
   * This is the raw error, i.e., i.e. \f$ (h(x)-z)/\sigma \f$ in case of a Gaussian
   */
  Vector whitenedError(const Values& c) const {
    return noiseModel_->whiten(unwhitenedError(c));
  }

  /**
   * Calculate the error of the factor.
   * This is the log-likelihood, e.g. \f$ 0.5(h(x)-z)^2/\sigma^2 \f$ in case of Gaussian.
   * In this class, we take the raw prediction error \f$ h(x)-z \f$, ask the noise model
   * to transform it to \f$ (h(x)-z)^2/\sigma^2 \f$, and then multiply by 0.5.
   */
  virtual double error(const Values& c) const {
  	if (this->active(c))
  		return 0.5 * noiseModel_->distance(unwhitenedError(c));
  	else
  		return 0.0;
  }

  /**
   * Linearize a non-linearFactorN to get a GaussianFactor,
   * \f$ Ax-b \approx h(x+\delta x)-z = h(x) + A \delta x - z \f$
   * Hence \f$ b = z - h(x) = - \mathtt{error\_vector}(x) \f$
   */
  boost::shared_ptr<GaussianFactor> linearize(const Values& x, const Ordering& ordering) const {
  	// Only linearize if the factor is active
		if (!this->active(x))
			return boost::shared_ptr<JacobianFactor>();

    // Create the set of terms - Jacobians for each index
    Vector b;
    // Call evaluate error to get Jacobians and b vector
    std::vector<Matrix> A(this->size());
    b = -unwhitenedError(x, A);

    this->noiseModel_->WhitenSystem(A,b);

    std::vector<std::pair<Index, Matrix> > terms(this->size());
    // Fill in terms
    for(size_t j=0; j<this->size(); ++j) {
      terms[j].first = ordering[this->keys()[j]];
      terms[j].second.swap(A[j]);
    }

    // TODO pass unwhitened + noise model to Gaussian factor
    noiseModel::Constrained::shared_ptr constrained =
        boost::shared_dynamic_cast<noiseModel::Constrained>(this->noiseModel_);
    if(constrained)
      return GaussianFactor::shared_ptr(
          new JacobianFactor(terms, b, constrained->unit()));
    else
      return GaussianFactor::shared_ptr(
          new JacobianFactor(terms, b, noiseModel::Unit::Create(b.size())));
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & boost::serialization::make_nvp("NonlinearFactor",
        boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(noiseModel_);
  }

}; // \class NoiseModelFactor


/* ************************************************************************* */
/** A convenient base class for creating your own NoiseModelFactor with 1
 * variable.  To derive from this class, implement evaluateError(). */
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
   *  @param key1 by which to look up X value in Values
   */
  NoiseModelFactor1(const SharedNoiseModel& noiseModel, Key key1) :
    Base(noiseModel, key1) {}

  /** Calls the 1-key specific version of evaluateError, which is pure virtual
   *  so must be implemented in the derived class.
   */
  virtual Vector unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H = boost::none) const {
    if(this->active(x)) {
      const X& x1 = x.at<X>(keys_[0]);
      if(H) {
        return evaluateError(x1, (*H)[0]);
      } else {
        return evaluateError(x1);
      }
    } else {
      return zero(this->dim());
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
  void serialize(ARCHIVE & ar, const unsigned int version) {
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
   * @param j1 key of the first variable
   * @param j2 key of the second variable
   */
  NoiseModelFactor2(const SharedNoiseModel& noiseModel, Key j1, Key j2) :
    Base(noiseModel, j1, j2) {}

  virtual ~NoiseModelFactor2() {}

  /** methods to retrieve both keys */
  inline Key key1() const { return keys_[0];	}
  inline Key key2() const {	return keys_[1];	}

  /** Calls the 2-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  virtual Vector unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H = boost::none) const {
    if(this->active(x)) {
      const X1& x1 = x.at<X1>(keys_[0]);
      const X2& x2 = x.at<X2>(keys_[1]);
      if(H) {
        return evaluateError(x1, x2, (*H)[0], (*H)[1]);
      } else {
        return evaluateError(x1, x2);
      }
    } else {
      return zero(this->dim());
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
  void serialize(ARCHIVE & ar, const unsigned int version) {
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
   * @param j1 key of the first variable
   * @param j2 key of the second variable
   * @param j3 key of the third variable
   */
  NoiseModelFactor3(const SharedNoiseModel& noiseModel, Key j1, Key j2, Key j3) :
    Base(noiseModel, j1, j2, j3) {}

  virtual ~NoiseModelFactor3() {}

  /** methods to retrieve keys */
  inline Key key1() const { return keys_[0]; }
  inline Key key2() const { return keys_[1]; }
  inline Key key3() const { return keys_[2]; }

  /** Calls the 3-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  virtual Vector unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H = boost::none) const {
    if(this->active(x)) {
      if(H)
        return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), (*H)[0], (*H)[1], (*H)[2]);
      else
        return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]));
    } else {
      return zero(this->dim());
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
  void serialize(ARCHIVE & ar, const unsigned int version) {
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
   * @param j1 key of the first variable
   * @param j2 key of the second variable
   * @param j3 key of the third variable
   * @param j4 key of the fourth variable
   */
  NoiseModelFactor4(const SharedNoiseModel& noiseModel, Key j1, Key j2, Key j3, Key j4) :
    Base(noiseModel, j1, j2, j3, j4) {}

  virtual ~NoiseModelFactor4() {}

  /** methods to retrieve keys */
  inline Key key1() const { return keys_[0]; }
  inline Key key2() const { return keys_[1]; }
  inline Key key3() const { return keys_[2]; }
  inline Key key4() const { return keys_[3]; }

  /** Calls the 4-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  virtual Vector unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H = boost::none) const {
  	if(this->active(x)) {
  		if(H)
  			return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]), (*H)[0], (*H)[1], (*H)[2], (*H)[3]);
  		else
  			return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]));
  	} else {
  		return zero(this->dim());
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
  void serialize(ARCHIVE & ar, const unsigned int version) {
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
   * @param j1 key of the first variable
   * @param j2 key of the second variable
   * @param j3 key of the third variable
   * @param j4 key of the fourth variable
   * @param j5 key of the fifth variable
   */
  NoiseModelFactor5(const SharedNoiseModel& noiseModel, Key j1, Key j2, Key j3, Key j4, Key j5) :
    Base(noiseModel, j1, j2, j3, j4, j5) {}

  virtual ~NoiseModelFactor5() {}

  /** methods to retrieve keys */
  inline Key key1() const { return keys_[0]; }
  inline Key key2() const { return keys_[1]; }
  inline Key key3() const { return keys_[2]; }
  inline Key key4() const { return keys_[3]; }
  inline Key key5() const { return keys_[4]; }

  /** Calls the 5-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  virtual Vector unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H = boost::none) const {
  	if(this->active(x)) {
      if(H)
        return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]), x.at<X5>(keys_[4]), (*H)[0], (*H)[1], (*H)[2], (*H)[3], (*H)[4]);
      else
        return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]), x.at<X5>(keys_[4]));
  	} else {
  		return zero(this->dim());
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
  void serialize(ARCHIVE & ar, const unsigned int version) {
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
   * @param j1 key of the first variable
   * @param j2 key of the second variable
   * @param j3 key of the third variable
   * @param j4 key of the fourth variable
   * @param j5 key of the fifth variable
   * @param j6 key of the fifth variable
   */
  NoiseModelFactor6(const SharedNoiseModel& noiseModel, Key j1, Key j2, Key j3, Key j4, Key j5, Key j6) :
    Base(noiseModel, j1, j2, j3, j4, j5, j6) {}

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
  virtual Vector unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H = boost::none) const {
  	if(this->active(x)) {
      if(H)
        return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]), x.at<X5>(keys_[4]), x.at<X6>(keys_[5]), (*H)[0], (*H)[1], (*H)[2], (*H)[3], (*H)[4], (*H)[5]);
      else
        return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]), x.at<X5>(keys_[4]), x.at<X6>(keys_[5]));
  	} else {
  		return zero(this->dim());
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
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & boost::serialization::make_nvp("NoiseModelFactor",
        boost::serialization::base_object<Base>(*this));
  }
}; // \class NoiseModelFactor6

/* ************************************************************************* */

} // \namespace gtsam
