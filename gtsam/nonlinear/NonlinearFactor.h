/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    NoiseModelFactor.h
 * @brief   Non-linear factor class
 * @author  Frank Dellaert
 * @author  Richard Roberts
 */

// \callgraph

#pragma once

#include <list>
#include <limits>

#include <boost/serialization/base_object.hpp>
#include <boost/tuple/tuple.hpp>

#include <gtsam/inference/Factor-inl.h>
#include <gtsam/inference/IndexFactor.h>
#include <gtsam/linear/SharedNoiseModel.h>
#include <gtsam/linear/JacobianFactor.h>

#include <gtsam/nonlinear/Ordering.h>

namespace gtsam {

using boost::make_tuple;

// Helper function to fill a vector from a tuple function of any length
template<typename CONS>
inline void __fill_from_tuple(std::vector<Symbol>& vector, size_t position, const CONS& tuple) {
  vector[position] = tuple.get_head();
  __fill_from_tuple<typename CONS::tail_type>(vector, position+1, tuple.get_tail());
}
template<>
inline void __fill_from_tuple<boost::tuples::null_type>(std::vector<Symbol>& vector, size_t position, const boost::tuples::null_type& tuple) {
  // Do nothing
}

/* ************************************************************************* */
/**
 * Nonlinear factor base class
 *
 * Templated on a values structure type. The values structures are typically
 * more general than just vectors, e.g., Rot3 or Pose3,
 * which are objects in non-linear manifolds (Lie groups).
 * \nosubgrouping
 */
template<class VALUES>
class NonlinearFactor: public Factor<Symbol> {

protected:

  // Some handy typedefs
  typedef Factor<Symbol> Base;
  typedef NonlinearFactor<VALUES> This;

public:

  typedef boost::shared_ptr<NonlinearFactor<VALUES> > shared_ptr;

	/// @name Standard Constructors
	/// @{

  /** Default constructor for I/O only */
  NonlinearFactor() {
  }

  /**
   * Constructor
   * @param keys A boost::tuple containing the variables involved in this factor,
   * example: <tt>NonlinearFactor(make_tuple(symbol1, symbol2, symbol3))</tt>
   */
  template<class U1, class U2>
  NonlinearFactor(const boost::tuples::cons<U1,U2>& keys) {
    this->keys_.resize(boost::tuples::length<boost::tuples::cons<U1,U2> >::value);
    // Use helper function to fill key vector, using 'cons' representation of tuple
    __fill_from_tuple(this->keys(), 0, keys);
  }

  /**
   * Constructor
   * @param keys The variables involved in this factor
   */
  template<class ITERATOR>
  NonlinearFactor(ITERATOR beginKeys, ITERATOR endKeys) {
    this->keys_.insert(this->keys_.end(), beginKeys, endKeys);
  }

	/// @}
	/// @name Testable
	/// @{

  /** print */
  virtual void print(const std::string& s = "") const {
    std::cout << s << ": NonlinearFactor\n";
  }

	/// @}
	/// @name Standard Interface
	/// @{

  /** Destructor */
  virtual ~NonlinearFactor() {}


  /**
   * Calculate the error of the factor
   * This is typically equal to log-likelihood, e.g. 0.5(h(x)-z)^2/sigma^2 in case of Gaussian.
   * You can override this for systems with unusual noise models.
   */
  virtual double error(const VALUES& c) const = 0;

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
	virtual bool active(const VALUES& c) const { return true; }

  /** linearize to a GaussianFactor */
  virtual boost::shared_ptr<GaussianFactor>
  linearize(const VALUES& c, const Ordering& ordering) const = 0;

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
template<class VALUES>
class NoiseModelFactor: public NonlinearFactor<VALUES> {

protected:

  // handy typedefs
  typedef NonlinearFactor<VALUES> Base;
  typedef NoiseModelFactor<VALUES> This;

  SharedNoiseModel noiseModel_; /** Noise model */

public:

  typedef boost::shared_ptr<NoiseModelFactor<VALUES> > shared_ptr;

  /** Default constructor for I/O only */
  NoiseModelFactor() {
  }

  /** Destructor */
  virtual ~NoiseModelFactor() {}

  /**
   * Constructor
   * @param keys A boost::tuple containing the variables involved in this factor,
   * example: <tt>NoiseModelFactor(noiseModel, make_tuple(symbol1, symbol2, symbol3)</tt>
   */
  template<class U1, class U2>
  NoiseModelFactor(const SharedNoiseModel& noiseModel, const boost::tuples::cons<U1,U2>& keys)
  : Base(keys), noiseModel_(noiseModel) {
  }

  /**
   * Constructor
   * @param keys The variables involved in this factor
   */
  template<class ITERATOR>
  NoiseModelFactor(const SharedNoiseModel& noiseModel, ITERATOR beginKeys, ITERATOR endKeys)
  : Base(beginKeys, endKeys), noiseModel_(noiseModel) {
  }

protected:

  /**
   * Constructor - only for subclasses, as this does not set keys.
   */
  NoiseModelFactor(const SharedNoiseModel& noiseModel) : noiseModel_(noiseModel) {}

public:

  /** Print */
  virtual void print(const std::string& s = "") const {
    std::cout << s << ": NoiseModelFactor\n";
    std::cout << "  ";
    BOOST_FOREACH(const Symbol& key, this->keys()) { std::cout << (std::string)key << " "; }
    std::cout << "\n";
    this->noiseModel_->print("  noise model: ");
  }

  /** Check if two factors are equal */
  virtual bool equals(const NoiseModelFactor<VALUES>& f, double tol = 1e-9) const {
    return noiseModel_->equals(*f.noiseModel_, tol) && Base::equals(f, tol);
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
  virtual Vector unwhitenedError(const VALUES& x, boost::optional<std::vector<Matrix>&> H = boost::none) const = 0;

  /**
   * Vector of errors, whitened
   * This is the raw error, i.e., i.e. \f$ (h(x)-z)/\sigma \f$ in case of a Gaussian
   */
  Vector whitenedError(const VALUES& c) const {
    return noiseModel_->whiten(unwhitenedError(c));
  }

  /**
   * Calculate the error of the factor.
   * This is the log-likelihood, e.g. \f$ 0.5(h(x)-z)^2/\sigma^2 \f$ in case of Gaussian.
   * In this class, we take the raw prediction error \f$ h(x)-z \f$, ask the noise model
   * to transform it to \f$ (h(x)-z)^2/\sigma^2 \f$, and then multiply by 0.5.
   */
  virtual double error(const VALUES& c) const {
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
  boost::shared_ptr<GaussianFactor> linearize(const VALUES& x, const Ordering& ordering) const {
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
template<class VALUES, class KEY>
class NonlinearFactor1: public NoiseModelFactor<VALUES> {

public:

  // typedefs for value types pulled from keys
  typedef typename KEY::Value X;

protected:

  // The value of the key. Not const to allow serialization
  KEY key_;

  typedef NoiseModelFactor<VALUES> Base;
  typedef NonlinearFactor1<VALUES, KEY> This;

public:

  /** Default constructor for I/O only */
  NonlinearFactor1() {}

  virtual ~NonlinearFactor1() {}

  inline const KEY& key() const {	return key_; }

  /**
   *  Constructor
   *  @param z measurement
   *  @param key by which to look up X value in Values
   */
  NonlinearFactor1(const SharedNoiseModel& noiseModel, const KEY& key1) :
    Base(noiseModel, make_tuple(key1)), key_(key1) {
  }

  /** Calls the 1-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  virtual Vector unwhitenedError(const VALUES& x, boost::optional<std::vector<Matrix>&> H = boost::none) const {
    if(this->active(x)) {
      const X& x1 = x[key_];
      if(H) {
        return evaluateError(x1, (*H)[0]);
      } else {
        return evaluateError(x1);
      }
    } else {
      return zero(this->dim());
    }
  }

  /** Print */
  virtual void print(const std::string& s = "") const {
    std::cout << s << ": NonlinearFactor1(" << (std::string) this->key_ << ")\n";
    this->noiseModel_->print("  noise model: ");
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
    ar & BOOST_SERIALIZATION_NVP(key_);
  }
};// \class NonlinearFactor1


/* ************************************************************************* */
/** A convenient base class for creating your own NoiseModelFactor with 2
 * variables.  To derive from this class, implement evaluateError(). */
template<class VALUES, class KEY1, class KEY2>
class NonlinearFactor2: public NoiseModelFactor<VALUES> {

public:

  // typedefs for value types pulled from keys
  typedef typename KEY1::Value X1;
  typedef typename KEY2::Value X2;

protected:

  // The values of the keys. Not const to allow serialization
  KEY1 key1_;
  KEY2 key2_;

  typedef NoiseModelFactor<VALUES> Base;
  typedef NonlinearFactor2<VALUES, KEY1, KEY2> This;

public:

  /**
   * Default Constructor for I/O
   */
  NonlinearFactor2() {}

  /**
   * Constructor
   * @param j1 key of the first variable
   * @param j2 key of the second variable
   */
  NonlinearFactor2(const SharedNoiseModel& noiseModel, const KEY1& j1, const KEY2& j2) :
    Base(noiseModel, make_tuple(j1,j2)), key1_(j1), key2_(j2) {}

  virtual ~NonlinearFactor2() {}

  /** methods to retrieve both keys */
  inline const KEY1& key1() const { return key1_;	}
  inline const KEY2& key2() const {	return key2_;	}

  /** Calls the 2-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  virtual Vector unwhitenedError(const VALUES& x, boost::optional<std::vector<Matrix>&> H = boost::none) const {
    if(this->active(x)) {
      const X1& x1 = x[key1_];
      const X2& x2 = x[key2_];
      if(H) {
        return evaluateError(x1, x2, (*H)[0], (*H)[1]);
      } else {
        return evaluateError(x1, x2);
      }
    } else {
      return zero(this->dim());
    }
  }

  /** Print */
  virtual void print(const std::string& s = "") const {
    std::cout << s << ": NonlinearFactor2("
    		<< (std::string) this->key1_ << ","
    		<< (std::string) this->key2_ << ")\n";
    this->noiseModel_->print("  noise model: ");
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
    ar & BOOST_SERIALIZATION_NVP(key1_);
    ar & BOOST_SERIALIZATION_NVP(key2_);
  }
}; // \class NonlinearFactor2

/* ************************************************************************* */
/** A convenient base class for creating your own NoiseModelFactor with 3
 * variables.  To derive from this class, implement evaluateError(). */
template<class VALUES, class KEY1, class KEY2, class KEY3>
class NonlinearFactor3: public NoiseModelFactor<VALUES> {

public:

  // typedefs for value types pulled from keys
  typedef typename KEY1::Value X1;
  typedef typename KEY2::Value X2;
  typedef typename KEY3::Value X3;

protected:

  // The values of the keys. Not const to allow serialization
  KEY1 key1_;
  KEY2 key2_;
  KEY3 key3_;

  typedef NoiseModelFactor<VALUES> Base;
  typedef NonlinearFactor3<VALUES, KEY1, KEY2, KEY3> This;

public:

  /**
   * Default Constructor for I/O
   */
  NonlinearFactor3() {}

  /**
   * Constructor
   * @param j1 key of the first variable
   * @param j2 key of the second variable
   * @param j3 key of the third variable
   */
  NonlinearFactor3(const SharedNoiseModel& noiseModel, const KEY1& j1, const KEY2& j2, const KEY3& j3) :
    Base(noiseModel, make_tuple(j1,j2,j3)), key1_(j1), key2_(j2), key3_(j3) {}

  virtual ~NonlinearFactor3() {}

  /** methods to retrieve keys */
  inline const KEY1& key1() const { return key1_; }
  inline const KEY2& key2() const { return key2_; }
  inline const KEY3& key3() const { return key3_; }

  /** Calls the 3-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  virtual Vector unwhitenedError(const VALUES& x, boost::optional<std::vector<Matrix>&> H = boost::none) const {
    if(this->active(x)) {
      if(H)
        return evaluateError(x[key1_], x[key2_], x[key3_], (*H)[0], (*H)[1], (*H)[2]);
      else
        return evaluateError(x[key1_], x[key2_], x[key3_]);
    } else {
      return zero(this->dim());
    }
  }

  /** Print */
  virtual void print(const std::string& s = "") const {
    std::cout << s << ": NonlinearFactor3("
    		<< (std::string) this->key1_ << ","
    		<< (std::string) this->key2_ << ","
    		<< (std::string) this->key3_ << ")\n";
    this->noiseModel_->print("  noise model: ");
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
    ar & BOOST_SERIALIZATION_NVP(key1_);
    ar & BOOST_SERIALIZATION_NVP(key2_);
    ar & BOOST_SERIALIZATION_NVP(key3_);
  }
}; // \class NonlinearFactor3

/* ************************************************************************* */
/** A convenient base class for creating your own NoiseModelFactor with 4
 * variables.  To derive from this class, implement evaluateError(). */
template<class VALUES, class KEY1, class KEY2, class KEY3, class KEY4>
class NonlinearFactor4: public NoiseModelFactor<VALUES> {

public:

  // typedefs for value types pulled from keys
  typedef typename KEY1::Value X1;
  typedef typename KEY2::Value X2;
  typedef typename KEY3::Value X3;
  typedef typename KEY4::Value X4;

protected:

  // The values of the keys. Not const to allow serialization
  KEY1 key1_;
  KEY2 key2_;
  KEY3 key3_;
  KEY4 key4_;

  typedef NoiseModelFactor<VALUES> Base;
  typedef NonlinearFactor4<VALUES, KEY1, KEY2, KEY3, KEY4> This;

public:

  /**
   * Default Constructor for I/O
   */
  NonlinearFactor4() {}

  /**
   * Constructor
   * @param j1 key of the first variable
   * @param j2 key of the second variable
   * @param j3 key of the third variable
   * @param j4 key of the fourth variable
   */
  NonlinearFactor4(const SharedNoiseModel& noiseModel, const KEY1& j1, const KEY2& j2, const KEY3& j3, const KEY4& j4) :
    Base(noiseModel, make_tuple(j1,j2,j3,j4)), key1_(j1), key2_(j2), key3_(j3), key4_(j4) {}

  virtual ~NonlinearFactor4() {}

  /** methods to retrieve keys */
  inline const KEY1& key1() const { return key1_; }
  inline const KEY2& key2() const { return key2_; }
  inline const KEY3& key3() const { return key3_; }
  inline const KEY4& key4() const { return key4_; }

  /** Calls the 4-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  virtual Vector unwhitenedError(const VALUES& x, boost::optional<std::vector<Matrix>&> H = boost::none) const {
  	if(this->active(x)) {
  		if(H)
  			return evaluateError(x[key1_], x[key2_], x[key3_], x[key4_], (*H)[0], (*H)[1], (*H)[2], (*H)[3]);
  		else
  			return evaluateError(x[key1_], x[key2_], x[key3_], x[key4_]);
  	} else {
  		return zero(this->dim());
  	}
  }

  /** Print */
  virtual void print(const std::string& s = "") const {
    std::cout << s << ": NonlinearFactor4("
    		<< (std::string) this->key1_ << ","
    		<< (std::string) this->key2_ << ","
     		<< (std::string) this->key3_ << ","
    		<< (std::string) this->key4_ << ")\n";
    this->noiseModel_->print("  noise model: ");
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
    ar & BOOST_SERIALIZATION_NVP(key1_);
    ar & BOOST_SERIALIZATION_NVP(key2_);
    ar & BOOST_SERIALIZATION_NVP(key3_);
    ar & BOOST_SERIALIZATION_NVP(key4_);
  }
}; // \class NonlinearFactor4

/* ************************************************************************* */
/** A convenient base class for creating your own NoiseModelFactor with 5
 * variables.  To derive from this class, implement evaluateError(). */
template<class VALUES, class KEY1, class KEY2, class KEY3, class KEY4, class KEY5>
class NonlinearFactor5: public NoiseModelFactor<VALUES> {

public:

  // typedefs for value types pulled from keys
  typedef typename KEY1::Value X1;
  typedef typename KEY2::Value X2;
  typedef typename KEY3::Value X3;
  typedef typename KEY4::Value X4;
  typedef typename KEY5::Value X5;

protected:

  // The values of the keys. Not const to allow serialization
  KEY1 key1_;
  KEY2 key2_;
  KEY3 key3_;
  KEY4 key4_;
  KEY5 key5_;

  typedef NoiseModelFactor<VALUES> Base;
  typedef NonlinearFactor5<VALUES, KEY1, KEY2, KEY3, KEY4, KEY5> This;

public:

  /**
   * Default Constructor for I/O
   */
  NonlinearFactor5() {}

  /**
   * Constructor
   * @param j1 key of the first variable
   * @param j2 key of the second variable
   * @param j3 key of the third variable
   * @param j4 key of the fourth variable
   * @param j5 key of the fifth variable
   */
  NonlinearFactor5(const SharedNoiseModel& noiseModel, const KEY1& j1, const KEY2& j2, const KEY3& j3, const KEY4& j4, const KEY5& j5) :
    Base(noiseModel, make_tuple(j1,j2,j3,j4,j5)), key1_(j1), key2_(j2), key3_(j3), key4_(j4), key5_(j5) {}

  virtual ~NonlinearFactor5() {}

  /** methods to retrieve keys */
  inline const KEY1& key1() const { return key1_; }
  inline const KEY2& key2() const { return key2_; }
  inline const KEY3& key3() const { return key3_; }
  inline const KEY4& key4() const { return key4_; }
  inline const KEY5& key5() const { return key5_; }

  /** Calls the 5-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  virtual Vector unwhitenedError(const VALUES& x, boost::optional<std::vector<Matrix>&> H = boost::none) const {
  	if(this->active(x)) {
      if(H)
        return evaluateError(x[key1_], x[key2_], x[key3_], x[key4_], x[key5_], (*H)[0], (*H)[1], (*H)[2], (*H)[3], (*H)[4]);
      else
        return evaluateError(x[key1_], x[key2_], x[key3_], x[key4_], x[key5_]);
  	} else {
  		return zero(this->dim());
  	}
  }

  /** Print */
  virtual void print(const std::string& s = "") const {
    std::cout << s << ": NonlinearFactor5("
    		<< (std::string) this->key1_ << ","
    		<< (std::string) this->key2_ << ","
     		<< (std::string) this->key3_ << ","
    		<< (std::string) this->key4_ << ","
    		<< (std::string) this->key5_ << ")\n";
    this->noiseModel_->print("  noise model: ");
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
    ar & BOOST_SERIALIZATION_NVP(key1_);
    ar & BOOST_SERIALIZATION_NVP(key2_);
    ar & BOOST_SERIALIZATION_NVP(key3_);
    ar & BOOST_SERIALIZATION_NVP(key4_);
    ar & BOOST_SERIALIZATION_NVP(key5_);
  }
}; // \class NonlinearFactor5

/* ************************************************************************* */
/** A convenient base class for creating your own NoiseModelFactor with 6
 * variables.  To derive from this class, implement evaluateError(). */
template<class VALUES, class KEY1, class KEY2, class KEY3, class KEY4, class KEY5, class KEY6>
class NonlinearFactor6: public NoiseModelFactor<VALUES> {

public:

  // typedefs for value types pulled from keys
  typedef typename KEY1::Value X1;
  typedef typename KEY2::Value X2;
  typedef typename KEY3::Value X3;
  typedef typename KEY4::Value X4;
  typedef typename KEY5::Value X5;
  typedef typename KEY6::Value X6;

protected:

  // The values of the keys. Not const to allow serialization
  KEY1 key1_;
  KEY2 key2_;
  KEY3 key3_;
  KEY4 key4_;
  KEY5 key5_;
  KEY6 key6_;

  typedef NoiseModelFactor<VALUES> Base;
  typedef NonlinearFactor6<VALUES, KEY1, KEY2, KEY3, KEY4, KEY5, KEY6> This;

public:

  /**
   * Default Constructor for I/O
   */
  NonlinearFactor6() {}

  /**
   * Constructor
   * @param j1 key of the first variable
   * @param j2 key of the second variable
   * @param j3 key of the third variable
   * @param j4 key of the fourth variable
   * @param j5 key of the fifth variable
   * @param j6 key of the fifth variable
   */
  NonlinearFactor6(const SharedNoiseModel& noiseModel, const KEY1& j1, const KEY2& j2, const KEY3& j3, const KEY4& j4, const KEY5& j5, const KEY6& j6) :
    Base(noiseModel, make_tuple(j1,j2,j3,j4,j5,j6)), key1_(j1), key2_(j2), key3_(j3), key4_(j4), key5_(j5), key6_(j6) {}

  virtual ~NonlinearFactor6() {}

  /** methods to retrieve keys */
  inline const KEY1& key1() const { return key1_; }
  inline const KEY2& key2() const { return key2_; }
  inline const KEY3& key3() const { return key3_; }
  inline const KEY4& key4() const { return key4_; }
  inline const KEY5& key5() const { return key5_; }
  inline const KEY6& key6() const { return key6_; }

  /** Calls the 6-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  virtual Vector unwhitenedError(const VALUES& x, boost::optional<std::vector<Matrix>&> H = boost::none) const {
  	if(this->active(x)) {
      if(H)
        return evaluateError(x[key1_], x[key2_], x[key3_], x[key4_], x[key5_], x[key6_], (*H)[0], (*H)[1], (*H)[2], (*H)[3], (*H)[4], (*H)[5]);
      else
        return evaluateError(x[key1_], x[key2_], x[key3_], x[key4_], x[key5_], x[key6_]);
  	} else {
  		return zero(this->dim());
  	}
  }

  /** Print */
  virtual void print(const std::string& s = "") const {
    std::cout << s << ": NonlinearFactor6("
    		<< (std::string) this->key1_ << ","
    		<< (std::string) this->key2_ << ","
     		<< (std::string) this->key3_ << ","
    		<< (std::string) this->key4_ << ","
    		<< (std::string) this->key5_ << ","
    		<< (std::string) this->key6_ << ")\n";
    this->noiseModel_->print("  noise model: ");
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
    ar & BOOST_SERIALIZATION_NVP(key1_);
    ar & BOOST_SERIALIZATION_NVP(key2_);
    ar & BOOST_SERIALIZATION_NVP(key3_);
    ar & BOOST_SERIALIZATION_NVP(key4_);
    ar & BOOST_SERIALIZATION_NVP(key5_);
    ar & BOOST_SERIALIZATION_NVP(key6_);
  }
}; // \class NonlinearFactor6

/* ************************************************************************* */

} // \namespace gtsam
