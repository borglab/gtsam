/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file NonlinearConstraint.h
 * @brief Implements nonlinear constraints that can be linearized using
 * direct linearization and solving through a quadratic merit function
 * @author Alex Cunningham
 */

#pragma once

#include <boost/function.hpp>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/**
 * Base class for nonlinear constraints
 * This allows for both equality and inequality constraints,
 * where equality constraints are active all the time (even slightly
 * nonzero constraint functions will still be active - inequality
 * constraints should be sure to force to actual zero)
 *
 * Nonlinear constraints evaluate their error as a part of a quadratic
 * error function: ||h(x)-z||^2 + mu * ||c(x)|| where mu is a gain
 * on the constraint function that should be made high enough to be
 * significant
 */
template <class VALUES>
class NonlinearConstraint : public NoiseModelFactor<VALUES> {

protected:
	typedef NonlinearConstraint<VALUES> This;
	typedef NoiseModelFactor<VALUES> Base;

	/** default constructor to allow for serialization */
	NonlinearConstraint() {}

	double mu_;  /// gain for quadratic merit function

public:

	/** Constructor - sets the cost function and the lagrange multipliers
	 * @param dim is the dimension of the factor
	 * @param keys is a boost::tuple containing the keys, e.g. \c make_tuple(key1,key2,key3)
	 * @param mu is the gain used at error evaluation (forced to be positive)
	 */
	template<class TUPLE>
	NonlinearConstraint(const TUPLE& keys, size_t dim, double mu = 1000.0):
		Base(noiseModel::Constrained::All(dim), keys), mu_(fabs(mu)) {}
	virtual ~NonlinearConstraint() {}

	/** returns the gain mu */
	double mu() const { return mu_; }

	/** Print */
	virtual void print(const std::string& s = "") const {
    std::cout << "NonlinearConstraint " << s << std::endl;
    std::cout << "  ";
    BOOST_FOREACH(const Symbol& key, this->keys()) { std::cout << (std::string)key << " "; }
    std::cout << "\n";
    std::cout << "mu: " << this->mu_ << std::endl;
	}

	/** Check if two factors are equal */
	virtual bool equals(const NonlinearFactor<VALUES>& f, double tol=1e-9) const {
		const This* p = dynamic_cast<const This*> (&f);
		if (p == NULL) return false;
		return Base::equals(*p, tol) && (fabs(mu_ - p->mu_) <= tol);
	}

	/** error function - returns the quadratic merit function */
	virtual double error(const VALUES& c) const  {
		if (active(c))
			return mu_ * unwhitenedError(c).squaredNorm();
		else
		  return 0.0;
	}

	/**
	 * active set check, defines what type of constraint this is
	 *
	 * In an inequality/bounding constraint, this active() returns true
	 * when the constraint is *NOT* fulfilled.
	 * @return true if the constraint is active
	 */
	virtual bool active(const VALUES& c) const=0;

	/**
	 * Linearizes around a given config
	 * @param config is the values structure
	 * @return a combined linear factor containing both the constraint and the constraint factor
	 */
	virtual boost::shared_ptr<GaussianFactor> linearize(const VALUES& c, const Ordering& ordering) const {
    if (!active(c))
      return boost::shared_ptr<JacobianFactor>();
    else
      return Base::linearize(c, ordering);
	}

private:

	/** Serialization function */
	friend class boost::serialization::access;
	template<class ARCHIVE>
	void serialize(ARCHIVE & ar, const unsigned int version) {
		ar & boost::serialization::make_nvp("NonlinearFactor",
				boost::serialization::base_object<Base>(*this));
		ar & BOOST_SERIALIZATION_NVP(mu_);
	}
};


/**
 * A unary constraint that defaults to an equality constraint
 */
template <class VALUES, class KEY>
class NonlinearConstraint1 : public NonlinearConstraint<VALUES> {

public:
	typedef typename KEY::Value X;

protected:
	typedef NonlinearConstraint1<VALUES,KEY> This;
	typedef NonlinearConstraint<VALUES> Base;

	/** default constructor to allow for serialization */
	NonlinearConstraint1() {}

	/** key for the constrained variable */
	KEY key_;

public:

	/**
	 * Basic constructor
	 * @param key is the identifier for the variable constrained
	 * @param dim is the size of the constraint (p)
	 * @param mu is the gain for the factor
	 */
	NonlinearConstraint1(const KEY& key, size_t dim, double mu = 1000.0)
		: Base(make_tuple(key), dim, mu), key_(key) { }
	virtual ~NonlinearConstraint1() {}

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
		ar & boost::serialization::make_nvp("NonlinearConstraint",
				boost::serialization::base_object<Base>(*this));
		ar & BOOST_SERIALIZATION_NVP(key_);
	}
};

/**
 * Unary Equality constraint - simply forces the value of active() to true
 */
template <class VALUES, class KEY>
class NonlinearEqualityConstraint1 : public NonlinearConstraint1<VALUES, KEY> {

public:
	typedef typename KEY::Value X;

protected:
	typedef NonlinearEqualityConstraint1<VALUES,KEY> This;
	typedef NonlinearConstraint1<VALUES,KEY> Base;

	/** default constructor to allow for serialization */
	NonlinearEqualityConstraint1() {}

public:
	NonlinearEqualityConstraint1(const KEY& key, size_t dim, double mu = 1000.0)
		: Base(key, dim, mu) {}
	virtual ~NonlinearEqualityConstraint1() {}

	/** Always active, so fixed value for active() */
	virtual bool active(const VALUES& c) const { return true; }

private:

	/** Serialization function */
	friend class boost::serialization::access;
	template<class ARCHIVE>
	void serialize(ARCHIVE & ar, const unsigned int version) {
		ar & boost::serialization::make_nvp("NonlinearConstraint1",
				boost::serialization::base_object<Base>(*this));
	}

};

/**
 * A binary constraint with arbitrary cost and jacobian functions
 */
template <class VALUES, class KEY1, class KEY2>
class NonlinearConstraint2 : public NonlinearConstraint<VALUES> {

public:
	typedef typename KEY1::Value X1;
	typedef typename KEY2::Value X2;

protected:
	typedef NonlinearConstraint2<VALUES,KEY1,KEY2> This;
	typedef NonlinearConstraint<VALUES> Base;

	/** default constructor to allow for serialization */
	NonlinearConstraint2() {}

	/** keys for the constrained variables */
	KEY1 key1_;
	KEY2 key2_;

public:

	/**
	 * Basic constructor
	 * @param key1 is the identifier for the first variable constrained
	 * @param key2 is the identifier for the second variable constrained
	 * @param dim is the size of the constraint (p)
	 * @param mu is the gain for the factor
	 */
	NonlinearConstraint2(const KEY1& key1, const KEY2& key2, size_t dim, double mu = 1000.0) :
			Base(make_tuple(key1, key2), dim, mu), key1_(key1), key2_(key2) { }
	virtual ~NonlinearConstraint2() {}

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
		ar & boost::serialization::make_nvp("NonlinearConstraint",
				boost::serialization::base_object<Base>(*this));
		ar & BOOST_SERIALIZATION_NVP(key1_);
		ar & BOOST_SERIALIZATION_NVP(key2_);
	}
};

/**
 * Binary Equality constraint - simply forces the value of active() to true
 */
template <class VALUES, class KEY1, class KEY2>
class NonlinearEqualityConstraint2 : public NonlinearConstraint2<VALUES, KEY1, KEY2> {

public:
	typedef typename KEY1::Value X1;
	typedef typename KEY2::Value X2;

protected:
	typedef NonlinearEqualityConstraint2<VALUES,KEY1,KEY2> This;
	typedef NonlinearConstraint2<VALUES,KEY1,KEY2> Base;

	/** default constructor to allow for serialization */
	NonlinearEqualityConstraint2() {}

public:
	NonlinearEqualityConstraint2(const KEY1& key1, const KEY2& key2, size_t dim, double mu = 1000.0)
		: Base(key1, key2, dim, mu) {}
	virtual ~NonlinearEqualityConstraint2() {}

	/** Always active, so fixed value for active() */
	virtual bool active(const VALUES& c) const { return true; }

private:

	/** Serialization function */
	friend class boost::serialization::access;
	template<class ARCHIVE>
	void serialize(ARCHIVE & ar, const unsigned int version) {
		ar & boost::serialization::make_nvp("NonlinearConstraint2",
				boost::serialization::base_object<Base>(*this));
	}
};

/**
 * A ternary constraint
 */
template <class VALUES, class KEY1, class KEY2, class KEY3>
class NonlinearConstraint3 : public NonlinearConstraint<VALUES> {

public:
	typedef typename KEY1::Value X1;
	typedef typename KEY2::Value X2;
	typedef typename KEY3::Value X3;

protected:
	typedef NonlinearConstraint3<VALUES,KEY1,KEY2,KEY3> This;
	typedef NonlinearConstraint<VALUES> Base;

	/** default constructor to allow for serialization */
	NonlinearConstraint3() {}

	/** keys for the constrained variables */
	KEY1 key1_;
	KEY2 key2_;
	KEY3 key3_;

public:

	/**
	 * Basic constructor
	 * @param key1 is the identifier for the first variable constrained
	 * @param key2 is the identifier for the second variable constrained
	 * @param key3 is the identifier for the second variable constrained
	 * @param dim is the size of the constraint (p)
	 * @param mu is the gain for the factor
	 */
	NonlinearConstraint3(const KEY1& key1, const KEY2& key2, const KEY3& key3,
			size_t dim, double mu = 1000.0) :
			Base(make_tuple(key1, key2, key3), dim, mu), key1_(key1), key2_(key2), key3_(key3) { }
	virtual ~NonlinearConstraint3() {}

  /** Calls the 2-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  virtual Vector unwhitenedError(const VALUES& x, boost::optional<std::vector<Matrix>&> H = boost::none) const {
    if(this->active(x)) {
      const X1& x1 = x[key1_];
      const X2& x2 = x[key2_];
      const X3& x3 = x[key3_];
      if(H) {
        return evaluateError(x1, x2, x3, (*H)[0], (*H)[1], (*H)[2]);
      } else {
        return evaluateError(x1, x2, x3);
      }
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
		ar & boost::serialization::make_nvp("NonlinearConstraint",
				boost::serialization::base_object<Base>(*this));
		ar & BOOST_SERIALIZATION_NVP(key1_);
		ar & BOOST_SERIALIZATION_NVP(key2_);
		ar & BOOST_SERIALIZATION_NVP(key3_);
	}
};

/**
 * Ternary Equality constraint - simply forces the value of active() to true
 */
template <class VALUES, class KEY1, class KEY2, class KEY3>
class NonlinearEqualityConstraint3 : public NonlinearConstraint3<VALUES, KEY1, KEY2, KEY3> {

public:
	typedef typename KEY1::Value X1;
	typedef typename KEY2::Value X2;
	typedef typename KEY3::Value X3;

protected:
	typedef NonlinearEqualityConstraint3<VALUES,KEY1,KEY2,KEY3> This;
	typedef NonlinearConstraint3<VALUES,KEY1,KEY2,KEY3> Base;

	/** default constructor to allow for serialization */
	NonlinearEqualityConstraint3() {}

public:
	NonlinearEqualityConstraint3(const KEY1& key1, const KEY2& key2, const KEY3& key3,
			size_t dim, double mu = 1000.0)
		: Base(key1, key2, key3, dim, mu) {}
	virtual ~NonlinearEqualityConstraint3() {}

	/** Always active, so fixed value for active() */
	virtual bool active(const VALUES& c) const { return true; }

private:

	/** Serialization function */
	friend class boost::serialization::access;
	template<class ARCHIVE>
	void serialize(ARCHIVE & ar, const unsigned int version) {
		ar & boost::serialization::make_nvp("NonlinearConstraint3",
				boost::serialization::base_object<Base>(*this));
	}
};


/**
 * Simple unary equality constraint - fixes a value for a variable
 */
template<class VALUES, class KEY>
class NonlinearEquality1 : public NonlinearEqualityConstraint1<VALUES, KEY> {

public:
	typedef typename KEY::Value X;

protected:
	typedef NonlinearEqualityConstraint1<VALUES, KEY> Base;

	/** default constructor to allow for serialization */
	NonlinearEquality1() {}

	X value_; /// fixed value for variable

public:

	typedef boost::shared_ptr<NonlinearEquality1<VALUES, KEY> > shared_ptr;

	NonlinearEquality1(const X& value, const KEY& key1, double mu = 1000.0)
		: Base(key1, X::Dim(), mu), value_(value) {}
	virtual ~NonlinearEquality1() {}

	/** g(x) with optional derivative */
	Vector evaluateError(const X& x1, boost::optional<Matrix&> H1 = boost::none) const {
		const size_t p = X::Dim();
		if (H1) *H1 = eye(p);
		return value_.logmap(x1);
	}

private:

	/** Serialization function */
	friend class boost::serialization::access;
	template<class ARCHIVE>
	void serialize(ARCHIVE & ar, const unsigned int version) {
		ar & boost::serialization::make_nvp("NonlinearEqualityConstraint1",
				boost::serialization::base_object<Base>(*this));
		ar & BOOST_SERIALIZATION_NVP(value_);
	}
};


/**
 * Simple binary equality constraint - this constraint forces two factors to
 * be the same.  This constraint requires the underlying type to a Lie type
 */
template<class VALUES, class KEY>
class NonlinearEquality2 : public NonlinearEqualityConstraint2<VALUES, KEY, KEY> {
public:
	typedef typename KEY::Value X;

protected:
	typedef NonlinearEqualityConstraint2<VALUES, KEY, KEY> Base;

	/** default constructor to allow for serialization */
	NonlinearEquality2() {}

public:

	typedef boost::shared_ptr<NonlinearEquality2<VALUES, KEY> > shared_ptr;

	NonlinearEquality2(const KEY& key1, const KEY& key2, double mu = 1000.0)
		: Base(key1, key2, X::Dim(), mu) {}
	virtual ~NonlinearEquality2() {}

	/** g(x) with optional derivative2 */
	Vector evaluateError(const X& x1, const X& x2,
			boost::optional<Matrix&> H1 = boost::none,
			boost::optional<Matrix&> H2 = boost::none) const {
		const size_t p = X::Dim();
		if (H1) *H1 = -eye(p);
		if (H2) *H2 = eye(p);
		return x1.logmap(x2);
	}

private:

	/** Serialization function */
	friend class boost::serialization::access;
	template<class ARCHIVE>
	void serialize(ARCHIVE & ar, const unsigned int version) {
		ar & boost::serialization::make_nvp("NonlinearEqualityConstraint2",
				boost::serialization::base_object<Base>(*this));
	}
};

}
