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

#include <map>
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
 * NOTE: inequality constraints removed for now
 *
 * Nonlinear constraints evaluate their error as a part of a quadratic
 * error function: ||h(x)-z||^2 + mu * ||c(x)|| where mu is a gain
 * on the constraint function that should be made high enough to be
 * significant
 */
template <class Values>
class NonlinearConstraint : public NonlinearFactor<Values> {

protected:
	typedef NonlinearConstraint<Values> This;
	typedef NonlinearFactor<Values> Base;

	double mu_;  /// gain for quadratic merit function
	size_t dim_; /// dimension of the constraint

public:

	/** Constructor - sets the cost function and the lagrange multipliers
	 * @param dim is the dimension of the factor
	 * @param mu is the gain used at error evaluation (forced to be positive)
	 */
	NonlinearConstraint(size_t dim, double mu = 1000.0):
		Base(noiseModel::Constrained::All(dim)), mu_(fabs(mu)), dim_(dim) {}
	virtual ~NonlinearConstraint() {}

	/** returns the gain mu */
	double mu() const { return mu_; }

	/** Print */
	virtual void print(const std::string& s = "") const=0;

	/** dimension of the constraint (number of rows) */
	size_t dim() const { return dim_; }

	/** Check if two factors are equal */
	virtual bool equals(const NonlinearFactor<Values>& f, double tol=1e-9) const {
		const This* p = dynamic_cast<const This*> (&f);
		if (p == NULL) return false;
		return Base::equals(*p, tol) && (mu_ == p->mu_);
	}

	/** error function - returns the quadratic merit function */
	virtual double error(const Values& c) const  {
		const Vector error_vector = unwhitenedError(c);
		if (active(c))
			return mu_ * inner_prod(error_vector, error_vector);
		else return 0.0;
	}

	/** Raw error vector function g(x) */
	virtual Vector unwhitenedError(const Values& c) const = 0;

	/**
	 * active set check, defines what type of constraint this is
	 *
	 * In an inequality/bounding constraint, this active() returns true
	 * when the constraint is *NOT* fulfilled.
	 * @return true if the constraint is active
	 */
	virtual bool active(const Values& c) const=0;

	/**
	 * Linearizes around a given config
	 * @param config is the values structure
	 * @return a combined linear factor containing both the constraint and the constraint factor
	 */
	virtual boost::shared_ptr<GaussianFactor> linearize(const Values& c, const Ordering& ordering) const=0;
};


/**
 * A unary constraint that defaults to an equality constraint
 */
template <class Values, class Key>
class NonlinearConstraint1 : public NonlinearConstraint<Values> {

public:
	typedef typename Key::Value X;

protected:
	typedef NonlinearConstraint1<Values,Key> This;
	typedef NonlinearConstraint<Values> Base;

	/** key for the constrained variable */
	Key key_;

public:

	/**
	 * Basic constructor
	 * @param key is the identifier for the variable constrained
	 * @param dim is the size of the constraint (p)
	 * @param mu is the gain for the factor
	 */
	NonlinearConstraint1(const Key& key, size_t dim, double mu = 1000.0)
		: Base(dim, mu), key_(key) {
		this->keys_.push_back(key);
	}
	virtual ~NonlinearConstraint1() {}

	/* print */
	void print(const std::string& s = "") const {
		std::cout << "NonlinearConstraint1 " << s << std::endl;
		std::cout << "key: " << (std::string) key_ << std::endl;
		std::cout << "mu: " << this->mu_ << std::endl;
	}

	/** Check if two factors are equal. Note type is Factor and needs cast. */
	virtual bool equals(const NonlinearFactor<Values>& f, double tol = 1e-9) const {
		const This* p = dynamic_cast<const This*> (&f);
		if (p == NULL) return false;
		return Base::equals(*p, tol) && (key_ == p->key_);
	}

	/** error function g(x), switched depending on whether the constraint is active */
	inline Vector unwhitenedError(const Values& x) const {
		if (!active(x)) {
			return zero(this->dim());
		}
		const Key& j = key_;
		const X& xj = x[j];
		return evaluateError(xj);
	}

	/** Linearize from config */
	boost::shared_ptr<GaussianFactor> linearize(const Values& x, const Ordering& ordering) const {
		if (!active(x)) {
			boost::shared_ptr<GaussianFactor> factor;
			return factor;
		}
		const X& xj = x[key_];
		Matrix A;
		Vector b = - evaluateError(xj, A);
		Index var = ordering[key_];
		SharedDiagonal model = noiseModel::Constrained::All(this->dim());
		return GaussianFactor::shared_ptr(new GaussianFactor(var, A, b, model));
	}

	/** g(x) with optional derivative - does not depend on active */
	virtual Vector evaluateError(const X& x, boost::optional<Matrix&> H =
			boost::none) const = 0;

	/**
	 * Create a symbolic factor using the given ordering to determine the
	 * variable indices.
	 */
	virtual Factor::shared_ptr symbolic(const Ordering& ordering) const {
		return Factor::shared_ptr(new Factor(ordering[key_]));
	}
};

/**
 * Unary Equality constraint - simply forces the value of active() to true
 */
template <class Values, class Key>
class NonlinearEqualityConstraint1 : public NonlinearConstraint1<Values, Key> {

public:
	typedef typename Key::Value X;

protected:
	typedef NonlinearEqualityConstraint1<Values,Key> This;
	typedef NonlinearConstraint1<Values,Key> Base;

public:
	NonlinearEqualityConstraint1(const Key& key, size_t dim, double mu = 1000.0)
		: Base(key, dim, mu) {}
	virtual ~NonlinearEqualityConstraint1() {}

	/** Always active, so fixed value for active() */
	virtual bool active(const Values& c) const { return true; }
};

/**
 * A binary constraint with arbitrary cost and jacobian functions
 */
template <class Values, class Key1, class Key2>
class NonlinearConstraint2 : public NonlinearConstraint<Values> {

public:
	typedef typename Key1::Value X1;
	typedef typename Key2::Value X2;

protected:
	typedef NonlinearConstraint2<Values,Key1,Key2> This;
	typedef NonlinearConstraint<Values> Base;

	/** keys for the constrained variables */
	Key1 key1_;
	Key2 key2_;

public:

	/**
	 * Basic constructor
	 * @param key1 is the identifier for the first variable constrained
	 * @param key2 is the identifier for the second variable constrained
	 * @param dim is the size of the constraint (p)
	 * @param mu is the gain for the factor
	 */
	NonlinearConstraint2(const Key1& key1, const Key2& key2, size_t dim, double mu = 1000.0) :
			Base(dim, mu), key1_(key1), key2_(key2) {
		this->keys_.push_back(key1);
		this->keys_.push_back(key2);
	}
	virtual ~NonlinearConstraint2() {}

	/* print */
	void print(const std::string& s = "") const {
		std::cout << "NonlinearConstraint2 " << s << std::endl;
		std::cout << "key1: " << (std::string) key1_ << std::endl;
		std::cout << "key2: " << (std::string) key2_ << std::endl;
		std::cout << "mu: " << this->mu_ << std::endl;
	}

	/** Check if two factors are equal. Note type is Factor and needs cast. */
	virtual bool equals(const NonlinearFactor<Values>& f, double tol = 1e-9) const {
		const This* p = dynamic_cast<const This*> (&f);
		if (p == NULL) return false;
		return Base::equals(*p, tol) && (key1_ == p->key1_) && (key2_ == p->key2_);
	}

	/** error function g(x), switched depending on whether the constraint is active */
	inline Vector unwhitenedError(const Values& x) const {
		if (!active(x)) {
			return zero(this->dim());
		}
		const Key1& j1 = key1_;
		const Key2& j2 = key2_;
		const X1& xj1 = x[j1];
		const X2& xj2 = x[j2];
		return evaluateError(xj1, xj2);
	}

	/** Linearize from config */
	boost::shared_ptr<GaussianFactor> linearize(const Values& c, const Ordering& ordering) const {
		if (!active(c)) {
			boost::shared_ptr<GaussianFactor> factor;
			return factor;
		}
		const Key1& j1 = key1_; const Key2& j2 = key2_;
		const X1& x1 = c[j1]; const X2& x2 = c[j2];
		Matrix grad1, grad2;
		Vector g = -1.0 * evaluateError(x1, x2, grad1, grad2);
		SharedDiagonal model = noiseModel::Constrained::All(this->dim());
		Index var1 = ordering[j1], var2 = ordering[j2];
		if (var1 < var2)
			GaussianFactor::shared_ptr(new GaussianFactor(var1, grad1, var2, grad2, g, model));
		else
			GaussianFactor::shared_ptr(new GaussianFactor(var2, grad2, var1, grad1, g, model));
	}

	/** g(x) with optional derivative2  - does not depend on active */
	virtual Vector evaluateError(const X1& x1, const X2& x2,
			boost::optional<Matrix&> H1 = boost::none,
			boost::optional<Matrix&> H2 = boost::none) const = 0;

	/**
	 * Create a symbolic factor using the given ordering to determine the
	 * variable indices.
	 */
	virtual Factor::shared_ptr symbolic(const Ordering& ordering) const {
		const Index var1 = ordering[key1_], var2 = ordering[key2_];
		if(var1 < var2)
			return Factor::shared_ptr(new Factor(var1, var2));
		else
			return Factor::shared_ptr(new Factor(var2, var1));
	}
};

/**
 * Binary Equality constraint - simply forces the value of active() to true
 */
template <class Values, class Key1, class Key2>
class NonlinearEqualityConstraint2 : public NonlinearConstraint2<Values, Key1, Key2> {

public:
	typedef typename Key1::Value X1;
	typedef typename Key2::Value X2;

protected:
	typedef NonlinearEqualityConstraint2<Values,Key1,Key2> This;
	typedef NonlinearConstraint2<Values,Key1,Key2> Base;

public:
	NonlinearEqualityConstraint2(const Key1& key1, const Key2& key2, size_t dim, double mu = 1000.0)
		: Base(key1, key2, dim, mu) {}
	virtual ~NonlinearEqualityConstraint2() {}


	/** Always active, so fixed value for active() */
	virtual bool active(const Values& c) const { return true; }
};

/**
 * A ternary constraint
 */
template <class Values, class Key1, class Key2, class Key3>
class NonlinearConstraint3 : public NonlinearConstraint<Values> {

public:
	typedef typename Key1::Value X1;
	typedef typename Key2::Value X2;
	typedef typename Key3::Value X3;

protected:
	typedef NonlinearConstraint3<Values,Key1,Key2,Key3> This;
	typedef NonlinearConstraint<Values> Base;

	/** keys for the constrained variables */
	Key1 key1_;
	Key2 key2_;
	Key3 key3_;

public:

	/**
	 * Basic constructor
	 * @param key1 is the identifier for the first variable constrained
	 * @param key2 is the identifier for the second variable constrained
	 * @param key3 is the identifier for the second variable constrained
	 * @param dim is the size of the constraint (p)
	 * @param mu is the gain for the factor
	 */
	NonlinearConstraint3(const Key1& key1, const Key2& key2, const Key3& key3,
			size_t dim, double mu = 1000.0) :
			Base(dim, mu), key1_(key1), key2_(key2), key3_(key3) {
		this->keys_.push_back(key1);
		this->keys_.push_back(key2);
		this->keys_.push_back(key3);
	}
	virtual ~NonlinearConstraint3() {}

	/* print */
	void print(const std::string& s = "") const {
		std::cout << "NonlinearConstraint3 " << s << std::endl;
		std::cout << "key1: " << (std::string) key1_ << std::endl;
		std::cout << "key2: " << (std::string) key2_ << std::endl;
		std::cout << "key3: " << (std::string) key3_ << std::endl;
		std::cout << "mu: " << this->mu_ << std::endl;
	}

	/** Check if two factors are equal. Note type is Factor and needs cast. */
	virtual bool equals(const NonlinearFactor<Values>& f, double tol = 1e-9) const {
		const This* p = dynamic_cast<const This*> (&f);
		if (p == NULL) return false;
		return Base::equals(*p, tol) && (key1_ == p->key1_) && (key2_ == p->key2_) && (key3_ == p->key3_);
	}

	/** error function g(x), switched depending on whether the constraint is active */
	inline Vector unwhitenedError(const Values& x) const {
		if (!active(x)) {
			return zero(this->dim());
		}
		const Key1& j1 = key1_;
		const Key2& j2 = key2_;
		const Key3& j3 = key3_;
		const X1& xj1 = x[j1];
		const X2& xj2 = x[j2];
		const X3& xj3 = x[j3];
		return evaluateError(xj1, xj2, xj3);
	}

	/** Linearize from config */
	boost::shared_ptr<GaussianFactor> linearize(const Values& c, const Ordering& ordering) const {
		if (!active(c)) {
			boost::shared_ptr<GaussianFactor> factor;
			return factor;
		}
		const Key1& j1 = key1_; const Key2& j2 = key2_; const Key3& j3 = key3_;
		const X1& x1 = c[j1]; const X2& x2 = c[j2]; const X3& x3 = c[j3];
		Matrix A1, A2, A3;
		Vector b = -1.0 * evaluateError(x1, x2, x3, A1, A2, A3);
		SharedDiagonal model = noiseModel::Constrained::All(this->dim());
		Index var1 = ordering[j1], var2 = ordering[j2], var3 = ordering[j3];

		// perform sorting
		if(var1 < var2 && var2 < var3)
			return GaussianFactor::shared_ptr(
					new GaussianFactor(var1, A1, var2, A2, var3, A3, b, model));
		else if(var2 < var1 && var1 < var3)
			return GaussianFactor::shared_ptr(
					new GaussianFactor(var2, A2, var1, A1, var3, A3, b, model));
		else if(var1 < var3 && var3 < var2)
			return GaussianFactor::shared_ptr(
					new GaussianFactor(var1, A1, var3, A3, var2, A2, b, model));
		else if(var2 < var3 && var3 < var1)
			return GaussianFactor::shared_ptr(
					new GaussianFactor(var2, A2, var3, A3, var1, A1, b, model));
		else if(var3 < var1 && var1 < var2)
			return GaussianFactor::shared_ptr(
					new GaussianFactor(var3, A3, var1, A1, var2, A2, b, model));
		else
			return GaussianFactor::shared_ptr(
					new GaussianFactor(var3, A3, var2, A2, var1, A1, b, model));
	}

	/** g(x) with optional derivative3  - does not depend on active */
	virtual Vector evaluateError(const X1& x1, const X2& x2, const X3& x3,
			boost::optional<Matrix&> H1 = boost::none,
			boost::optional<Matrix&> H2 = boost::none,
			boost::optional<Matrix&> H3 = boost::none) const = 0;

    /**
     * Create a symbolic factor using the given ordering to determine the
     * variable indices.
     */
    virtual Factor::shared_ptr symbolic(const Ordering& ordering) const {
      const Index var1 = ordering[key1_], var2 = ordering[key2_], var3 = ordering[key3_];
      if(var1 < var2 && var2 < var3)
        return Factor::shared_ptr(new Factor(ordering[key1_], ordering[key2_], ordering[key3_]));
      else if(var2 < var1 && var1 < var3)
        return Factor::shared_ptr(new Factor(ordering[key2_], ordering[key2_], ordering[key3_]));
      else if(var1 < var3 && var3 < var2)
        return Factor::shared_ptr(new Factor(ordering[key1_], ordering[key3_], ordering[key2_]));
      else if(var2 < var3 && var3 < var1)
        return Factor::shared_ptr(new Factor(ordering[key2_], ordering[key3_], ordering[key1_]));
      else if(var3 < var1 && var1 < var2)
        return Factor::shared_ptr(new Factor(ordering[key3_], ordering[key1_], ordering[key2_]));
      else
        return Factor::shared_ptr(new Factor(ordering[key3_], ordering[key2_], ordering[key1_]));
    }
};

/**
 * Ternary Equality constraint - simply forces the value of active() to true
 */
template <class Values, class Key1, class Key2, class Key3>
class NonlinearEqualityConstraint3 : public NonlinearConstraint3<Values, Key1, Key2, Key3> {

public:
	typedef typename Key1::Value X1;
	typedef typename Key2::Value X2;
	typedef typename Key3::Value X3;

protected:
	typedef NonlinearEqualityConstraint3<Values,Key1,Key2,Key3> This;
	typedef NonlinearConstraint3<Values,Key1,Key2,Key3> Base;

public:
	NonlinearEqualityConstraint3(const Key1& key1, const Key2& key2, const Key3& key3,
			size_t dim, double mu = 1000.0)
		: Base(key1, key2, key3, dim, mu) {}
	virtual ~NonlinearEqualityConstraint3() {}

	/** Always active, so fixed value for active() */
	virtual bool active(const Values& c) const { return true; }
};


/**
 * Simple unary equality constraint - fixes a value for a variable
 */
template<class Values, class Key>
class NonlinearEquality1 : public NonlinearEqualityConstraint1<Values, Key> {

public:
	typedef typename Key::Value X;

protected:
	typedef NonlinearEqualityConstraint1<Values, Key> Base;

	X value_; /// fixed value for variable

public:

	typedef boost::shared_ptr<NonlinearEquality1<Values, Key> > shared_ptr;

	NonlinearEquality1(const X& value, const Key& key1, double mu = 1000.0)
		: Base(key1, X::Dim(), mu), value_(value) {}
	virtual ~NonlinearEquality1() {}

	/** g(x) with optional derivative */
	Vector evaluateError(const X& x1, boost::optional<Matrix&> H1 = boost::none) const {
		const size_t p = X::Dim();
		if (H1) *H1 = eye(p);
		return value_.logmap(x1);
	}
};


/**
 * Simple binary equality constraint - this constraint forces two factors to
 * be the same.  This constraint requires the underlying type to a Lie type
 */
template<class Values, class Key>
class NonlinearEquality2 : public NonlinearEqualityConstraint2<Values, Key, Key> {
public:
	typedef typename Key::Value X;

protected:
	typedef NonlinearEqualityConstraint2<Values, Key, Key> Base;

public:

	typedef boost::shared_ptr<NonlinearEquality2<Values, Key> > shared_ptr;

	NonlinearEquality2(const Key& key1, const Key& key2, double mu = 1000.0)
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
};

}
