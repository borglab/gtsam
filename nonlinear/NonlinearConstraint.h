/*
 * @file NonlinearConstraint.h
 * @brief Implements nonlinear constraints that can be linearized using
 * direct linearization and solving through a quadratic merit function
 * @author Alex Cunningham
 */

#pragma once

#include <map>
#include <boost/function.hpp>
#include "NonlinearFactor.h"

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
template <class Config>
class NonlinearConstraint : public NonlinearFactor<Config> {

protected:
	typedef NonlinearConstraint<Config> This;
	typedef NonlinearFactor<Config> Base;
	double mu_; // gain for quadratic merit function

public:

	/** Constructor - sets the cost function and the lagrange multipliers
	 * @param dim is the dimension of the factor
	 * @param mu is the gain used at error evaluation (forced to be positive)
	 */
	NonlinearConstraint(size_t dim, double mu = 1000.0):
		Base(noiseModel::Constrained::All(dim)), mu_(fabs(mu)) {}

	/** returns the gain mu */
	double mu() const { return mu_; }

	/** Print */
	virtual void print(const std::string& s = "") const=0;

	/** Check if two factors are equal */
	virtual bool equals(const Factor<Config>& f, double tol=1e-9) const {
		const This* p = dynamic_cast<const This*> (&f);
		if (p == NULL) return false;
		return Base::equals(*p, tol) && (mu_ == p->mu_);
	}

	/** error function - returns the quadratic merit function */
	virtual double error(const Config& c) const  {
		const Vector error_vector = unwhitenedError(c);
		if (active(c))
			return mu_ * inner_prod(error_vector, error_vector);
		else return 0.0;
	}

	/** Raw error vector function g(x) */
	virtual Vector unwhitenedError(const Config& c) const = 0;

	/**
	 * active set check, defines what type of constraint this is
	 * By default, the constraint is always active, so it is an equality constraint
	 * @return true if the constraint is active
	 */
	virtual bool active(const Config& c) const { return true; }

	/**
	 * Linearizes around a given config
	 * @param config is the configuration
	 * @return a combined linear factor containing both the constraint and the constraint factor
	 */
	virtual boost::shared_ptr<GaussianFactor> linearize(const Config& c) const=0;
};


/**
 * A unary constraint that defaults to an equality constraint
 */
template <class Config, class Key, class X>
class NonlinearConstraint1 : public NonlinearConstraint<Config> {

protected:
	typedef NonlinearConstraint1<Config,Key,X> This;
	typedef NonlinearConstraint<Config> Base;

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

	/* print */
	void print(const std::string& s = "") const {
		std::cout << "NonlinearConstraint1 " << s << std::endl;
		std::cout << "key: " << (std::string) key_ << std::endl;
		std::cout << "mu: " << this->mu_ << std::endl;
	}

	/** Check if two factors are equal. Note type is Factor and needs cast. */
	virtual bool equals(const Factor<Config>& f, double tol = 1e-9) const {
		const This* p = dynamic_cast<const This*> (&f);
		if (p == NULL) return false;
		return Base::equals(*p, tol) && (key_ == p->key_);
	}

	/** error function g(x) */
	inline Vector unwhitenedError(const Config& x) const {
		const Key& j = key_;
		const X& xj = x[j];
		return evaluateError(xj);
	}

	/** Linearize from config */
	boost::shared_ptr<GaussianFactor> linearize(const Config& c) const {
		if (!active(c)) {
			boost::shared_ptr<GaussianFactor> factor;
			return factor;
		}
		const Key& j = key_;
		const X& x = c[j];
		Matrix grad;
		Vector g = -1.0 * evaluateError(x, grad);
		SharedDiagonal model = noiseModel::Constrained::All(this->dim());
		return GaussianFactor::shared_ptr(new GaussianFactor(this->key_, grad, g, model));
	}

	/** g(x) with optional derivative */
	virtual Vector evaluateError(const X& x, boost::optional<Matrix&> H =
			boost::none) const = 0;
};

/**
 * A binary constraint with arbitrary cost and jacobian functions
 */
template <class Config, class Key1, class X1, class Key2, class X2>
class NonlinearConstraint2 : public NonlinearConstraint<Config> {

protected:
	typedef NonlinearConstraint2<Config,Key1,X1,Key2,X2> This;
	typedef NonlinearConstraint<Config> Base;

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

	/* print */
	void print(const std::string& s = "") const {
		std::cout << "NonlinearConstraint2 " << s << std::endl;
		std::cout << "key1: " << (std::string) key1_ << std::endl;
		std::cout << "key2: " << (std::string) key2_ << std::endl;
		std::cout << "mu: " << this->mu_ << std::endl;
	}

	/** Check if two factors are equal. Note type is Factor and needs cast. */
	virtual bool equals(const Factor<Config>& f, double tol = 1e-9) const {
		const This* p = dynamic_cast<const This*> (&f);
		if (p == NULL) return false;
		return Base::equals(*p, tol) && (key1_ == p->key1_) && (key2_ == p->key2_);
	}

	/** error function g(x) */
	inline Vector unwhitenedError(const Config& x) const {
		const Key1& j1 = key1_;
		const Key2& j2 = key2_;
		const X1& xj1 = x[j1];
		const X2& xj2 = x[j2];
		return evaluateError(xj1, xj2);
	}

	/** Linearize from config */
	boost::shared_ptr<GaussianFactor> linearize(const Config& c) const {
		if (!active(c)) {
			boost::shared_ptr<GaussianFactor> factor;
			return factor;
		}
		const Key1& j1 = key1_; const Key2& j2 = key2_;
		const X1& x1 = c[j1]; const X2& x2 = c[j2];
		Matrix grad1, grad2;
		Vector g = -1.0 * evaluateError(x1, x2, grad1, grad2);
		SharedDiagonal model = noiseModel::Constrained::All(this->dim());
		return GaussianFactor::shared_ptr(new GaussianFactor(j1, grad1, j2, grad2, g, model));
	}

	/** g(x) with optional derivative2 */
	virtual Vector evaluateError(const X1& x1, const X2& x2,
			boost::optional<Matrix&> H1 = boost::none,
			boost::optional<Matrix&> H2 = boost::none) const = 0;
};

/**
 * Simple binary equality constraint - this constraint forces two factors to
 * be the same.  This constraint requires the underlying type to a Lie type
 */
template<class Config, class Key, class X>
class NonlinearEquality2 : public NonlinearConstraint2<Config, Key, X, Key, X> {
protected:
	typedef NonlinearConstraint2<Config, Key, X, Key, X> Base;

public:

	typedef boost::shared_ptr<NonlinearEquality2<Config, Key, X> > shared_ptr;

	NonlinearEquality2(const Key& key1, const Key& key2, double mu = 1000.0)
		: Base(key1, key2, X::dim(), mu) {}

	/** g(x) with optional derivative2 */
	Vector evaluateError(const X& x1, const X& x2,
			boost::optional<Matrix&> H1 = boost::none,
			boost::optional<Matrix&> H2 = boost::none) const {
		const size_t p = X::dim();
		if (H1) *H1 = -eye(p);
		if (H2) *H2 = eye(p);
		return logmap(x1, x2);
	}
};

}
