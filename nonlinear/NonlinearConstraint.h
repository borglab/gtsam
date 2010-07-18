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

	double mu_; // gain for quadratic merit function

public:

	/** Constructor - sets the cost function and the lagrange multipliers
	 * @param dim is the dimension of the factor
	 * @param mu is the gain used at error evaluation (forced to be positive)
	 */
	NonlinearConstraint(size_t dim, double mu = 1000.0);

	/** returns the gain mu */
	double mu() const { return mu_; }

	/** Print */
	virtual void print(const std::string& s = "") const=0;

	/** Check if two factors are equal */
	virtual bool equals(const Factor<Config>& f, double tol=1e-9) const=0;

	/** error function - returns the quadratic merit function */
	virtual double error(const Config& c) const;

	/**
	 * Linearizes around a given config
	 * @param config is the configuration
	 * @return a combined linear factor containing both the constraint and the constraint factor
	 */
	virtual boost::shared_ptr<GaussianFactor> linearize(const Config& c) const=0;
};


/**
 * A unary constraint with arbitrary cost and jacobian functions
 * This is an example class designed for easy testing, but real uses should probably
 * subclass NonlinearConstraint and implement virtual functions directly
 */
template <class Config, class Key, class X>
class NonlinearConstraint1 : public NonlinearConstraint<Config> {

private:

	/**
	 * Calculates the jacobian of the constraint function
	 * returns a pxn matrix
	 * Use boost.bind to create the function object
	 * @param config to use for linearization
	 * @return the jacobian of the constraint in terms of key
	 */
	boost::function<Matrix(const Config& config)> G_;

	/** calculates the constraint function of the current config
	 * If the value is zero, the constraint is not active
	 * Use boost.bind to create the function object
	 * @param config is a configuration of all the variables
	 * @return the cost for each of p constraints, arranged in a vector
	 */
	boost::function<Vector(const Config& config)> g_;

	/** key for the constrained variable */
	Key key_;

public:

	/**
	 * Basic constructor
	 * @param key is the identifier for the variable constrained
	 * @param G gives the jacobian of the constraint function
	 * @param g is the constraint function
	 * @param dim is the size of the constraint (p)
	 * @param mu is the gain for the factor
	 */
	NonlinearConstraint1(
			Vector (*g)(const Config& config),
			const Key& key,
			Matrix (*G)(const Config& config),
			size_t dim,
			double mu = 1000.0);

	/**
	 * Basic constructor with boost function pointers
	 * @param key is the identifier for the variable constrained
	 * @param G gives the jacobian of the constraint function
	 * @param g is the constraint function as a boost function pointer
	 * @param dim is the size of the constraint (p)
	 * @param mu is the gain for the factor
	 */
	NonlinearConstraint1(
			boost::function<Vector(const Config& config)> g,
			const Key& key,
			boost::function<Matrix(const Config& config)> G,
			size_t dim,
			double mu = 1000.0);

	/** Print */
	void print(const std::string& s = "") const;

	/** Check if two factors are equal */
	bool equals(const Factor<Config>& f, double tol=1e-9) const;

	/** Error function */
	virtual inline Vector unwhitenedError(const Config& c) const { return g_(c); }

	/** Linearize from config */
	virtual boost::shared_ptr<GaussianFactor> linearize(const Config& c) const;
};

/**
 * A binary constraint with arbitrary cost and jacobian functions
 */
template <class Config, class Key1, class X1, class Key2, class X2>
class NonlinearConstraint2 : public NonlinearConstraint<Config> {

private:

	/**
	 * Calculates the jacobians of the constraint function in terms of
	 * the first and second variables
	 * returns a pxn matrix
	 * @param config to use for linearization
	 * @return the jacobian of the constraint in terms of key
	 */
	boost::function<Matrix(const Config& config)> G1_;
	boost::function<Matrix(const Config& config)> G2_;

	/** calculates the constraint function of the current config
	 * If the value is zero, the constraint is not active
	 * Use boost.bind to create the function object
	 * @param config is a configuration of all the variables
	 * @return the cost for each of p constraints, arranged in a vector
	 */
	boost::function<Vector(const Config& config)> g_;

	/** keys for the constrained variables */
	Key1 key1_;
	Key2 key2_;

public:

	/**
	 * Basic constructor
	 * @param key is the identifier for the variable constrained
	 * @param G gives the jacobian of the constraint function
	 * @param g is the constraint function
	 * @param dim is the size of the constraint (p)
	 * @param mu is the gain for the factor
	 */
	NonlinearConstraint2(
			Vector (*g)(const Config& config),
			const Key1& key1,
			Matrix (*G1)(const Config& config),
			const Key2& key2,
			Matrix (*G2)(const Config& config),
			size_t dim,
			double mu = 1000.0);

	/**
	 * Basic constructor with direct function objects
	 * Use boost.bind to construct the function objects
	 * @param key is the identifier for the variable constrained
	 * @param G gives the jacobian of the constraint function
	 * @param g is the constraint function
	 * @param dim is the size of the constraint (p)
	 * @param mu is the gain for the factor
	 */
	NonlinearConstraint2(
			boost::function<Vector(const Config& config)> g,
			const Key1& key1,
			boost::function<Matrix(const Config& config)> G1,
			const Key2& key2,
			boost::function<Matrix(const Config& config)> G2,
			size_t dim,
			double mu = 1000.0);

	/** Print */
	void print(const std::string& s = "") const;

	/** Check if two factors are equal */
	bool equals(const Factor<Config>& f, double tol=1e-9) const;

	/** Error function */
	virtual inline Vector unwhitenedError(const Config& c) const { return g_(c); }

	/** Linearize from config */
	virtual boost::shared_ptr<GaussianFactor> linearize(const Config& c) const;
};

}
