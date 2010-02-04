/*
 * @file NonlinearConstraint.h
 * @brief Implements nonlinear constraints that can be linearized and
 * inserted into an existing nonlinear graph and solved via SQP
 * @author Alex Cunningham
 */

#pragma once

#include <map>
#include <boost/function.hpp>
#include "NonlinearFactor.h"

namespace gtsam {

/** Typedef for Lagrange key type - must be present in factors and config */
typedef TypedSymbol<Vector, 'L'> LagrangeKey;

/**
 * Base class for nonlinear constraints
 * This allows for both equality and inequality constraints,
 * where equality constraints are active all the time (even slightly
 * nonzero constraint functions will still be active - inequality
 * constraints should be sure to force to actual zero)
 *
 * The measurement z in the underlying NonlinearFactor is the
 * set of Lagrange multipliers.
 */
template <class Config>
class NonlinearConstraint : public NonlinearFactor<Config> {

protected:

	/** key for the lagrange multipliers */
	LagrangeKey lagrange_key_;

	/** number of lagrange multipliers */
	size_t p_;

	/** type of constraint */
	bool isEquality_;

	/** calculates the constraint function of the current config
	 * If the value is zero, the constraint is not active
	 * Use boost.bind to create the function object
	 * @param config is a configuration of all the variables
	 * @return the cost for each of p constraints, arranged in a vector
	 */
	boost::function<Vector(const Config& config)> g_;

public:

	/** Constructor - sets the cost function and the lagrange multipliers
	 * @param lagrange_key is the label for the associated lagrange multipliers
	 * @param dim_lagrange is the number of associated constraints
	 * @param isEquality is true if the constraint is an equality constraint
	 * @param g is the cost function for the constraint
	 */
	NonlinearConstraint(const LagrangeKey& lagrange_key,
						size_t dim_lagrange,
						Vector (*g)(const Config& config),
						bool isEquality=true);

	/** Constructor - sets a more general cost function using boost::bind directly
	 * @param lagrange_key is the label for the associated lagrange multipliers
	 * @param dim_lagrange is the number of associated constraints
	 * @param g is the cost function for the constraint
	 * @param isEquality is true if the constraint is an equality constraint
	 */
	NonlinearConstraint(const LagrangeKey& lagrange_key,
						size_t dim_lagrange,
						boost::function<Vector(const Config& config)> g,
						bool isEquality=true);

	/** returns the key used for the Lagrange multipliers */
	LagrangeKey lagrangeKey() const { return lagrange_key_; }

	/** returns the number of lagrange multipliers */
	size_t nrConstraints() const { return p_; }

	/** returns the type of constraint */
	bool isEquality() const { return isEquality_; }

	/** Print */
	virtual void print(const std::string& s = "") const =0;

	/** Check if two factors are equal */
	virtual bool equals(const Factor<Config>& f, double tol=1e-9) const=0;

	/** error function - returns the result of the constraint function */
	inline Vector unwhitenedError(const Config& c) const { return g_(c); }

	/**
	 * Determines whether the constraint is active given a particular configuration
	 * @param config is the input to the g(x) function
	 * @return true if constraint needs to be linearized
	 */
	bool active(const Config& config) const;

	/**
	 * Real linearize, given a config that includes Lagrange multipliers
	 * @param config is the configuration (with lagrange multipliers)
	 * @return a combined linear factor containing both the constraint and the constraint factor
	 */
	virtual boost::shared_ptr<GaussianFactor> linearize(const Config& c) const=0;
};


/**
 * A unary constraint with arbitrary cost and jacobian functions
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

	/** key for the constrained variable */
	Key key_;

public:

	/**
	 * Basic constructor
	 * @param key is the identifier for the variable constrained
	 * @param G gives the jacobian of the constraint function
	 * @param g is the constraint function
	 * @param dim_constraint is the size of the constraint (p)
	 * @param lagrange_key is the identifier for the lagrange multiplier
	 * @param isEquality is true if the constraint is an equality constraint
	 */
	NonlinearConstraint1(
			Vector (*g)(const Config& config),
			const Key& key,
			Matrix (*G)(const Config& config),
			size_t dim_constraint,
			const LagrangeKey& lagrange_key,
			bool isEquality=true);

	/**
	 * Basic constructor with boost function pointers
	 * @param key is the identifier for the variable constrained
	 * @param G gives the jacobian of the constraint function
	 * @param g is the constraint function as a boost function pointer
	 * @param dim_constraint is the size of the constraint (p)
	 * @param lagrange_key is the identifier for the lagrange multiplier
	 * @param isEquality is true if the constraint is an equality constraint
	 */
	NonlinearConstraint1(
			boost::function<Vector(const Config& config)> g,
			const Key& key,
			boost::function<Matrix(const Config& config)> G,
			size_t dim_constraint,
			const LagrangeKey& lagrange_key,
			bool isEquality=true);

	/** Print */
	void print(const std::string& s = "") const;

	/** Check if two factors are equal */
	bool equals(const Factor<Config>& f, double tol=1e-9) const;

	/**
	 * Linearize from config - must have Lagrange multipliers
	 */
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

	/** keys for the constrained variables */
	Key1 key1_;
	Key2 key2_;

public:

	/**
	 * Basic constructor
	 * @param key is the identifier for the variable constrained
	 * @param G gives the jacobian of the constraint function
	 * @param g is the constraint function
	 * @param dim_constraint is the size of the constraint (p)
	 * @param lagrange_key is the identifier for the lagrange multiplier
	 * @param isEquality is true if the constraint is an equality constraint
	 */
	NonlinearConstraint2(
			Vector (*g)(const Config& config),
			const Key1& key1,
			Matrix (*G1)(const Config& config),
			const Key2& key2,
			Matrix (*G2)(const Config& config),
			size_t dim_constraint,
			const LagrangeKey& lagrange_key,
			bool isEquality=true);

	/**
	 * Basic constructor with direct function objects
	 * Use boost.bind to construct the function objects
	 * @param key is the identifier for the variable constrained
	 * @param G gives the jacobian of the constraint function
	 * @param g is the constraint function
	 * @param dim_constraint is the size of the constraint (p)
	 * @param lagrange_key is the identifier for the lagrange multiplier
	 * @param isEquality is true if the constraint is an equality constraint
	 */
	NonlinearConstraint2(
			boost::function<Vector(const Config& config)> g,
			const Key1& key1,
			boost::function<Matrix(const Config& config)> G1,
			const Key2& key2,
			boost::function<Matrix(const Config& config)> G2,
			size_t dim_constraint,
			const LagrangeKey& lagrange_key,
			bool isEquality=true);

	/** Print */
	void print(const std::string& s = "") const;

	/** Check if two factors are equal */
	bool equals(const Factor<Config>& f, double tol=1e-9) const;

	/**
	 * Linearize from config - must have Lagrange multipliers
	 */
	virtual boost::shared_ptr<GaussianFactor> linearize(const Config& c) const;
};

}
