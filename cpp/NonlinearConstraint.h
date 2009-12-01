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
	std::string lagrange_key_;

	/** number of lagrange multipliers */
	size_t p_;

	/** type of constraint */
	bool isEquality_;

	/** calculates the constraint function of the current config
	 * If the value is zero, the constraint is not active
	 * Use boost.bind to create the function object
	 * @param config is a configuration of all the variables
	 * @param keys is the set of keys - assumed that the function knows how to use
	 * @return the cost for each of p constraints, arranged in a vector
	 */
	boost::function<Vector(const Config& config, const std::list<std::string>& keys)> g_;

public:

	/** Constructor - sets the cost function and the lagrange multipliers
	 * @param lagrange_key is the label for the associated lagrange multipliers
	 * @param dim_lagrange is the number of associated constraints
	 * @param isEquality is true if the constraint is an equality constraint
	 * @param g is the cost function for the constraint
	 */
	NonlinearConstraint(const std::string& lagrange_key,
						size_t dim_lagrange,
						Vector (*g)(const Config& config, const std::list<std::string>& keys),
						bool isEquality=true);

	/** Constructor - sets a more general cost function using boost::bind directly
	 * @param lagrange_key is the label for the associated lagrange multipliers
	 * @param dim_lagrange is the number of associated constraints
	 * @param g is the cost function for the constraint
	 * @param isEquality is true if the constraint is an equality constraint
	 */
	NonlinearConstraint(const std::string& lagrange_key,
						size_t dim_lagrange,
						boost::function<Vector(const Config& config, const std::list<std::string>& keys)> g,
						bool isEquality=true);

	/** returns the key used for the Lagrange multipliers */
	std::string lagrangeKey() const { return lagrange_key_; }

	/** returns the number of lagrange multipliers */
	size_t nrConstraints() const { return p_; }

	/** returns the type of constraint */
	bool isEquality() const { return isEquality_; }

	/** Print */
	virtual void print(const std::string& s = "") const =0;

	/** Check if two factors are equal */
	virtual bool equals(const Factor<Config>& f, double tol=1e-9) const=0;

	/** error function - returns the result of the constraint function */
	inline Vector error_vector(const Config& c) const { return g_(c, this->keys()); }

	/**
	 * Determines whether the constraint is active given a particular configuration
	 * @param config is the input to the g(x) function
	 * @return true if constraint needs to be linearized
	 */
	bool active(const Config& config) const;

	/**
	 * Linearize using a real Config and a VectorConfig of Lagrange multipliers
	 * Returns the two separate Gaussian factors to solve
	 * @param config is the real Config of the real variables
	 * @param lagrange is the VectorConfig of lagrange multipliers
	 * @return a pair GaussianFactor (probabilistic) and GaussianFactor (constraint)
	 */
	virtual std::pair<GaussianFactor::shared_ptr, GaussianFactor::shared_ptr>
	linearize(const Config& config, const VectorConfig& lagrange) const=0;

	/**
	 * linearize with only Config, which is not currently implemented
	 * This will be implemented later for other constrained optimization
	 * algorithms
	 */
	virtual boost::shared_ptr<GaussianFactor> linearize(const Config& c) const {
		throw std::invalid_argument("No current constraint linearization for a single Config!");
	}
};


/**
 * A unary constraint with arbitrary cost and gradient functions
 */
template <class Config>
class NonlinearConstraint1 : public NonlinearConstraint<Config> {

private:

	/**
	 * Calculates the gradient of the constraint function
	 * returns a pxn matrix
	 * Use boost.bind to create the function object
	 * @param config to use for linearization
	 * @param key of selected variable
	 * @return the jacobian of the constraint in terms of key
	 */
	boost::function<Matrix(const Config& config, const std::list<std::string>& keys)> gradG_;

	/** key for the constrained variable */
	std::string key_;

public:

	/**
	 * Basic constructor
	 * @param key is the identifier for the variable constrained
	 * @param gradG gives the gradient of the constraint function
	 * @param g is the constraint function
	 * @param dim_constraint is the size of the constraint (p)
	 * @param lagrange_key is the identifier for the lagrange multiplier
	 * @param isEquality is true if the constraint is an equality constraint
	 */
	NonlinearConstraint1(
			const std::string& key,
			Matrix (*gradG)(const Config& config, const std::list<std::string>& keys),
			Vector (*g)(const Config& config, const std::list<std::string>& keys),
			size_t dim_constraint,
			const std::string& lagrange_key="",
			bool isEquality=true);

	/**
	 * Basic constructor with boost function pointers
	 * @param key is the identifier for the variable constrained
	 * @param gradG gives the gradient of the constraint function
	 * @param g is the constraint function as a boost function pointer
	 * @param dim_constraint is the size of the constraint (p)
	 * @param lagrange_key is the identifier for the lagrange multiplier
	 * @param isEquality is true if the constraint is an equality constraint
	 */
	NonlinearConstraint1(
			const std::string& key,
			boost::function<Matrix(const Config& config, const std::list<std::string>& keys)> gradG,
			boost::function<Vector(const Config& config, const std::list<std::string>& keys)> g,
			size_t dim_constraint,
			const std::string& lagrange_key="",
			bool isEquality=true);

	/** Print */
	void print(const std::string& s = "") const;

	/** Check if two factors are equal */
	bool equals(const Factor<Config>& f, double tol=1e-9) const;

	/**
	 * Linearize using a real Config and a VectorConfig of Lagrange multipliers
	 * Returns the two separate Gaussian factors to solve
	 * @param config is the real Config of the real variables
	 * @param lagrange is the VectorConfig of lagrange multipliers
	 * @return a pair GaussianFactor (probabilistic) and GaussianFactor (constraint)
	 */
	std::pair<GaussianFactor::shared_ptr, GaussianFactor::shared_ptr>
	linearize(const Config& config, const VectorConfig& lagrange) const;
};

/**
 * A binary constraint with arbitrary cost and gradient functions
 */
template <class Config>
class NonlinearConstraint2 : public NonlinearConstraint<Config> {

private:

	/**
	 * Calculates the gradients of the constraint function in terms of
	 * the first and second variables
	 * returns a pxn matrix
	 * @param config to use for linearization
	 * @param key of selected variable
	 * @return the jacobian of the constraint in terms of key
	 */
	boost::function<Matrix(const Config& config, const std::list<std::string>& keys)> gradG1_;
	boost::function<Matrix(const Config& config, const std::list<std::string>& keys)> gradG2_;

	/** keys for the constrained variables */
	std::string key1_;
	std::string key2_;

public:

	/**
	 * Basic constructor
	 * @param key is the identifier for the variable constrained
	 * @param gradG gives the gradient of the constraint function
	 * @param g is the constraint function
	 * @param dim_constraint is the size of the constraint (p)
	 * @param lagrange_key is the identifier for the lagrange multiplier
	 * @param isEquality is true if the constraint is an equality constraint
	 */
	NonlinearConstraint2(
			const std::string& key1,
			Matrix (*gradG1)(const Config& config, const std::list<std::string>& keys),
			const std::string& key2,
			Matrix (*gradG2)(const Config& config, const std::list<std::string>& keys),
			Vector (*g)(const Config& config, const std::list<std::string>& keys),
			size_t dim_constraint,
			const std::string& lagrange_key="",
			bool isEquality=true);

	/**
	 * Basic constructor with direct function objects
	 * Use boost.bind to construct the function objects
	 * @param key is the identifier for the variable constrained
	 * @param gradG gives the gradient of the constraint function
	 * @param g is the constraint function
	 * @param dim_constraint is the size of the constraint (p)
	 * @param lagrange_key is the identifier for the lagrange multiplier
	 * @param isEquality is true if the constraint is an equality constraint
	 */
	NonlinearConstraint2(
			const std::string& key1,
			boost::function<Matrix(const Config& config, const std::list<std::string>& keys)> gradG1,
			const std::string& key2,
			boost::function<Matrix(const Config& config, const std::list<std::string>& keys)> gradG2,
			boost::function<Vector(const Config& config, const std::list<std::string>& keys)> g,
			size_t dim_constraint,
			const std::string& lagrange_key="",
			bool isEquality=true);

	/** Print */
	void print(const std::string& s = "") const;

	/** Check if two factors are equal */
	bool equals(const Factor<Config>& f, double tol=1e-9) const;

	/**
	 * Linearize using a real Config and a VectorConfig of Lagrange multipliers
	 * Returns the two separate Gaussian factors to solve
	 * @param config is the real Config of the real variables
	 * @param lagrange is the VectorConfig of lagrange multipliers
	 * @return a pair GaussianFactor (probabilistic) and GaussianFactor (constraint)
	 */
	std::pair<GaussianFactor::shared_ptr, GaussianFactor::shared_ptr>
	linearize(const Config& config, const VectorConfig& lagrange) const;
};

}
