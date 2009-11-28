/**
 * @file SQPOptimizer.h
 * @brief Interface for a generic SQP-based nonlinear optimization engine
 * @author Alex Cunningham
 */

#pragma once

#include "Ordering.h"
#include "VectorConfig.h"

namespace gtsam {

/**
 * This class is an engine for performing SQP-based optimization
 * It stores a graph, a config, and needs a specific ordering, and
 * then will perform optimization iterations in a functional way.
 */
template<class FactorGraph, class Config>
class SQPOptimizer {

public:
	// verbosity level
	typedef enum {
		SILENT,
		FULL
	} Verbosity;

	// useful for storing configurations
	typedef boost::shared_ptr<const Config> shared_config;
	typedef boost::shared_ptr<VectorConfig> shared_vconfig;

private:
	// keep const references to the graph and initial ordering
	const FactorGraph* graph_;
	const Ordering* ordering_;

	// keep configurations
	shared_config config_;
	shared_vconfig lagrange_config_;

	// keep a configuration that has been updated to include the lagrange multipliers
	Ordering full_ordering_;


	// keep a set of errors for the overall system and just the constraints
	double error_;
	double constraint_error_;

public:
	/**
	 * Standard external constructor
	 * @param graph is the nonlinear graph to optimize
	 * @param ordering is the elimination ordering to use
	 * @param config is the initial configuration for the real variables
	 */
	SQPOptimizer(const FactorGraph& graph, const Ordering& ordering,
			shared_config config);

	/**
	 * Constructor that includes a lagrange initialization.  Primarily
	 * for internal iterations, but if the user has an idea of what a good
	 * set of lagrange multipliers is, they can specify them, assuming that
	 * the naming convention is the same as the internal system.
	 * @param graph is the nonlinear graph to optimize
	 * @param ordering is the elimination ordering to use
	 * @param config is the initial configuration for the real variables
	 * @param lagrange is the configuration of lagrange multipliers
	 */
	SQPOptimizer(const FactorGraph& graph, const Ordering& ordering,
			shared_config config, shared_vconfig lagrange);

	/// Access functions
	const FactorGraph* graph() const { return graph_; }
	const Ordering* ordering() const { return ordering_; }
	shared_config config() const { return config_; }
	shared_vconfig configLagrange() const { return lagrange_config_; }
	double error() const { return error_; }

	/**
	 * Primary optimization iteration, updates the configs
	 * @return a new optimization object with updated values
	 */
	SQPOptimizer<FactorGraph, Config> iterate(Verbosity verbosity=SILENT) const;

	/**
	 * Iterates recursively until converence occurs
	 * @param relThresh minimum change in error between iterations
	 * @param absThresh  minimum error necessary to converge
	 * @param constraintThresh  minimum constraint error to be feasible
	 * @param maxIterations is the maximum number of iterations
	 * @param verbosity controls output print statements
	 * @return a new optimization object with final values
	 */
	SQPOptimizer<FactorGraph, Config>
	iterateSolve(double relThresh, double absThresh, double constraintThresh,
			size_t maxIterations = 10, Verbosity verbosity=SILENT) const;

	/**
	 * Checks whether convergence has occurred, and returns true if
	 * the solution will not get better, based on the previous error conditions.
	 * @param full_error is the error all the factors and constraints
	 * @param constraint_error is the error of just the constraints
	 * @param relThresh is the relative threshold between
	 * @return true if the problem has converged
	 */
	bool checkConvergence(double relThresh, double absThresh,
			double constraintThresh, double full_error, double constraint_error) const;

	/** Standard print function with optional name */
	void print(const std::string& s);
};

}

