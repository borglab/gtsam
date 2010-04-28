/**
 * NonlinearOptimizer-inl.h
 * This is a template definition file, include it where needed (only!)
 * so that the appropriate code is generated and link errors avoided.
 * @brief: Encapsulates nonlinear optimization state
 * @Author: Frank Dellaert
 * Created on: Sep 7, 2009
 */

#pragma once

#include <iostream>
#include <boost/tuple/tuple.hpp>
#include "NonlinearOptimizer.h"

#define INSTANTIATE_NONLINEAR_OPTIMIZER(G,C) \
  template class NonlinearOptimizer<G,C>;

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	inline bool check_convergence(double relativeErrorTreshold,
			double absoluteErrorTreshold, double currentError, double newError,
			int verbosity) {
		// check if diverges
		double absoluteDecrease = currentError - newError;
		if (verbosity >= 2)
			cout << "absoluteDecrease: " << absoluteDecrease << endl;
//		if (absoluteDecrease < 0)
//			throw overflow_error(
//					"NonlinearFactorGraph::optimize: error increased, diverges.");

		// calculate relative error decrease and update currentError
		double relativeDecrease = absoluteDecrease / currentError;
		if (verbosity >= 2)
			cout << "relativeDecrease: " << relativeDecrease << endl;
		bool converged = (relativeDecrease < relativeErrorTreshold)
				|| (absoluteDecrease < absoluteErrorTreshold);
		if (verbosity >= 1 && converged)
			cout << "converged" << endl;
		return converged;
	}

	/* ************************************************************************* */
	template<class G, class C, class L, class S, class W>
	NonlinearOptimizer<G, C, L, S, W>::NonlinearOptimizer(shared_graph graph,
			shared_config config, shared_solver solver, double lambda) :
		graph_(graph), config_(config), lambda_(lambda), solver_(solver) {
		if (!graph) throw std::invalid_argument(
				"NonlinearOptimizer constructor: graph = NULL");
		if (!config) throw std::invalid_argument(
				"NonlinearOptimizer constructor: config = NULL");
		if (!solver) throw std::invalid_argument(
				"NonlinearOptimizer constructor: solver = NULL");
		error_ = graph->error(*config);
	}

	/* ************************************************************************* */
	// linearize and optimize
	/* ************************************************************************* */
	template<class G, class C, class L, class S, class W>
	VectorConfig NonlinearOptimizer<G, C, L, S, W>::linearizeAndOptimizeForDelta() const {
		boost::shared_ptr<L> linearized = solver_->linearize(*graph_, *config_);
		return solver_->optimize(*linearized);
	}

	/* ************************************************************************* */
	// One iteration of Gauss Newton
	/* ************************************************************************* */
	template<class G, class C, class L, class S, class W>
	NonlinearOptimizer<G, C, L, S, W> NonlinearOptimizer<G, C, L, S, W>::iterate(
			verbosityLevel verbosity) const {
		// linearize and optimize
		VectorConfig delta = linearizeAndOptimizeForDelta();

		// maybe show output
		if (verbosity >= DELTA)
			delta.print("delta");

		// take old config and update it
		shared_config newConfig(new C(expmap(*config_,delta)));

		// maybe show output
		if (verbosity >= CONFIG)
			newConfig->print("newConfig");

		NonlinearOptimizer newOptimizer = NonlinearOptimizer(graph_, newConfig, solver_, lambda_);

		if (verbosity >= ERROR)
			cout << "error: " << newOptimizer.error_ << endl;

		return newOptimizer;
	}

	/* ************************************************************************* */
	template<class G, class C, class L, class S, class W>
	NonlinearOptimizer<G, C, L, S, W> NonlinearOptimizer<G, C, L, S, W>::gaussNewton(
			double relativeThreshold, double absoluteThreshold,
			verbosityLevel verbosity, int maxIterations) const {
		static W writer(error_);

		// check if we're already close enough
		if (error_ < absoluteThreshold) {
			if (verbosity >= ERROR) cout << "Exiting, as error = " << error_
					<< " < absoluteThreshold (" << absoluteThreshold << ")" << endl;
			return *this;
		}

		// linearize, solve, update
		NonlinearOptimizer next = iterate(verbosity);

		writer.write(next.error_);

		// check convergence
		bool converged = gtsam::check_convergence(relativeThreshold,
				absoluteThreshold, error_, next.error_, verbosity);

				// return converged state or iterate
		if (converged)
			return next;
		else
			return next.gaussNewton(relativeThreshold, absoluteThreshold, verbosity);
	}

	/* ************************************************************************* */
	// Recursively try to do tempered Gauss-Newton steps until we succeed.
	// Form damped system with given lambda, and return a new, more optimistic
	// optimizer if error decreased or recurse with a larger lambda if not.
	// TODO: in theory we can't infinitely recurse, but maybe we should put a max.
	/* ************************************************************************* */
	template<class G, class C, class L, class S, class W>
	NonlinearOptimizer<G, C, L, S, W> NonlinearOptimizer<G, C, L, S, W>::try_lambda(
			const L& linear, verbosityLevel verbosity, double factor, LambdaMode lambdaMode) const {

		if (verbosity >= TRYLAMBDA)
			cout << "trying lambda = " << lambda_ << endl;

		// add prior-factors
		L damped = linear.add_priors(1.0/sqrt(lambda_));
		if (verbosity >= DAMPED)
			damped.print("damped");

		// solve
		VectorConfig delta = solver_->optimize(damped);
		if (verbosity >= TRYDELTA)
			delta.print("delta");

		// update config
		shared_config newConfig(new C(expmap(*config_,delta))); // TODO: updateConfig
//		if (verbosity >= TRYCONFIG)
//			newConfig->print("config");

		// create new optimization state with more adventurous lambda
		NonlinearOptimizer next(graph_, newConfig, solver_, lambda_ / factor);
		if (verbosity >= TRYLAMBDA) cout << "next error = " << next.error_ << endl;

		if(lambdaMode >= CAUTIOUS) {
			throw runtime_error("CAUTIOUS mode not working yet, please use BOUNDED.");
		}

		if(next.error_ <= error_) {

			// If we're cautious, see if the current lambda is better
			// todo:  include stopping criterion here?
			if(lambdaMode == CAUTIOUS) {
				NonlinearOptimizer sameLambda(graph_, newConfig, solver_, lambda_);
				if(sameLambda.error_ <= next.error_)
					return sameLambda;
			}

			// Either we're not cautious, or we are but the adventerous lambda is better than the same one.
			return next;
		} else if (lambda_ > 1e+10) // if lambda gets too big, something is broken
			throw runtime_error("Lambda has grown too large!");
		else {

			// A more adventerous lambda was worse.  If we're cautious, try the same lambda.
			if(lambdaMode == CAUTIOUS) {
				NonlinearOptimizer sameLambda(graph_, newConfig, solver_, lambda_);
				if(sameLambda.error_ <= error_)
					return sameLambda;
			}

			// Either we're not cautious, or the same lambda was worse than the current error.
			// The more adventerous lambda was worse too, so make lambda more conservative
			// and keep the same config.

			// TODO: can we avoid copying the config ?
			if(lambdaMode >= BOUNDED && lambda_ >= 1.0e5) {
				return NonlinearOptimizer(graph_, newConfig, solver_, lambda_);;
			} else {
				NonlinearOptimizer cautious(graph_, config_, solver_, lambda_ * factor);
				return cautious.try_lambda(linear, verbosity, factor, lambdaMode);
			}

		}
	}

	/* ************************************************************************* */
	// One iteration of Levenberg Marquardt
	/* ************************************************************************* */
	template<class G, class C, class L, class S, class W>
	NonlinearOptimizer<G, C, L, S, W> NonlinearOptimizer<G, C, L, S, W>::iterateLM(
			verbosityLevel verbosity, double lambdaFactor, LambdaMode lambdaMode) const {

		// maybe show output
		if (verbosity >= CONFIG)
			config_->print("config");
		if (verbosity >= ERROR)
			cout << "error: " << error_ << endl;
		if (verbosity >= LAMBDA)
			cout << "lambda = " << lambda_ << endl;

		// linearize all factors once
		boost::shared_ptr<L> linear = solver_->linearize(*graph_, *config_);
		if (verbosity >= LINEAR)
			linear->print("linear");

		// try lambda steps with successively larger lambda until we achieve descent
		if (verbosity >= LAMBDA) cout << "Trying Lambda for the first time" << endl;
		return try_lambda(*linear, verbosity, lambdaFactor, lambdaMode);
	}

	/* ************************************************************************* */
	template<class G, class C, class L, class S, class W>
	NonlinearOptimizer<G, C, L, S, W> NonlinearOptimizer<G, C, L, S, W>::levenbergMarquardt(
			double relativeThreshold, double absoluteThreshold,
			verbosityLevel verbosity, int maxIterations, double lambdaFactor, LambdaMode lambdaMode) const {

		if (maxIterations <= 0) return *this;

		// check if we're already close enough
		if (error_ < absoluteThreshold) {
			if (verbosity >= ERROR) cout << "Exiting, as error = " << error_
					<< " < absoluteThreshold (" << absoluteThreshold << ")" << endl;
			return *this;
		}

		// do one iteration of LM
		NonlinearOptimizer next = iterateLM(verbosity, lambdaFactor, lambdaMode);

		// check convergence
		// TODO: move convergence checks here and incorporate in verbosity levels
		// TODO: build into iterations somehow as an instance variable
		bool converged = gtsam::check_convergence(relativeThreshold,
				absoluteThreshold, error_, next.error_, verbosity);

		// return converged state or iterate
		if (converged || maxIterations <= 1) {
			// maybe show output
			if (verbosity >= CONFIG)
				next.config_->print("final config");
			if (verbosity >= ERROR)
				cout << "final error: " << next.error_ << endl;
			if (verbosity >= LAMBDA)
				cout << "final lambda = " << next.lambda_ << endl;
			return next;
		} else
			return next.levenbergMarquardt(relativeThreshold, absoluteThreshold,
					verbosity, maxIterations-1, lambdaFactor, lambdaMode);
	}

/* ************************************************************************* */

}
