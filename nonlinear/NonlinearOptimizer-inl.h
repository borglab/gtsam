/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

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
#include <gtsam/nonlinear/NonlinearOptimizer.h>

#define INSTANTIATE_NONLINEAR_OPTIMIZER(G,C) \
  template class NonlinearOptimizer<G,C>;

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	inline bool check_convergence(
			double relativeErrorTreshold,
			double absoluteErrorTreshold,
			double errorThreshold,
			double currentError, double newError, int verbosity) {

		if ( verbosity >= 2 ) {
			if ( newError <= errorThreshold )
				cout << "errorThreshold: " << newError << " < " << errorThreshold << endl;
			else
				cout << "errorThreshold: " << newError << " > " << errorThreshold << endl;
		}

		if ( newError <= errorThreshold ) return true ;

		// check if diverges
		double absoluteDecrease = currentError - newError;
		if (verbosity >= 2) {
			if (absoluteDecrease < absoluteErrorTreshold)
				cout << "absoluteDecrease: " << absoluteDecrease << " < " << absoluteErrorTreshold << endl;
			else
				cout << "absoluteDecrease: " << absoluteDecrease << " >= " << absoluteErrorTreshold << endl;
		}

		// calculate relative error decrease and update currentError
		double relativeDecrease = absoluteDecrease / currentError;
		if (verbosity >= 2) {
			if (relativeDecrease < relativeErrorTreshold)
				cout << "relativeDecrease: " << relativeDecrease << " < " << relativeErrorTreshold << endl;
			else
				cout << "relativeDecrease: " << relativeDecrease << " >= " << relativeErrorTreshold << endl;
		}
		bool converged = (relativeDecrease < relativeErrorTreshold)
				|| (absoluteDecrease < absoluteErrorTreshold);
		if (verbosity >= 1 && converged)
			cout << "converged" << endl;
		return converged;
	}

	/* ************************************************************************* */
	template<class G, class C, class L, class S, class W>
	NonlinearOptimizer<G, C, L, S, W>::NonlinearOptimizer(shared_graph graph,
			shared_values values, shared_ordering ordering, double lambda) :
			graph_(graph), values_(values), error_(graph->error(*values)),
			ordering_(ordering), lambda_(lambda), dimensions_(new vector<size_t>(values->dims(*ordering))) {
		if (!graph) throw std::invalid_argument(
				"NonlinearOptimizer constructor: graph = NULL");
		if (!values) throw std::invalid_argument(
				"NonlinearOptimizer constructor: values = NULL");
		if (!ordering) throw std::invalid_argument(
				"NonlinearOptimizer constructor: ordering = NULL");
	}

	/* ************************************************************************* */
	// linearize and optimize
	/* ************************************************************************* */
	template<class G, class C, class L, class S, class W>
	VectorValues NonlinearOptimizer<G, C, L, S, W>::linearizeAndOptimizeForDelta() const {
		boost::shared_ptr<L> linearized = graph_->linearize(*values_, *ordering_);
//		NonlinearOptimizer prepared(graph_, values_, ordering_, error_, lambda_);
		return *S(*linearized).optimize();
	}

	/* ************************************************************************* */
	// One iteration of Gauss Newton
	/* ************************************************************************* */
	template<class G, class C, class L, class S, class W>
	NonlinearOptimizer<G, C, L, S, W> NonlinearOptimizer<G, C, L, S, W>::iterate(
			Parameters::verbosityLevel verbosity) const {
		// linearize and optimize
		VectorValues delta = linearizeAndOptimizeForDelta();

		// maybe show output
		if (verbosity >= Parameters::DELTA)
			delta.print("delta");

		// take old values and update it
		shared_values newValues(new C(values_->expmap(delta, *ordering_)));

		// maybe show output
		if (verbosity >= Parameters::VALUES)
			newValues->print("newValues");

		NonlinearOptimizer newOptimizer = newValues_(newValues);

		if (verbosity >= Parameters::ERROR)
			cout << "error: " << newOptimizer.error_ << endl;

		return newOptimizer;
	}

	/* ************************************************************************* */
	template<class G, class C, class L, class S, class W>
	NonlinearOptimizer<G, C, L, S, W> NonlinearOptimizer<G, C, L, S, W>::gaussNewton(
			double relativeThreshold, double absoluteThreshold,
			Parameters::verbosityLevel verbosity, int maxIterations) const {
		static W writer(error_);

		// check if we're already close enough
		if (error_ < absoluteThreshold) {
			if (verbosity >= Parameters::ERROR) cout << "Exiting, as error = " << error_
					<< " < absoluteThreshold (" << absoluteThreshold << ")" << endl;
			return *this;
		}

		// linearize, solve, update
		NonlinearOptimizer next = iterate(verbosity);

		writer.write(next.error_);

		// check convergence
		bool converged = gtsam::check_convergence(
				relativeThreshold,
				absoluteThreshold,
				0.0,
				error_,
				next.error_,
				verbosity);

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
			const L& linear, Parameters::verbosityLevel verbosity, double factor, Parameters::LambdaMode lambdaMode) const {

		if (verbosity >= Parameters::TRYLAMBDA)
			cout << "trying lambda = " << lambda_ << endl;

		// add prior-factors
		L damped = linear.add_priors(1.0/sqrt(lambda_), *dimensions_);
		if (verbosity >= Parameters::DAMPED)
			damped.print("damped");

		// solve
		VectorValues delta = *S(damped).optimize();
		if (verbosity >= Parameters::TRYDELTA)
			delta.print("delta");

		// update values
		shared_values newValues(new C(values_->expmap(delta, *ordering_))); // TODO: updateValues
//		if (verbosity >= TRYvalues)
//			newValues->print("values");

		// create new optimization state with more adventurous lambda
		NonlinearOptimizer next(newValuesNewLambda_(newValues, lambda_ / factor));
		if (verbosity >= Parameters::TRYLAMBDA) cout << "next error = " << next.error_ << endl;

		if(lambdaMode >= Parameters::CAUTIOUS) {
			throw runtime_error("CAUTIOUS mode not working yet, please use BOUNDED.");
		}

		if(next.error_ <= error_) {

			// If we're cautious, see if the current lambda is better
			// todo:  include stopping criterion here?
			if(lambdaMode == Parameters::CAUTIOUS) {
				NonlinearOptimizer sameLambda(newValues_(newValues));
				if(sameLambda.error_ <= next.error_)
					return sameLambda;
			}

			// Either we're not cautious, or we are but the adventerous lambda is better than the same one.
			return next;
		} else if (lambda_ > 1e+10) // if lambda gets too big, something is broken
			throw runtime_error("Lambda has grown too large!");
		else {

			// A more adventerous lambda was worse.  If we're cautious, try the same lambda.
			if(lambdaMode == Parameters::CAUTIOUS) {
				NonlinearOptimizer sameLambda(newValues_(newValues));
				if(sameLambda.error_ <= error_)
					return sameLambda;
			}

			// Either we're not cautious, or the same lambda was worse than the current error.
			// The more adventerous lambda was worse too, so make lambda more conservative
			// and keep the same values.

			// TODO: can we avoid copying the values ?
			if(lambdaMode >= Parameters::BOUNDED && lambda_ >= 1.0e5) {
				return NonlinearOptimizer(newValues_(newValues));
			} else {
				NonlinearOptimizer cautious(newLambda_(lambda_ * factor));
				return cautious.try_lambda(linear, verbosity, factor, lambdaMode);
			}

		}
	}

	/* ************************************************************************* */
	// One iteration of Levenberg Marquardt
	/* ************************************************************************* */
	template<class G, class C, class L, class S, class W>
	NonlinearOptimizer<G, C, L, S, W> NonlinearOptimizer<G, C, L, S, W>::iterateLM(
			Parameters::verbosityLevel verbosity, double lambdaFactor, Parameters::LambdaMode lambdaMode) const {

		// show output
		if (verbosity >= Parameters::VALUES)
			values_->print("values");
		if (verbosity >= Parameters::ERROR)
			cout << "error: " << error_ << endl;
		if (verbosity >= Parameters::LAMBDA)
			cout << "lambda = " << lambda_ << endl;

		// linearize all factors once
		boost::shared_ptr<L> linear = graph_->linearize(*values_, *ordering_);
		if (verbosity >= Parameters::LINEAR)
			linear->print("linear");

		// try lambda steps with successively larger lambda until we achieve descent
		if (verbosity >= Parameters::LAMBDA) cout << "Trying Lambda for the first time" << endl;
		return try_lambda(*linear, verbosity, lambdaFactor, lambdaMode);
	}

	/* ************************************************************************* */
	template<class G, class C, class L, class S, class W>
	NonlinearOptimizer<G, C, L, S, W> NonlinearOptimizer<G, C, L, S, W>::levenbergMarquardt(
			double relativeThreshold, double absoluteThreshold,
			Parameters::verbosityLevel verbosity, int maxIterations, double lambdaFactor, Parameters::LambdaMode lambdaMode) const {

		return levenbergMarquardt(NonlinearOptimizationParameters (absoluteThreshold, relativeThreshold, absoluteThreshold,
				maxIterations, lambdaFactor, verbosity, lambdaMode)) ;
	}


	template<class G, class C, class L, class S, class W>
	NonlinearOptimizer<G, C, L, S, W> NonlinearOptimizer<G, C, L, S, W>::
	levenbergMarquardt(const NonlinearOptimizationParameters &para) const {

		if (para.maxIterations_ <= 0) return *this;

		// check if we're already close enough
		if (error_ < para.sumError_) {
			if (para.verbosity_ >= Parameters::ERROR)
				cout << "Exiting, as error = " << error_ << " < " << para.sumError_ << endl;
			return *this;
		}

		// do one iteration of LM
		NonlinearOptimizer next = iterateLM(para.verbosity_, para.lambdaFactor_, para.lambdaMode_);

		// check convergence
		// TODO: move convergence checks here and incorporate in verbosity levels
		// TODO: build into iterations somehow as an instance variable
		bool converged = gtsam::check_convergence(
				para.relDecrease_,
				para.absDecrease_,
				para.sumError_,
				error_,
				next.error_,
				para.verbosity_);

		// return converged state or iterate
		if (converged || para.maxIterations_ <= 1) {
			// maybe show output
			if (para.verbosity_ >= Parameters::VALUES)
				next.values_->print("final values");
			if (para.verbosity_ >= Parameters::ERROR)
				cout << "final error: " << next.error_ << endl;
			if (para.verbosity_ >= Parameters::LAMBDA)
				cout << "final lambda = " << next.lambda_ << endl;
			return next;
		} else {
			NonlinearOptimizationParameters newPara = para ;
			newPara.maxIterations_ = newPara.maxIterations_ - 1;
			return next.levenbergMarquardt(newPara) ;
		}
	}

	/* ************************************************************************* */

}
