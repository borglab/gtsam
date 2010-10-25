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

	/* ************************************************************************* */
	template<class G, class C, class L, class S, class W>
	NonlinearOptimizer<G, C, L, S, W>::NonlinearOptimizer(shared_graph graph,
			shared_values values, shared_ordering ordering, double lambda) :
			graph_(graph), values_(values), error_(graph->error(*values)), ordering_(ordering),
			parameters_(Parameters::newLambda(lambda)), dimensions_(new vector<size_t>(values->dims(*ordering))) {
		if (!graph) throw std::invalid_argument(
				"NonlinearOptimizer constructor: graph = NULL");
		if (!values) throw std::invalid_argument(
				"NonlinearOptimizer constructor: values = NULL");
		if (!ordering) throw std::invalid_argument(
				"NonlinearOptimizer constructor: ordering = NULL");
	}

	template<class G, class C, class L, class S, class W>
	NonlinearOptimizer<G, C, L, S, W>::NonlinearOptimizer(
			shared_graph graph,	shared_values values, shared_ordering ordering, shared_solver solver, const double lambda):
			graph_(graph), values_(values), error_(graph->error(*values)), ordering_(ordering), solver_(solver),
			parameters_(Parameters::newLambda(lambda)), dimensions_(new vector<size_t>(values->dims(*ordering))) {
		if (!graph) throw std::invalid_argument(
				"NonlinearOptimizer constructor: graph = NULL");
		if (!values) throw std::invalid_argument(
				"NonlinearOptimizer constructor: values = NULL");
		if (!ordering) throw std::invalid_argument(
				"NonlinearOptimizer constructor: ordering = NULL");
		if (!solver) throw std::invalid_argument(
				"NonlinearOptimizer constructor: solver = NULL");
	}

	template<class G, class C, class L, class S, class W>
	NonlinearOptimizer<G, C, L, S, W>::NonlinearOptimizer(shared_graph graph,
			shared_values values, shared_ordering ordering, shared_parameters parameters) :
			graph_(graph), values_(values), error_(graph->error(*values)),
			ordering_(ordering), parameters_(parameters), dimensions_(new vector<size_t>(values->dims(*ordering))) {
		if (!graph) throw std::invalid_argument(
				"NonlinearOptimizer constructor: graph = NULL");
		if (!values) throw std::invalid_argument(
				"NonlinearOptimizer constructor: values = NULL");
		if (!ordering) throw std::invalid_argument(
				"NonlinearOptimizer constructor: ordering = NULL");
	}

	template<class G, class C, class L, class S, class W>
	NonlinearOptimizer<G, C, L, S, W>::NonlinearOptimizer(
			shared_graph graph,
			shared_values values,
			shared_ordering ordering,
			shared_solver solver,
			shared_parameters parameters):
			graph_(graph), values_(values), error_(graph->error(*values)), ordering_(ordering), solver_(solver),
			parameters_(parameters), dimensions_(new vector<size_t>(values->dims(*ordering))) {
		if (!graph) throw std::invalid_argument(
				"NonlinearOptimizer constructor: graph = NULL");
		if (!values) throw std::invalid_argument(
				"NonlinearOptimizer constructor: values = NULL");
		if (!ordering) throw std::invalid_argument(
				"NonlinearOptimizer constructor: ordering = NULL");
		if (!solver) throw std::invalid_argument(
				"NonlinearOptimizer constructor: solver = NULL");
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
	NonlinearOptimizer<G, C, L, S, W> NonlinearOptimizer<G, C, L, S, W>::iterate() const {

		Parameters::verbosityLevel verbosity = parameters_->verbosity_ ;
	    boost::shared_ptr<L> linearized = graph_->linearize(*values_, *ordering_);
		shared_solver newSolver = solver_;

		if(newSolver) newSolver = newSolver->update(*linearized);
		else newSolver.reset(new S(*linearized));

		VectorValues delta = *newSolver->optimize();

		// maybe show output
		if (verbosity >= Parameters::DELTA) delta.print("delta");

		// take old values and update it
		shared_values newValues(new C(values_->expmap(delta, *ordering_)));

		// maybe show output
		if (verbosity >= Parameters::VALUES) newValues->print("newValues");

		NonlinearOptimizer newOptimizer = newValuesSolver_(newValues, newSolver);

		if (verbosity >= Parameters::ERROR)	cout << "error: " << newOptimizer.error_ << endl;
		return newOptimizer;
	}

	template<class G, class C, class L, class S, class W>
	NonlinearOptimizer<G, C, L, S, W> NonlinearOptimizer<G, C, L, S, W>::iterate(
			Parameters::verbosityLevel verbosity) const {
		return this->newVerbosity_(verbosity).iterate();
	}

	/* ************************************************************************* */

	template<class G, class C, class L, class S, class W>
	NonlinearOptimizer<G, C, L, S, W> NonlinearOptimizer<G, C, L, S, W>::gaussNewton() const {
		static W writer(error_);

		if (error_ < parameters_->sumError_ ) {
			if ( parameters_->verbosity_ >= Parameters::ERROR)
				cout << "Exiting, as error = " << error_
				     << " < sumError (" << parameters_->sumError_ << ")" << endl;
			return *this;
		}

		// linearize, solve, update
		NonlinearOptimizer next = iterate();

		writer.write(next.error_);

		// check convergence
		bool converged = gtsam::check_convergence(*parameters_,	error_,	next.error_);

		// return converged state or iterate
		if (converged) return next;
		else return next.gaussNewton();
	}

	template<class G, class C, class L, class S, class W>
	NonlinearOptimizer<G, C, L, S, W> NonlinearOptimizer<G, C, L, S, W>::gaussNewton(
			double relativeThreshold,
			double absoluteThreshold,
			Parameters::verbosityLevel verbosity,
			int maxIterations) const {

		Parameters def ;
		def.relDecrease_ = relativeThreshold ;
		def.absDecrease_ = absoluteThreshold ;
		def.verbosity_ = verbosity ;
		def.maxIterations_ = maxIterations ;

		shared_parameters ptr(boost::make_shared<NonlinearOptimizationParameters>(def)) ;
		return newParameters_(ptr).gaussNewton() ;
	}

	/* ************************************************************************* */
	// Recursively try to do tempered Gauss-Newton steps until we succeed.
	// Form damped system with given lambda, and return a new, more optimistic
	// optimizer if error decreased or recurse with a larger lambda if not.
	// TODO: in theory we can't infinitely recurse, but maybe we should put a max.
	/* ************************************************************************* */
	template<class G, class C, class L, class S, class W>
	NonlinearOptimizer<G, C, L, S, W> NonlinearOptimizer<G, C, L, S, W>::try_lambda(const L& linear) const {

		const Parameters::verbosityLevel verbosity = parameters_->verbosity_ ;
		const double lambda = parameters_->lambda_ ;
		const Parameters::LambdaMode lambdaMode = parameters_->lambdaMode_ ;
		const double factor = parameters_->lambdaFactor_ ;

		if (verbosity >= Parameters::TRYLAMBDA) cout << "trying lambda = " << lambda << endl;

		// add prior-factors
		L damped = linear.add_priors(1.0/sqrt(lambda), *dimensions_);
		if (verbosity >= Parameters::DAMPED) damped.print("damped");

		// solve
		shared_solver newSolver = solver_;

		if(newSolver) newSolver = newSolver->update(damped);
		else newSolver.reset(new S(damped));

		VectorValues delta = *newSolver->optimize();
		if (verbosity >= Parameters::TRYDELTA) delta.print("delta");

		// update values
		shared_values newValues(new C(values_->expmap(delta, *ordering_))); // TODO: updateValues

		// create new optimization state with more adventurous lambda
		NonlinearOptimizer next(newValuesSolverLambda_(newValues, newSolver, lambda / factor));
		if (verbosity >= Parameters::TRYLAMBDA) cout << "next error = " << next.error_ << endl;

		if( lambdaMode >= Parameters::CAUTIOUS) throw runtime_error("CAUTIOUS mode not working yet, please use BOUNDED.");

		if( next.error_ <= error_ ) {
			// If we're cautious, see if the current lambda is better
			// todo:  include stopping criterion here?
			if( lambdaMode == Parameters::CAUTIOUS ) {
				NonlinearOptimizer sameLambda(newValuesSolver_(newValues, newSolver));
				if(sameLambda.error_ <= next.error_) return sameLambda;
			}

			// Either we're not cautious, or we are but the adventerous lambda is better than the same one.
			return next;

		} else if (lambda > 1e+10) // if lambda gets too big, something is broken
			throw runtime_error("Lambda has grown too large!");
		else {
			// A more adventerous lambda was worse.  If we're cautious, try the same lambda.
			if(lambdaMode == Parameters::CAUTIOUS) {
				NonlinearOptimizer sameLambda(newValuesSolver_(newValues, newSolver));
				if(sameLambda.error_ <= error_) return sameLambda;
			}

			// Either we're not cautious, or the same lambda was worse than the current error.
			// The more adventerous lambda was worse too, so make lambda more conservative
			// and keep the same values.

			// TODO: can we avoid copying the values ?
			if(lambdaMode >= Parameters::BOUNDED && lambda >= 1.0e5) {
				return NonlinearOptimizer(newValuesSolver_(newValues, newSolver));
			} else {
				NonlinearOptimizer cautious(newLambda_(lambda * factor));
				return cautious.try_lambda(linear);
			}
		}

	}

	/* ************************************************************************* */
	// One iteration of Levenberg Marquardt
	/* ************************************************************************* */

	template<class G, class C, class L, class S, class W>
	NonlinearOptimizer<G, C, L, S, W> NonlinearOptimizer<G, C, L, S, W>::iterateLM() const {

		const Parameters::verbosityLevel verbosity = parameters_->verbosity_ ;
		const double lambda = parameters_->lambda_ ;

		// show output
		if (verbosity >= Parameters::VALUES) values_->print("values");
		if (verbosity >= Parameters::ERROR) cout << "error: " << error_ << endl;
		if (verbosity >= Parameters::LAMBDA) cout << "lambda = " << lambda << endl;

		// linearize all factors once
		boost::shared_ptr<L> linear = graph_->linearize(*values_, *ordering_);

		if (verbosity >= Parameters::LINEAR) linear->print("linear");

		// try lambda steps with successively larger lambda until we achieve descent
		if (verbosity >= Parameters::LAMBDA) cout << "Trying Lambda for the first time" << endl;

		return try_lambda(*linear);
	}


	template<class G, class C, class L, class S, class W>
	NonlinearOptimizer<G, C, L, S, W> NonlinearOptimizer<G, C, L, S, W>::iterateLM(
		Parameters::verbosityLevel verbosity,
		double lambdaFactor,
		Parameters::LambdaMode lambdaMode) const {

		NonlinearOptimizationParameters def(*parameters_) ;
		def.verbosity_ = verbosity ;
		def.lambdaFactor_ = lambdaFactor ;
		def.lambdaMode_ = lambdaMode ;
		shared_parameters ptr(boost::make_shared<Parameters>(def)) ;
		return newParameters_(ptr).iterateLM();
	}

	/* ************************************************************************* */

	template<class G, class C, class L, class S, class W>
	NonlinearOptimizer<G, C, L, S, W> NonlinearOptimizer<G, C, L, S, W>::levenbergMarquardt() const {

		const int maxIterations = parameters_->maxIterations_ ;
		if ( maxIterations <= 0) return *this;

		const Parameters::verbosityLevel verbosity = parameters_->verbosity_ ;

		// check if we're already close enough
		if (error_ < parameters_->sumError_) {
			if ( verbosity >= Parameters::ERROR )
				cout << "Exiting, as sumError = " << error_ << " < " << parameters_->sumError_ << endl;
			return *this;
		}

		// do one iteration of LM
		NonlinearOptimizer next = iterateLM();

		// check convergence
		// TODO: move convergence checks here and incorporate in verbosity levels
		// TODO: build into iterations somehow as an instance variable
		bool converged = gtsam::check_convergence(*parameters_, error_, next.error_);

		// return converged state or iterate
		if ( converged || maxIterations <= 1 ) {
			if (verbosity >= Parameters::VALUES) next.values_->print("final values");
			if (verbosity >= Parameters::ERROR) cout << "final error: " << next.error_ << endl;
			if (verbosity >= Parameters::LAMBDA) cout << "final lambda = " << next.lambda() << endl;
			return next;
		} else {
			return next.newMaxIterations_(maxIterations - 1).levenbergMarquardt() ;
		}
	}

	template<class G, class C, class L, class S, class W>
	NonlinearOptimizer<G, C, L, S, W> NonlinearOptimizer<G, C, L, S, W>::levenbergMarquardt(
			double relativeThreshold,
			double absoluteThreshold,
			Parameters::verbosityLevel verbosity,
			int maxIterations,
			double lambdaFactor,
			Parameters::LambdaMode lambdaMode) const {

		NonlinearOptimizationParameters def;
		def.relDecrease_ = relativeThreshold ;
		def.absDecrease_ = absoluteThreshold ;
		def.verbosity_ = verbosity ;
		def.maxIterations_ = maxIterations ;
		def.lambdaFactor_ = lambdaFactor ;
		def.lambdaMode_ = lambdaMode ;
		shared_parameters ptr = boost::make_shared<Parameters>(def) ;
		return newParameters_(ptr).levenbergMarquardt() ;
	}

	template<class G, class C, class L, class S, class W>
	NonlinearOptimizer<G, C, L, S, W> NonlinearOptimizer<G, C, L, S, W>::
	levenbergMarquardt(const NonlinearOptimizationParameters &parameters) const {
		return newParameters_(boost::make_shared<Parameters>(parameters)).levenbergMarquardt() ;
	}

	/* ************************************************************************* */

}
