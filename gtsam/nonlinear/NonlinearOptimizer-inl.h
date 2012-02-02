/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file NonlinearOptimizer-inl.h
 * This is a template definition file, include it where needed (only!)
 * so that the appropriate code is generated and link errors avoided.
 * @brief: Encapsulates nonlinear optimization state
 * @author Frank Dellaert
 * @date Sep 7, 2009
 */

#pragma once

#include <iostream>
#include <boost/tuple/tuple.hpp>
#include <gtsam/base/cholesky.h>
#include <gtsam/nonlinear/DoglegOptimizerImpl.h>

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	template<class G, class L, class S, class W>
	NonlinearOptimizer<G, L, S, W>::NonlinearOptimizer(shared_graph graph,
			shared_values values, shared_ordering ordering, shared_parameters parameters) :
			graph_(graph), values_(values), error_(graph->error(*values)), ordering_(ordering),
			parameters_(parameters), iterations_(0),
			dimensions_(new vector<size_t>(values->dims(*ordering))),
			structure_(new VariableIndex(*graph->symbolic(*ordering))) {
		if (!graph) throw std::invalid_argument(
				"NonlinearOptimizer constructor: graph = NULL");
		if (!values) throw std::invalid_argument(
				"NonlinearOptimizer constructor: values = NULL");
		if (!ordering) throw std::invalid_argument(
				"NonlinearOptimizer constructor: ordering = NULL");
	}

	/* ************************************************************************* */
	// FIXME: remove this constructor
	template<class G, class L, class S, class W>
	NonlinearOptimizer<G, L, S, W>::NonlinearOptimizer(shared_graph graph,
			shared_values values, shared_ordering ordering,
			shared_solver spcg_solver, shared_parameters parameters) :
			graph_(graph), values_(values), error_(graph->error(*values)), ordering_(ordering),
			parameters_(parameters), iterations_(0),
			dimensions_(new vector<size_t>(values->dims(*ordering))),
			spcg_solver_(spcg_solver) {
		if (!graph) throw std::invalid_argument(
				"NonlinearOptimizer constructor: graph = NULL");
		if (!values) throw std::invalid_argument(
				"NonlinearOptimizer constructor: values = NULL");
		if (!spcg_solver) throw std::invalid_argument(
				"NonlinearOptimizer constructor: spcg_solver = NULL");
	}

	/* ************************************************************************* */
	// One iteration of Gauss Newton
	/* ************************************************************************* */
	template<class G, class L, class S, class W>
	NonlinearOptimizer<G, L, S, W> NonlinearOptimizer<G, L, S, W>::iterate() const {

		Parameters::verbosityLevel verbosity = parameters_->verbosity_ ;

		// FIXME: get rid of spcg solver
		shared_solver solver;
		if (spcg_solver_) { // special case for SPCG
			spcg_solver_->replaceFactors(linearize());
			solver = spcg_solver_;
		} else { // normal case
			solver = createSolver();
		}

		VectorValues delta = *solver->optimize();

		// maybe show output
		if (verbosity >= Parameters::DELTA) delta.print("delta");

		// take old values and update it
		shared_values newValues(new Values(values_->retract(delta, *ordering_)));

		// maybe show output
		if (verbosity >= Parameters::VALUES) newValues->print("newValues");

		NonlinearOptimizer newOptimizer = newValues_(newValues);
		++ newOptimizer.iterations_;

		if (verbosity >= Parameters::ERROR)	cout << "error: " << newOptimizer.error_ << endl;
		return newOptimizer;
	}

	/* ************************************************************************* */
	template<class G, class L, class S, class W>
	NonlinearOptimizer<G, L, S, W> NonlinearOptimizer<G, L, S, W>::gaussNewton() const {
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

	/* ************************************************************************* */
	// Iteratively try to do tempered Gauss-Newton steps until we succeed.
	// Form damped system with given lambda, and return a new, more optimistic
	// optimizer if error decreased or iterate with a larger lambda if not.
	// TODO: in theory we can't infinitely recurse, but maybe we should put a max.
	// Reminder: the parameters are Graph type $G$, Values class type $T$,
	// linear system class $L$, the non linear solver type $S$, and the writer type $W$
	template<class G, class L, class S, class W>
	NonlinearOptimizer<G, L, S, W> NonlinearOptimizer<G, L, S, W>::try_lambda(const L& linearSystem) {

		const Parameters::verbosityLevel verbosity = parameters_->verbosity_ ;
		const Parameters::LambdaMode lambdaMode = parameters_->lambdaMode_ ;
		const double factor = parameters_->lambdaFactor_ ;
		double lambda = parameters_->lambda_ ;

		if( lambdaMode >= Parameters::CAUTIOUS) throw runtime_error("CAUTIOUS mode not working yet, please use BOUNDED.");

		double next_error = error_;
		shared_values next_values = values_;

		// Keep increasing lambda until we make make progress
		while(true) {
		  if (verbosity >= Parameters::TRYLAMBDA) cout << "trying lambda = " << lambda << endl;

		  // add prior-factors
		  // TODO: replace this dampening with a backsubstitution approach
		  typename L::shared_ptr dampedSystem(new L(linearSystem));
		  {
		    double sigma = 1.0 / sqrt(lambda);
		    dampedSystem->reserve(dampedSystem->size() + dimensions_->size());
		    // for each of the variables, add a prior
		    for(Index j=0; j<dimensions_->size(); ++j) {
		      size_t dim = (*dimensions_)[j];
		      Matrix A = eye(dim);
		      Vector b = zero(dim);
		      SharedDiagonal model = noiseModel::Isotropic::Sigma(dim,sigma);
		      typename L::sharedFactor prior(new JacobianFactor(j, A, b, model));
		      dampedSystem->push_back(prior);
		    }
		  }
		  if (verbosity >= Parameters::DAMPED) dampedSystem->print("damped");

		  // Create a new solver using the damped linear system
 		  // FIXME: remove spcg specific code
		  if (spcg_solver_) spcg_solver_->replaceFactors(dampedSystem);
		  shared_solver solver = (spcg_solver_) ? spcg_solver_ : shared_solver(
		  		new S(dampedSystem, structure_, parameters_->useQR_));

		  // Try solving
		  try {
		    VectorValues delta = *solver->optimize();
		    if (verbosity >= Parameters::TRYLAMBDA) cout << "linear delta norm = " << delta.vector().norm() << endl;
		    if (verbosity >= Parameters::TRYDELTA) delta.print("delta");

		    // update values
		    shared_values newValues(new Values(values_->retract(delta, *ordering_)));

		    // create new optimization state with more adventurous lambda
		    double error = graph_->error(*newValues);

		    if (verbosity >= Parameters::TRYLAMBDA) cout << "next error = " << error << endl;

		    if( error <= error_ ) {
		      next_values = newValues;
		      next_error = error;
		      lambda /= factor;
		      break;
		    }
		    else {
		      // Either we're not cautious, or the same lambda was worse than the current error.
		      // The more adventurous lambda was worse too, so make lambda more conservative
		      // and keep the same values.
		      if(lambdaMode >= Parameters::BOUNDED && lambda >= 1.0e5) {
	          if(verbosity >= Parameters::ERROR)
	            cout << "Warning:  Levenberg-Marquardt giving up because cannot decrease error with maximum lambda" << endl;
		        break;
		      } else {
		        lambda *= factor;
		      }
		    }
		  } catch(const NegativeMatrixException& e) {
		    if(verbosity >= Parameters::LAMBDA)
		      cout << "Negative matrix, increasing lambda" << endl;
		    // Either we're not cautious, or the same lambda was worse than the current error.
		    // The more adventurous lambda was worse too, so make lambda more conservative
		    // and keep the same values.
		    if(lambdaMode >= Parameters::BOUNDED && lambda >= 1.0e5) {
		      if(verbosity >= Parameters::ERROR)
		        cout << "Warning:  Levenberg-Marquardt giving up because cannot decrease error with maximum lambda" << endl;
		      break;
		    } else {
		      lambda *= factor;
		    }
		  } catch(...) {
		    throw;
		  }
		} // end while

		return newValuesErrorLambda_(next_values, next_error, lambda);
	}

	/* ************************************************************************* */
	// One iteration of Levenberg Marquardt
	// Reminder: the parameters are Graph type $G$, Values class type $T$,
	// linear system class $L$, the non linear solver type $S$, and the writer type $W$
	template<class G, class L, class S, class W>
	NonlinearOptimizer<G, L, S, W> NonlinearOptimizer<G, L, S, W>::iterateLM() {

		const Parameters::verbosityLevel verbosity = parameters_->verbosity_ ;
		const double lambda = parameters_->lambda_ ;

		// show output
		if (verbosity >= Parameters::VALUES) values_->print("values");
		if (verbosity >= Parameters::ERROR) cout << "error: " << error_ << endl;
		if (verbosity >= Parameters::LAMBDA) cout << "lambda = " << lambda << endl;

		// linearize all factors once
		boost::shared_ptr<L> linear(new L(*graph_->linearize(*values_, *ordering_)));

		if (verbosity >= Parameters::LINEAR) linear->print("linear");

		// try lambda steps with successively larger lambda until we achieve descent
		if (verbosity >= Parameters::LAMBDA) cout << "Trying Lambda for the first time" << endl;

		return try_lambda(*linear);
	}

	/* ************************************************************************* */
	// Reminder: the parameters are Graph type $G$, Values class type $T$,
	// linear system class $L$, the non linear solver type $S$, and the writer type $W$
	template<class G, class L, class S, class W>
	NonlinearOptimizer<G, L, S, W> NonlinearOptimizer<G, L, S, W>::levenbergMarquardt() {

		// Initialize
		bool converged = false;
		const Parameters::verbosityLevel verbosity = parameters_->verbosity_ ;

		// check if we're already close enough
		if (error_ < parameters_->sumError_) {
			if ( verbosity >= Parameters::ERROR )
				cout << "Exiting, as sumError = " << error_ << " < " << parameters_->sumError_ << endl;
			return *this;
		}

		// for the case that maxIterations_ = 0
		iterations_ = 1;
		if (iterations_ >= parameters_->maxIterations_)
			return *this;

		// Iterative loop that implements Levenberg-Marquardt
		while (true) {
			double previous_error = error_;
			// do one iteration of LM
			NonlinearOptimizer next = iterateLM();
			error_ = next.error_;
			values_ = next.values_;
			parameters_ = next.parameters_;
			iterations_ = next.iterations_;

			// check convergence
			// TODO: move convergence checks here and incorporate in verbosity levels
			// TODO: build into iterations somehow as an instance variable
			converged = gtsam::check_convergence(*parameters_, previous_error, error_);

			if(iterations_ >= parameters_->maxIterations_ || converged == true) {
				if (verbosity >= Parameters::VALUES) values_->print("final values");
				if (verbosity >= Parameters::ERROR && iterations_ >= parameters_->maxIterations_) cout << "Terminating because reached maximum iterations" << endl;
				if (verbosity >= Parameters::ERROR) cout << "final error: " << error_ << endl;
				if (verbosity >= Parameters::LAMBDA) cout << "final lambda = " << lambda() << endl;
				return *this;
			}
			iterations_++;
		}
	}

  /* ************************************************************************* */
  template<class G, class L, class S, class W>
  NonlinearOptimizer<G, L, S, W> NonlinearOptimizer<G, L, S, W>::iterateDogLeg() {

    S solver(*graph_->linearize(*values_, *ordering_));
    DoglegOptimizerImpl::IterationResult result = DoglegOptimizerImpl::Iterate(
        parameters_->lambda_, DoglegOptimizerImpl::ONE_STEP_PER_ITERATION, *solver.eliminate(),
        *graph_, *values_, *ordering_, error_, parameters_->verbosity_ > Parameters::ERROR);
    shared_values newValues(new Values(values_->retract(result.dx_d, *ordering_)));
    return newValuesErrorLambda_(newValues, result.f_error, result.Delta);
  }

  /* ************************************************************************* */
  template<class G, class L, class S, class W>
  NonlinearOptimizer<G, L, S, W> NonlinearOptimizer<G, L, S, W>::dogLeg() {
    static W writer(error_);

    // check if we're already close enough
    if (error_ < parameters_->sumError_) {
      if ( parameters_->verbosity_ >= Parameters::ERROR )
        cout << "Exiting, as sumError = " << error_ << " < " << parameters_->sumError_ << endl;
      return *this;
    }

    // for the case that maxIterations_ = 0
    iterations_ = 1;
    if (iterations_ >= parameters_->maxIterations_)
      return *this;

    // Iterative loop that runs Dog Leg
    while (true) {
      double previous_error = error_;
      // do one iteration of LM
      NonlinearOptimizer next = iterateDogLeg();
      writer.write(next.error_);
      error_ = next.error_;
      values_ = next.values_;
      parameters_ = next.parameters_;

      // check convergence
      // TODO: move convergence checks here and incorporate in verbosity levels
      // TODO: build into iterations somehow as an instance variable
      bool converged = gtsam::check_convergence(*parameters_, previous_error, error_);

      if(iterations_ >= parameters_->maxIterations_ || converged == true) {
        if (parameters_->verbosity_ >= Parameters::VALUES) values_->print("final values");
        if (parameters_->verbosity_ >= Parameters::ERROR) cout << "final error: " << error_ << endl;
        if (parameters_->verbosity_ >= Parameters::LAMBDA) cout << "final Delta (called lambda) = " << lambda() << endl;
        return *this;
      }
      iterations_++;
    }

  }

}
