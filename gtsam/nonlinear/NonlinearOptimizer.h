/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * NonlinearOptimizer.h
 * @brief: Encapsulates nonlinear optimization state
 * @Author: Frank Dellaert
 * Created on: Sep 7, 2009
 */

#ifndef NONLINEAROPTIMIZER_H_
#define NONLINEAROPTIMIZER_H_

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/GaussianMultifrontalSolver.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizationParameters.h>

namespace gtsam {

	class NullOptimizerWriter {
	public:
		NullOptimizerWriter(double error) {}
		virtual void write(double error) {}
	};

	/**
	 * The class NonlinearOptimizer encapsulates an optimization state.
	 * Typically it is instantiated with a NonlinearFactorGraph and an initial values
	 * and then one of the optimization routines is called. These recursively iterate
	 * until convergence. All methods are functional and return a new state.
	 *
	 * The class is parameterized by the Graph type $G$, Values class type $T$,
	 * linear system class $L$, the non linear solver type $S$, and the writer type $W$
	 *
	 * The values class type $T$ is in order to be able to optimize over non-vector values structures.
	 *
	 * A nonlinear system solver $S$
   * Concept NonLinearSolver<G,T,L> implements
   *   linearize: G * T -> L
   *   solve : L -> T
	 *
	 * The writer $W$ generates output to disk or the screen.
	 *
	 * For example, in a 2D case, $G$ can be Pose2Graph, $T$ can be Pose2Values,
	 * $L$ can be GaussianFactorGraph and $S$ can be Factorization<Pose2Graph, Pose2Values>.
	 * The solver class has two main functions: linearize and optimize. The first one
	 * linearizes the nonlinear cost function around the current estimate, and the second
	 * one optimizes the linearized system using various methods.
	 *
	 * To use the optimizer in code, include <gtsam/NonlinearOptimizer-inl.h> in your cpp file
	 *
	 *
	 */
	template<class G, class T, class L = GaussianFactorGraph, class GS = GaussianMultifrontalSolver, class W = NullOptimizerWriter>
	class NonlinearOptimizer {
	public:

		// For performance reasons in recursion, we store values in a shared_ptr
		typedef boost::shared_ptr<const T> shared_values;
		typedef boost::shared_ptr<const G> shared_graph;
		typedef boost::shared_ptr<const Ordering> shared_ordering;
		typedef boost::shared_ptr<GS> shared_solver;
		typedef NonlinearOptimizationParameters Parameters;
		typedef boost::shared_ptr<const Parameters> shared_parameters ;

	private:

		typedef NonlinearOptimizer<G, T, L, GS> This;
		typedef boost::shared_ptr<const std::vector<size_t> > shared_dimensions;

		// keep a reference to const version of the graph
		// These normally do not change
		const shared_graph graph_;

		// keep a values structure and its error
		// These typically change once per iteration (in a functional way)
		shared_values values_;
		double error_;

		// the variable ordering
		const shared_ordering ordering_;

		// the linear system solver
		shared_solver solver_;

		shared_parameters parameters_;

		// for performance track
		size_t iterations_;

//		// keep current lambda for use within LM only
//		// TODO: red flag, should we have an LM class ?
//		const double lambda_;

		// The dimensions of each linearized variable
		const shared_dimensions dimensions_;

		// Recursively try to do tempered Gauss-Newton steps until we succeed
//		NonlinearOptimizer try_lambda(const L& linear,
//				Parameters::verbosityLevel verbosity, double factor, Parameters::LambdaMode lambdaMode) const;
		NonlinearOptimizer try_lambda(L& linear);

    /**
     * Constructor that does not do any computation
     */
//    NonlinearOptimizer(shared_graph graph, shared_values values, const double error, shared_ordering ordering,
//        shared_solver solver, const double lambda, shared_dimensions dimensions): graph_(graph), values_(values),
//        error_(error), ordering_(ordering), solver_(solver), lambda_(lambda), dimensions_(dimensions) {}

	NonlinearOptimizer(shared_graph graph, shared_values values, const double error, shared_ordering ordering,
		shared_solver solver, shared_parameters parameters, shared_dimensions dimensions): graph_(graph), values_(values),
		error_(error), ordering_(ordering), solver_(solver), parameters_(parameters), dimensions_(dimensions) {}

    /** Create a new NonlinearOptimizer with a different lambda */
    This newValuesSolver_(shared_values newValues, shared_solver newSolver) const {
      return NonlinearOptimizer(graph_, newValues, graph_->error(*newValues), ordering_, newSolver, parameters_, dimensions_); }

    This newValuesErrorLambda_(shared_values newValues, double newError, double newLambda) const {
      return NonlinearOptimizer(graph_, newValues, newError, ordering_, solver_, parameters_->newLambda_(newLambda), dimensions_); }


	/*
    This newLambda_(double newLambda) const {
      return NonlinearOptimizer(graph_, values_, error_, ordering_, solver_, parameters_->newLambda_(newLambda), dimensions_); }

    This newValuesSolver_(shared_values newValues, shared_solver newSolver) const {
      return NonlinearOptimizer(graph_, newValues, graph_->error(*newValues), ordering_, newSolver, parameters_, dimensions_); }

    This newValuesSolverLambda_(shared_values newValues, shared_solver newSolver, double newLambda) const {
      return NonlinearOptimizer(graph_, newValues, graph_->error(*newValues), ordering_, newSolver, parameters_->newLambda_(newLambda), dimensions_); }

    This newValuesErrorLambda_(shared_values newValues, double newError, double newLambda) const {
         return NonlinearOptimizer(graph_, newValues, newError, ordering_, solver_, parameters_->newLambda_(newLambda), dimensions_); }


    This newVerbosity_(Parameters::verbosityLevel verbosity) const {
      return NonlinearOptimizer(graph_, values_, error_, ordering_, solver_, parameters_->newVerbosity_(verbosity), dimensions_); }

    This newParameters_(shared_parameters parameters) const {
      return NonlinearOptimizer(graph_, values_, error_, ordering_, solver_, parameters, dimensions_); }

    This newMaxIterations_(int maxIterations) const {
      return NonlinearOptimizer(graph_, values_, error_, ordering_, solver_, parameters_->newMaxIterations_(maxIterations), dimensions_); }
    */

	public:
		/**
		 * Constructor that evaluates new error
		 */

		// suggested constructors
		NonlinearOptimizer(shared_graph graph,
						   shared_values values,
						   shared_ordering ordering,
						   shared_parameters parameters = boost::make_shared<Parameters>());

		// suggested constructors
		NonlinearOptimizer(shared_graph graph,
						   shared_values values,
						   shared_ordering ordering,
						   shared_solver solver,
						   shared_parameters parameters = boost::make_shared<Parameters>());

		/**
		 * Copy constructor
		 */
//		NonlinearOptimizer(const NonlinearOptimizer<G, T, L, GS> &optimizer) :
//		  graph_(optimizer.graph_), values_(optimizer.values_), error_(optimizer.error_),
//		  ordering_(optimizer.ordering_), solver_(optimizer.solver_), lambda_(optimizer.lambda_), dimensions_(optimizer.dimensions_) {}

		NonlinearOptimizer(const NonlinearOptimizer<G, T, L, GS> &optimizer) :
		  graph_(optimizer.graph_), values_(optimizer.values_), error_(optimizer.error_),
		  ordering_(optimizer.ordering_), solver_(optimizer.solver_),
		  parameters_(optimizer.parameters_), iterations_(0), dimensions_(optimizer.dimensions_) {}

		/**
		 * Return current error
		 */
		double error() const { return error_; }

		/**
		 * Return current lambda
		 */
		double lambda() const {	return parameters_->lambda_; }

		/**
		 * Return the values
		 */
		shared_values values() const{ return values_; }

		/**
		 * Return the itertions
		 */
		size_t iterations() const { return iterations_; }

		/**
		 * Return mean and covariance on a single variable
		 */
		std::pair<Vector,Matrix> marginalCovariance(Symbol j) const {
			return solver_->marginalCovariance((*ordering_)[j]);
		}

		/**
		 *  linearize and optimize
		 *  This returns an VectorValues, i.e., vectors in tangent space of T
		 */
		VectorValues linearizeAndOptimizeForDelta() const;

		/**
		 * Do one Gauss-Newton iteration and return next state
		 */

		// suggested interface
		NonlinearOptimizer iterate() const;

		/**
		 * Optimize a solution for a non linear factor graph
		 * @param relativeTreshold
		 * @param absoluteTreshold
		 * @param verbosity Integer specifying how much output to provide
		 */

		// suggested interface
		NonlinearOptimizer gaussNewton() const;

		/**
		 * One iteration of Levenberg Marquardt
		 */

		// suggested interface
		NonlinearOptimizer iterateLM();

		/**
		 * Optimize using Levenberg-Marquardt. Really Levenberg's
		 * algorithm at this moment, as we just add I*\lambda to Hessian
		 * H'H. The probabilistic explanation is very simple: every
		 * variable gets an extra Gaussian prior that biases staying at
		 * current value, with variance 1/lambda. This is done very easily
		 * (but perhaps wastefully) by adding a prior factor for each of
		 * the variables, after linearization.
		 *
		 * @param relativeThreshold
		 * @param absoluteThreshold
		 * @param verbosity    Integer specifying how much output to provide
		 * @param lambdaFactor Factor by which to decrease/increase lambda
		 */

		// suggested interface
		NonlinearOptimizer levenbergMarquardt();

		/**
		 * Static interface to LM optimization using default ordering and thresholds
		 * @param graph 	   Nonlinear factor graph to optimize
		 * @param values       Initial values
		 * @param verbosity    Integer specifying how much output to provide
		 * @return 			   an optimized values structure
		 */

		static shared_values optimizeLM(shared_graph graph,
										shared_values values,
										shared_parameters parameters = boost::make_shared<Parameters>()) {

			// Use a variable ordering from COLAMD
			Ordering::shared_ptr ordering = graph->orderingCOLAMD(*values);
			// initial optimization state is the same in both cases tested
			//GS solver(*graph->linearize(*values, *ordering));

			NonlinearOptimizer optimizer(graph, values, ordering, parameters);
			NonlinearOptimizer result = optimizer.levenbergMarquardt();
			return result.values();
		}

		static shared_values optimizeLM(shared_graph graph,
										shared_values values,
										Parameters::verbosityLevel verbosity)
		{
			return optimizeLM(graph, values, Parameters::newVerbosity(verbosity));
		}
		/**
		 * Static interface to LM optimization (no shared_ptr arguments) - see above
		 */

		static shared_values optimizeLM(const G& graph,
										const T& values,
										const Parameters parameters = Parameters()) {
			return optimizeLM(boost::make_shared<const G>(graph),
							  boost::make_shared<const T>(values),
							  boost::make_shared<Parameters>(parameters));
		}

		static shared_values optimizeLM(const G& graph,
										const T& values,
										Parameters::verbosityLevel verbosity) {
			return optimizeLM(boost::make_shared<const G>(graph),
							  boost::make_shared<const T>(values),
							  verbosity);
		}

		/**
		 * Static interface to GN optimization using default ordering and thresholds
		 * @param graph 	   Nonlinear factor graph to optimize
		 * @param values       Initial values
		 * @param verbosity    Integer specifying how much output to provide
		 * @return 			   an optimized values structure
		 */

		// suggested interface
		static shared_values optimizeGN(shared_graph graph,
										shared_values values,
										shared_parameters parameters = boost::make_shared<Parameters>()) {

			Ordering::shared_ptr ordering = graph->orderingCOLAMD(*values);
			// initial optimization state is the same in both cases tested
			GS solver(*graph->linearize(*values, *ordering));

			NonlinearOptimizer optimizer(graph, values, ordering, parameters);
			NonlinearOptimizer result = optimizer.gaussNewton();
			return result.values();
		}

		/**
		 * Static interface to GN optimization (no shared_ptr arguments) - see above
		 */

		// suggested interface
		static shared_values optimizeGN(const G& graph, const T& values, const Parameters parameters = Parameters()) {
			return optimizeGN(boost::make_shared<const G>(graph),
							  boost::make_shared<const T>(values),
							  boost::make_shared<Parameters>(parameters));
		}
	};

	/**
	 * Check convergence
	 */
	bool check_convergence (
			double relativeErrorTreshold,
			double absoluteErrorTreshold,
			double errorThreshold,
			double currentError, double newError, int verbosity);

	bool check_convergence (
			const NonlinearOptimizationParameters &parameters,
			double currentError, double newError);

} // gtsam

#endif /* NONLINEAROPTIMIZER_H_ */
