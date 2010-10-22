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
		typedef boost::shared_ptr<const GS> shared_solver;
		typedef NonlinearOptimizationParameters Parameters;

	private:

		typedef NonlinearOptimizer<G, T, L, GS> This;
		typedef boost::shared_ptr<const std::vector<size_t> > shared_dimensions;

		// keep a reference to const version of the graph
		// These normally do not change
		const shared_graph graph_;

		// keep a values structure and its error
		// These typically change once per iteration (in a functional way)
		const shared_values values_;
		const double error_;

		// the variable ordering
		const shared_ordering ordering_;

		// the linear system solver
		const shared_solver solver_;

		// keep current lambda for use within LM only
		// TODO: red flag, should we have an LM class ?
		const double lambda_;

		// The dimensions of each linearized variable
		const shared_dimensions dimensions_;

		// Recursively try to do tempered Gauss-Newton steps until we succeed
		NonlinearOptimizer try_lambda(const L& linear,
				Parameters::verbosityLevel verbosity, double factor, Parameters::LambdaMode lambdaMode) const;

    /**
     * Constructor that does not do any computation
     */
    NonlinearOptimizer(shared_graph graph, shared_values values, const double error, shared_ordering ordering,
        shared_solver solver, const double lambda, shared_dimensions dimensions): graph_(graph), values_(values),
        error_(error), ordering_(ordering), solver_(solver), lambda_(lambda), dimensions_(dimensions) {}

    /** Create a new NonlinearOptimizer with a different lambda */
    This newLambda_(double newLambda) const {
      return NonlinearOptimizer(graph_, values_, error_, ordering_, solver_, newLambda, dimensions_); }

    This newValuesSolver_(shared_values newValues, shared_solver newSolver) const {
      return NonlinearOptimizer(graph_, newValues, graph_->error(*newValues), ordering_, newSolver, lambda_, dimensions_); }

    This newValuesSolverLambda_(shared_values newValues, shared_solver newSolver, double newLambda) const {
      return NonlinearOptimizer(graph_, newValues, graph_->error(*newValues), ordering_, newSolver, newLambda, dimensions_); }

	public:

		/**
		 * Constructor that evaluates new error
		 */
		NonlinearOptimizer(shared_graph graph, shared_values values, shared_ordering ordering,
				const double lambda = 1e-5);

		/**
		 * Copy constructor
		 */
		NonlinearOptimizer(const NonlinearOptimizer<G, T, L, GS> &optimizer) :
		  graph_(optimizer.graph_), values_(optimizer.values_), error_(optimizer.error_),
		  ordering_(optimizer.ordering_), solver_(optimizer.solver_), lambda_(optimizer.lambda_), dimensions_(optimizer.dimensions_) {}

		/**
		 * Return current error
		 */
		double error() const {
			return error_;
		}

		/**
		 * Return current lambda
		 */
		double lambda() const {
			return lambda_;
		}

		/**
		 * Return the values
		 */
		shared_values values() const{
			return values_;
		}

		/**
		 *  linearize and optimize
		 *  This returns an VectorValues, i.e., vectors in tangent space of T
		 */
		VectorValues linearizeAndOptimizeForDelta() const;

		/**
		 * Do one Gauss-Newton iteration and return next state
		 */
		NonlinearOptimizer iterate(Parameters::verbosityLevel verbosity = Parameters::SILENT) const;

		/**
		 * Optimize a solution for a non linear factor graph
		 * @param relativeTreshold
		 * @param absoluteTreshold
		 * @param verbosity Integer specifying how much output to provide
		 */
		NonlinearOptimizer
		gaussNewton(double relativeThreshold, double absoluteThreshold,
				Parameters::verbosityLevel verbosity = Parameters::SILENT, int maxIterations = 100) const;

		/**
		 * One iteration of Levenberg Marquardt
		 */
		NonlinearOptimizer iterateLM(Parameters::verbosityLevel verbosity = Parameters::SILENT,
				double lambdaFactor = 10, Parameters::LambdaMode lambdaMode = Parameters::BOUNDED) const;

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
		NonlinearOptimizer
		levenbergMarquardt(double relativeThreshold, double absoluteThreshold,
				Parameters::verbosityLevel verbosity = Parameters::SILENT, int maxIterations = 100,
				double lambdaFactor = 10, Parameters::LambdaMode lambdaMode = Parameters::BOUNDED) const;


		NonlinearOptimizer
		levenbergMarquardt(const NonlinearOptimizationParameters &para) const;

		/**
		 * Static interface to LM optimization using default ordering and thresholds
		 * @param graph 	   Nonlinear factor graph to optimize
		 * @param values       Initial values
		 * @param verbosity    Integer specifying how much output to provide
		 * @return 			   an optimized values structure
		 */
		static shared_values optimizeLM(shared_graph graph, shared_values values,
				Parameters::verbosityLevel verbosity = Parameters::SILENT) {

		  // Use a variable ordering from COLAMD
		  Ordering::shared_ptr ordering = graph->orderingCOLAMD(*values);

			double relativeThreshold = 1e-5, absoluteThreshold = 1e-5;

			// initial optimization state is the same in both cases tested
			GS solver(*graph->linearize(*values, *ordering));
			NonlinearOptimizer optimizer(graph, values, ordering);

			// Levenberg-Marquardt
			NonlinearOptimizer result = optimizer.levenbergMarquardt(relativeThreshold,
					absoluteThreshold, verbosity);
			return result.values();
		}

		/**
		 * Static interface to LM optimization (no shared_ptr arguments) - see above
		 */
		inline static shared_values optimizeLM(const G& graph, const T& values,
				Parameters::verbosityLevel verbosity = Parameters::SILENT) {
			return optimizeLM(boost::make_shared<const G>(graph),
							  boost::make_shared<const T>(values), verbosity);
		}

		/**
		 * Static interface to GN optimization using default ordering and thresholds
		 * @param graph 	   Nonlinear factor graph to optimize
		 * @param values       Initial values
		 * @param verbosity    Integer specifying how much output to provide
		 * @return 			   an optimized values structure
		 */
		static shared_values optimizeGN(shared_graph graph, shared_values values,
				Parameters::verbosityLevel verbosity = Parameters::SILENT) {
      Ordering::shared_ptr ordering = graph->orderingCOLAMD(*values);
			double relativeThreshold = 1e-5, absoluteThreshold = 1e-5;

			// initial optimization state is the same in both cases tested
			GS solver(*graph->linearize(*values, *ordering));
			NonlinearOptimizer optimizer(graph, values, ordering);

			// Gauss-Newton
			NonlinearOptimizer result = optimizer.gaussNewton(relativeThreshold,
					absoluteThreshold, verbosity);
			return result.values();
		}

		/**
		 * Static interface to GN optimization (no shared_ptr arguments) - see above
		 */
		inline static shared_values optimizeGN(const G& graph, const T& values,
				Parameters::verbosityLevel verbosity = Parameters::SILENT) {
			return optimizeGN(boost::make_shared<const G>(graph),
							  boost::make_shared<const T>(values), verbosity);
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


} // gtsam

#endif /* NONLINEAROPTIMIZER_H_ */
