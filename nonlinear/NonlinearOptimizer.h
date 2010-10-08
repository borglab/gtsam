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
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/VectorConfig.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/Factorization.h>

namespace gtsam {

	class NullOptimizerWriter {
	public:
		NullOptimizerWriter(double error) {}
		virtual void write(double error) {}
	};

	/**
	 * The class NonlinearOptimizer encapsulates an optimization state.
	 * Typically it is instantiated with a NonlinearFactorGraph and an initial config
	 * and then one of the optimization routines is called. These recursively iterate
	 * until convergence. All methods are functional and return a new state.
	 *
	 * The class is parameterized by the Graph type $G$, Config class type $T$,
	 * linear system class $L$ and the non linear solver type $S$.
	 * the config type is in order to be able to optimize over non-vector configurations.
	 * To use in code, include <gtsam/NonlinearOptimizer-inl.h> in your cpp file
	 *
	 * For example, in a 2D case, $G$ can be Pose2Graph, $T$ can be Pose2Config,
	 * $L$ can be GaussianFactorGraph and $S$ can be Factorization<Pose2Graph, Pose2Config>.
	 * The solver class has two main functions: linearize and optimize. The first one
	 * linearizes the nonlinear cost function around the current estimate, and the second
	 * one optimizes the linearized system using various methods.
	 */
	template<class G, class T, class L = GaussianFactorGraph, class S = Factorization<G, T>, class Writer = NullOptimizerWriter>
	class NonlinearOptimizer {
	public:

		// For performance reasons in recursion, we store configs in a shared_ptr
		typedef boost::shared_ptr<const T> shared_config;
		typedef boost::shared_ptr<const G> shared_graph;
		typedef boost::shared_ptr<const S> shared_solver;
		typedef const S solver;

		typedef enum {
			SILENT,
			ERROR,
			LAMBDA,
			TRYLAMBDA,
			CONFIG,
			DELTA,
			TRYCONFIG,
			TRYDELTA,
			LINEAR,
			DAMPED
		} verbosityLevel;

		typedef enum {
			FAST,
			BOUNDED,
			CAUTIOUS
		} LambdaMode;

		// a container for all related parameters
		struct NonLinearOptimizerPara {
		public:
			double absDecrease_; /* threshold for the absolute decrease per iteration */
			double relDecrease_; /* threshold for the relative decrease per iteration */
			double sumError_; /* threshold for the sum of error */
			int maxIterations_ ;
			double lambdaFactor_ ;
			verbosityLevel verbosity_;
			LambdaMode lambdaMode_;

		public:

			NonLinearOptimizerPara(): absDecrease_(1), relDecrease_(1e-3), sumError_(0.0),
				maxIterations_(100), lambdaFactor_(10.0), verbosity_(ERROR), lambdaMode_(BOUNDED){}

			NonLinearOptimizerPara(double absDecrease, double relDecrease, double sumError,
					int iIters = 100, double lambdaFactor = 10, verbosityLevel v = ERROR, LambdaMode lambdaMode = BOUNDED)
			:absDecrease_(absDecrease), relDecrease_(relDecrease), sumError_(sumError),
			 maxIterations_(iIters), lambdaFactor_(lambdaFactor), verbosity_(v), lambdaMode_(lambdaMode){}

		};


	private:

		// keep a reference to const version of the graph
		// These normally do not change
		const shared_graph graph_;

		// keep a configuration and its error
		// These typically change once per iteration (in a functional way)
		const shared_config config_;
		double error_; // TODO FD: no more const because in constructor I need to set it after checking :-(

		// keep current lambda for use within LM only
		// TODO: red flag, should we have an LM class ?
		const double lambda_;

		// the linear system solver
		const shared_solver solver_;

		// Recursively try to do tempered Gauss-Newton steps until we succeed
		NonlinearOptimizer try_lambda(const L& linear,
				verbosityLevel verbosity, double factor, LambdaMode lambdaMode) const;

	public:

		/**
		 * Constructor that evaluates new error
		 */
		NonlinearOptimizer(shared_graph graph, shared_config config, shared_solver solver,
				const double lambda = 1e-5);

		/**
		 * Constructor that does not do any computation
		 */
		NonlinearOptimizer(shared_graph graph, shared_config config, shared_solver solver,
				const double error, const double lambda): graph_(graph), config_(config),
			  error_(error), lambda_(lambda), solver_(solver) {}

		/**
		 * Copy constructor
		 */
		NonlinearOptimizer(const NonlinearOptimizer<G, T, L, S> &optimizer) :
		  graph_(optimizer.graph_), config_(optimizer.config_),
		  error_(optimizer.error_), lambda_(optimizer.lambda_), solver_(optimizer.solver_) {}

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
		 * Return the config
		 */
		shared_config config() const{
			return config_;
		}

		/**
		 *  linearize and optimize
		 *  This returns an VectorConfig, i.e., vectors in tangent space of T
		 */
		VectorConfig linearizeAndOptimizeForDelta() const;

		/**
		 * Do one Gauss-Newton iteration and return next state
		 */
		NonlinearOptimizer iterate(verbosityLevel verbosity = SILENT) const;

		/**
		 * Optimize a solution for a non linear factor graph
		 * @param relativeTreshold
		 * @param absoluteTreshold
		 * @param verbosity Integer specifying how much output to provide
		 */
		NonlinearOptimizer
		gaussNewton(double relativeThreshold, double absoluteThreshold,
				verbosityLevel verbosity = SILENT, int maxIterations = 100) const;

		/**
		 * One iteration of Levenberg Marquardt
		 */
		NonlinearOptimizer iterateLM(verbosityLevel verbosity = SILENT,
				double lambdaFactor = 10, LambdaMode lambdaMode = BOUNDED) const;

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
				verbosityLevel verbosity = SILENT, int maxIterations = 100,
				double lambdaFactor = 10, LambdaMode lambdaMode = BOUNDED) const;


		NonlinearOptimizer
		levenbergMarquardt(const NonLinearOptimizerPara &para) const;

		/**
		 * Static interface to LM optimization using default ordering and thresholds
		 * @param graph 	   Nonlinear factor graph to optimize
		 * @param config       Initial config
		 * @param verbosity    Integer specifying how much output to provide
		 * @return 			   an optimized configuration
		 */
		static shared_config optimizeLM(shared_graph graph, shared_config config,
				verbosityLevel verbosity = SILENT) {
			boost::shared_ptr<gtsam::Ordering> ord(new gtsam::Ordering(graph->getOrdering()));
			double relativeThreshold = 1e-5, absoluteThreshold = 1e-5;

			// initial optimization state is the same in both cases tested
			shared_solver solver(new S(ord, false));
			NonlinearOptimizer optimizer(graph, config, solver);

			// Levenberg-Marquardt
			NonlinearOptimizer result = optimizer.levenbergMarquardt(relativeThreshold,
					absoluteThreshold, verbosity);
			return result.config();
		}

		/**
		 * Static interface to LM optimization (no shared_ptr arguments) - see above
		 */
		inline static shared_config optimizeLM(const G& graph, const T& config,
				verbosityLevel verbosity = SILENT) {
			return optimizeLM(boost::make_shared<const G>(graph),
							  boost::make_shared<const T>(config), verbosity);
		}

		/**
		 * Static interface to GN optimization using default ordering and thresholds
		 * @param graph 	   Nonlinear factor graph to optimize
		 * @param config       Initial config
		 * @param verbosity    Integer specifying how much output to provide
		 * @return 			   an optimized configuration
		 */
		static shared_config optimizeGN(shared_graph graph, shared_config config,
				verbosityLevel verbosity = SILENT) {
			boost::shared_ptr<gtsam::Ordering> ord(new gtsam::Ordering(graph->getOrdering()));
			double relativeThreshold = 1e-5, absoluteThreshold = 1e-5;

			// initial optimization state is the same in both cases tested
			shared_solver solver(new S(ord, false));
			NonlinearOptimizer optimizer(graph, config, solver);

			// Gauss-Newton
			NonlinearOptimizer result = optimizer.gaussNewton(relativeThreshold,
					absoluteThreshold, verbosity);
			return result.config();
		}

		/**
		 * Static interface to GN optimization (no shared_ptr arguments) - see above
		 */
		inline static shared_config optimizeGN(const G& graph, const T& config,
				verbosityLevel verbosity = SILENT) {
			return optimizeGN(boost::make_shared<const G>(graph),
							  boost::make_shared<const T>(config), verbosity);
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
