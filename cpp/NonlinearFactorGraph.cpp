/**
 * @file    NonlinearFactorGraph.cpp
 * @brief   Factor Graph Constsiting of non-linear factors
 * @brief   nonlinearFactorGraph
 * @author  Frank Dellaert
 * @author  Carlos Nieto
 * @author  Christian Potthast
 */

#include <math.h>
#include <climits>
#include <stdexcept>
#include <boost/tuple/tuple.hpp>
#include "NonlinearFactorGraph.h" 

using namespace std;
namespace gtsam {

/* ************************************************************************* */
LinearFactorGraph NonlinearFactorGraph::linearize(const FGConfig& config) const
{
	// TODO speed up the function either by returning a pointer or by
	// returning the linearisation as a second argument and returning
	// the reference

	// create an empty linear FG
	LinearFactorGraph linearFG;

	// linearize all factors
	for(const_iterator factor=factors.begin(); factor<factors.end(); factor++){
		LinearFactor::shared_ptr lf = (*factor)->linearize(config);
		linearFG.push_back(lf);
	}

	return linearFG;
}

/* ************************************************************************* */
double calculate_error (const NonlinearFactorGraph& fg, const FGConfig& config, int verbosity) {
	double newError = fg.error(config);
	if (verbosity>=1) cout << "error: " << newError << endl;
	return newError;
}

/* ************************************************************************* */
bool check_convergence (double relativeErrorTreshold,
		double absoluteErrorTreshold,
		double currentError, double newError,
		int verbosity)
{
	// check if diverges
	double absoluteDecrease = currentError - newError;
	if (verbosity>=2) cout << "absoluteDecrease: " << absoluteDecrease << endl;
	if (absoluteDecrease<0)
		throw overflow_error("NonlinearFactorGraph::optimize: error increased, diverges.");

	// calculate relative error decrease and update currentError
	double relativeDecrease = absoluteDecrease/currentError;
	if (verbosity>=2) cout << "relativeDecrease: " << relativeDecrease << endl;
	bool converged =
			(relativeDecrease < relativeErrorTreshold) ||
			(absoluteDecrease < absoluteErrorTreshold);
	if (verbosity>=1 && converged) cout << "converged" << endl;
	return converged;
}

/* ************************************************************************* */
// linearize and solve, return delta config (TODO: return updated)
/* ************************************************************************* */
FGConfig NonlinearFactorGraph::iterate
(const FGConfig& config, const Ordering& ordering) const
{
	LinearFactorGraph linear = linearize(config);   // linearize all factors
	return linear.optimize(ordering);
}

/* ************************************************************************* */
// One iteration of optimize, imperative :-(
/* ************************************************************************* */
double NonlinearFactorGraph::iterate
(FGConfig& config, const Ordering& ordering, int verbosity) const
{
	FGConfig delta = iterate(config, ordering); // linearize and solve
	if (verbosity>=4) delta.print("delta");     // maybe show output
	config += delta;                            // update config
	if (verbosity>=3) config.print("config");   // maybe show output
	return calculate_error(*this,config,verbosity);
}

/* ************************************************************************* */
Ordering NonlinearFactorGraph::getOrdering(FGConfig& config) const
{
	// TODO: FD: Whoa! This is crazy !!!!! re-linearizing just to get ordering ?
	LinearFactorGraph lfg = linearize(config);
	return lfg.getOrdering();
}
/* ************************************************************************* */
void NonlinearFactorGraph::optimize(FGConfig& config,
		const Ordering& ordering,
		double relativeErrorTreshold,
		double absoluteErrorTreshold,
		int verbosity) const
		{
	bool converged = false;
	double currentError = calculate_error(*this,config, verbosity);

	// while not converged
	while(!converged) {
		double newError = iterate(config, ordering, verbosity); // linearize, solve, update
		converged = gtsam::check_convergence(relativeErrorTreshold, absoluteErrorTreshold,
				currentError, newError, verbosity);
		currentError = newError;
	}
		}

/* ************************************************************************* */
bool NonlinearFactorGraph::check_convergence(const FGConfig& config1,
		const FGConfig& config2,
		double relativeErrorTreshold,
		double absoluteErrorTreshold,
		int verbosity)
{
	double currentError = calculate_error(*this, config1, verbosity);
	double newError     = calculate_error(*this, config2, verbosity);
	return gtsam::check_convergence(relativeErrorTreshold, absoluteErrorTreshold,
			currentError, newError, verbosity);

}


/* ************************************************************************* */
// One attempt at a tempered Gauss-Newton step with given lambda
/* ************************************************************************* */
pair<FGConfig,double> NonlinearFactorGraph::try_lambda
(const FGConfig& config, const LinearFactorGraph& linear,
		double lambda, const Ordering& ordering, int verbosity) const
		{
	if (verbosity>=1)
		cout << "trying lambda = " << lambda << endl;
	LinearFactorGraph damped =
			linear.add_priors(sqrt(lambda));                 // add prior-factors
	if (verbosity>=7) damped.print("damped");
	FGConfig delta = damped.optimize(ordering);        // solve
	if (verbosity>=5) delta.print("delta");
	FGConfig newConfig = config + delta;               // update config
	if (verbosity>=5) newConfig.print("config");
	double newError =                                  // calculate...
			calculate_error(*this,newConfig,verbosity>=4);   // ...new error
	return make_pair(newConfig,newError);              // return config and error
		}

/* ************************************************************************* */
// One iteration of Levenberg Marquardt, imperative :-(
/* ************************************************************************* */
double NonlinearFactorGraph::iterateLM
(FGConfig& config, double currentError, double& lambda, double lambdaFactor,
		const Ordering& ordering, int verbosity) const
		{
	FGConfig newConfig;
	double newError;
	LinearFactorGraph linear = linearize(config); // linearize all factors once
	if (verbosity>=6) linear.print("linear");

	// try lambda steps with successively larger lambda until we achieve descent
	boost::tie(newConfig,newError) = try_lambda(config, linear, lambda, ordering,verbosity);
	while (newError > currentError) {
		lambda = lambda * lambdaFactor; // be more cautious
		boost::tie(newConfig,newError) = try_lambda(config, linear, lambda, ordering,verbosity);
	}

	// next time, let's be more adventerous
	lambda = lambda / lambdaFactor;

	// return result of this iteration
	config = newConfig;
	if (verbosity>=4) config.print("config"); // maybe show output
	if (verbosity>=1) cout << "error: " << newError << endl;
	return newError;
		}

/* ************************************************************************* */
void NonlinearFactorGraph::optimizeLM(FGConfig& config,
		const Ordering& ordering,
		double relativeErrorTreshold,
		double absoluteErrorTreshold,
		int verbosity,
		double lambda0,
		double lambdaFactor) const
		{
	double lambda = lambda0;
	bool converged = false;
	double currentError = calculate_error(*this,config, verbosity);

	if (verbosity>=4) config.print("config"); // maybe show output

	// while not converged
	while(!converged) {
		// linearize, solve, update
		double newError = iterateLM(config, currentError, lambda, lambdaFactor, ordering, verbosity);
		converged = gtsam::check_convergence(relativeErrorTreshold, absoluteErrorTreshold,
				currentError, newError, verbosity);
		currentError = newError;
	}
		}

/* ************************************************************************* */

pair<LinearFactorGraph, FGConfig> NonlinearFactorGraph::OneIterationLM( FGConfig& config,
		const Ordering& ordering,
		double relativeErrorTreshold,
		double absoluteErrorTreshold,
		int verbosity,
		double lambda0,
		double lambdaFactor) {

	double lambda = lambda0;
	bool converged = false;
	double currentError = calculate_error(*this,config, verbosity);

	if (verbosity>=4) config.print("config"); // maybe show output

	FGConfig newConfig;
	double newError;
	LinearFactorGraph linear = linearize(config); // linearize all factors once
	if (verbosity>=6) linear.print("linear");

	// try lambda steps with successively larger lambda until we achieve descent
	boost::tie(newConfig,newError) = try_lambda(config, linear, lambda, ordering,verbosity);
	while (newError > currentError) {
		lambda = lambda * lambdaFactor; // be more cautious
		boost::tie(newConfig,newError) = try_lambda(config, linear, lambda, ordering,verbosity);
	}

	// next time, let's be more adventerous
	lambda = lambda / lambdaFactor;

	// return result of this iteration
	config = newConfig;
	if (verbosity>=4) config.print("config"); // maybe show output
	if (verbosity>=1) cout << "error: " << newError << endl;

	pair<LinearFactorGraph, FGConfig> p(linear, newConfig);
	return p;



	// linearize, solve, update
	//double newError = iterateLM(config, currentError, lambda, lambdaFactor, ordering, verbosity);

}

/* ************************************************************************* */


} // namespace gtsam
