/*
 * IterativeOptimizationParameters.h
 *
 *  Created on: Oct 22, 2010
 *      Author: Yong-Dian Jian
 */

#pragma once

namespace gtsam {

	// a container for all related parameters
	struct IterativeOptimizationParameters {

	public:

		typedef enum {
			SILENT,
			ERROR,
		} verbosityLevel;

	protected:
		int maxIterations_;
		int reset_ ; // number of iterations before reset, for cg and gmres
		double epsilon_; // relative error
		double epsilon_abs_; // absolute error
		verbosityLevel verbosity_;

	public:
		IterativeOptimizationParameters():
			maxIterations_(100),
			reset_(101),
			epsilon_(1e-5),
			epsilon_abs_(1e-5),
			verbosity_(ERROR) {}

		IterativeOptimizationParameters(const IterativeOptimizationParameters &parameters):
			maxIterations_(parameters.maxIterations_),
			reset_(parameters.reset_),
			epsilon_(parameters.epsilon_),
			epsilon_abs_(parameters.epsilon_abs_),
			verbosity_(parameters.verbosity_) {}


		IterativeOptimizationParameters
		(int maxIterations, double epsilon, double epsilon_abs, verbosityLevel verbosity=ERROR, int reset=-1):
		maxIterations_(maxIterations), reset_(reset),
		epsilon_(epsilon), epsilon_abs_(epsilon_abs), verbosity_(verbosity) {
			if (reset_==-1) reset_ = maxIterations_ + 1 ;
		}

		int maxIterations() const { return maxIterations_; }
		int reset() const { return reset_; }
		double epsilon() const { return epsilon_ ;}
		double epsilon_abs() const { return epsilon_abs_ ; }
		verbosityLevel verbosity() const { return verbosity_ ; }

	};
}
