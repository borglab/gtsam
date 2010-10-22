/*
 * NonlinearOptimizationParameters.h
 *
 *   Created on: Oct 14, 2010
 *       Author: nikai
 *  Description:
 */

#pragma once

namespace gtsam {

	// a container for all related parameters
	struct NonlinearOptimizationParameters {
		typedef enum {
			SILENT,
			ERROR,
			LAMBDA,
			TRYLAMBDA,
			VALUES,
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

		double absDecrease_; /* threshold for the absolute decrease per iteration */
		double relDecrease_; /* threshold for the relative decrease per iteration */
		double sumError_; /* threshold for the sum of error */
		int maxIterations_ ;
		double lambdaFactor_ ;
		verbosityLevel verbosity_;
		LambdaMode lambdaMode_;

		NonlinearOptimizationParameters(): absDecrease_(1), relDecrease_(1e-3), sumError_(0.0),
		maxIterations_(100), lambdaFactor_(10.0), verbosity_(ERROR), lambdaMode_(BOUNDED){}

		NonlinearOptimizationParameters(double absDecrease, double relDecrease, double sumError,
				int iIters = 100, double lambdaFactor = 10, verbosityLevel v = ERROR, LambdaMode lambdaMode = BOUNDED)
		:absDecrease_(absDecrease), relDecrease_(relDecrease), sumError_(sumError),
		 maxIterations_(iIters), lambdaFactor_(lambdaFactor), verbosity_(v), lambdaMode_(lambdaMode){}

	};
}
