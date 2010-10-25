/*
 * NonlinearOptimizationParameters.h
 *
 *   Created on: Oct 14, 2010
 *       Author: nikai
 *  Description:
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

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
		double lambda_ ;
		double lambdaFactor_ ;
		verbosityLevel verbosity_;
		LambdaMode lambdaMode_;

		typedef NonlinearOptimizationParameters This;
		typedef boost::shared_ptr<NonlinearOptimizationParameters> sharedThis ;


		NonlinearOptimizationParameters(): absDecrease_(1e-6), relDecrease_(1e-6), sumError_(0.0),
		maxIterations_(100), lambda_(1e-5), lambdaFactor_(10.0), verbosity_(ERROR), lambdaMode_(BOUNDED){}

		NonlinearOptimizationParameters(double absDecrease, double relDecrease, double sumError,
				int iIters = 100, double lambda = 1e-5, double lambdaFactor = 10, verbosityLevel v = ERROR, LambdaMode lambdaMode = BOUNDED)
		:absDecrease_(absDecrease), relDecrease_(relDecrease), sumError_(sumError),
		 maxIterations_(iIters), lambda_(lambda), lambdaFactor_(lambdaFactor), verbosity_(v), lambdaMode_(lambdaMode){}

		NonlinearOptimizationParameters(const NonlinearOptimizationParameters &parameters):
			absDecrease_(parameters.absDecrease_),
			relDecrease_(parameters.relDecrease_),
			sumError_(parameters.sumError_),
			maxIterations_(parameters.maxIterations_),
			lambda_(parameters.lambda_),
			lambdaFactor_(parameters.lambdaFactor_),
			verbosity_(parameters.verbosity_), lambdaMode_(parameters.lambdaMode_){}


		sharedThis newVerbosity_(verbosityLevel verbosity) const {
			sharedThis ptr (boost::make_shared<NonlinearOptimizationParameters>(*this)) ;
			ptr->verbosity_ = verbosity ;
			return ptr ;
		}

		sharedThis newLambda_(double lambda) const {
			sharedThis ptr (boost::make_shared<NonlinearOptimizationParameters>(*this)) ;
			ptr->lambda_ = lambda ;
			return ptr ;
		}

		sharedThis newMaxIterations_(int maxIterations) const {
			sharedThis ptr (boost::make_shared<NonlinearOptimizationParameters>(*this)) ;
			ptr->maxIterations_ = maxIterations ;
			return ptr ;
		}

		static sharedThis newLambda(double lambda) {
			sharedThis ptr (boost::make_shared<NonlinearOptimizationParameters>());
			ptr->lambda_ = lambda ;
			return ptr ;
		}







	};
}
