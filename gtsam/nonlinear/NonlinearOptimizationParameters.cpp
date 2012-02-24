/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file NonlinearOptimizationParameters.cpp
 * @date Jan 28, 2012
 * @author Alex Cunningham
 * @brief Implements parameters structure
 */

#include <iostream>

#include <boost/make_shared.hpp>

#include <gtsam/nonlinear/NonlinearOptimizationParameters.h>

namespace gtsam {

using namespace std;

/* ************************************************************************* */
NonlinearOptimizationParameters::NonlinearOptimizationParameters() :
		absDecrease_(1e-6), relDecrease_(1e-6), sumError_(0.0), maxIterations_(
				100), lambda_(1e-5), lambdaFactor_(10.0), verbosity_(SILENT), lambdaMode_(
						BOUNDED), useQR_(false) {
}

/* ************************************************************************* */
NonlinearOptimizationParameters::NonlinearOptimizationParameters(double absDecrease, double relDecrease,
		double sumError, int iIters, double lambda,
		double lambdaFactor, verbosityLevel v,
		LambdaMode lambdaMode, bool useQR) :
		absDecrease_(absDecrease), relDecrease_(relDecrease), sumError_(
				sumError), maxIterations_(iIters), lambda_(lambda), lambdaFactor_(
						lambdaFactor), verbosity_(v), lambdaMode_(lambdaMode), useQR_(useQR) {
}

/* ************************************************************************* */
NonlinearOptimizationParameters::NonlinearOptimizationParameters(
		const NonlinearOptimizationParameters &parameters) :
		absDecrease_(parameters.absDecrease_), relDecrease_(
				parameters.relDecrease_), sumError_(parameters.sumError_), maxIterations_(
						parameters.maxIterations_), lambda_(parameters.lambda_), lambdaFactor_(
								parameters.lambdaFactor_), verbosity_(parameters.verbosity_), lambdaMode_(
										parameters.lambdaMode_), useQR_(parameters.useQR_) {
}

/* ************************************************************************* */
void NonlinearOptimizationParameters::print(const std::string& s) const {
	cout << "NonlinearOptimizationParameters " << s << endl;
	cout << "absolute decrease threshold: " << absDecrease_ << endl;
	cout << "relative decrease threshold: " << relDecrease_ << endl;
	cout << "        error sum threshold: " << sumError_ << endl;
	cout << "  maximum nr. of iterations: " << maxIterations_ << endl;
	cout << "       initial lambda value: " << lambda_ << endl;
	cout << "  factor to multiply lambda: " << lambdaFactor_ << endl;
	cout << "            verbosity level: " << verbosity_ << endl;
	cout << "                lambda mode: " << lambdaMode_ << endl;
	cout << "                     use QR: " << useQR_ << endl;
}

/* ************************************************************************* */
NonlinearOptimizationParameters::shared_ptr
NonlinearOptimizationParameters::newLambda_(double lambda) const {
	shared_ptr ptr(
			boost::make_shared < NonlinearOptimizationParameters > (*this));
	ptr->lambda_ = lambda;
	return ptr;
}

/* ************************************************************************* */
NonlinearOptimizationParameters::shared_ptr
NonlinearOptimizationParameters::newVerbosity(verbosityLevel verbosity) {
	shared_ptr ptr(boost::make_shared<NonlinearOptimizationParameters>());
	ptr->verbosity_ = verbosity;
	return ptr;
}

/* ************************************************************************* */
NonlinearOptimizationParameters::shared_ptr
NonlinearOptimizationParameters::newMaxIterations(int maxIterations) {
	shared_ptr ptr(boost::make_shared<NonlinearOptimizationParameters>());
	ptr->maxIterations_ = maxIterations;
	return ptr;
}

/* ************************************************************************* */
NonlinearOptimizationParameters::shared_ptr
NonlinearOptimizationParameters::newLambda(double lambda) {
	shared_ptr ptr(boost::make_shared<NonlinearOptimizationParameters>());
	ptr->lambda_ = lambda;
	return ptr;
}

/* ************************************************************************* */
NonlinearOptimizationParameters::shared_ptr
NonlinearOptimizationParameters::newDecreaseThresholds(double absDecrease,
		double relDecrease) {
	shared_ptr ptr(boost::make_shared<NonlinearOptimizationParameters>());
	ptr->absDecrease_ = absDecrease;
	ptr->relDecrease_ = relDecrease;
	return ptr;
}

/* ************************************************************************* */
NonlinearOptimizationParameters::shared_ptr
NonlinearOptimizationParameters::newFactorization(bool useQR) {
	shared_ptr ptr(boost::make_shared<NonlinearOptimizationParameters>());
	ptr->useQR_ = useQR;
	return ptr;
}

} // \namespace gtsam
