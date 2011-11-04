/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file KalmanFilter.cpp
 *
 * @brief Simple linear Kalman filter.
 * Implemented using factor graphs, i.e., does LDL-based SRIF, really.
 *
 * @date Sep 3, 2011
 * @author Stephen Williams
 * @author Frank Dellaert
 */

#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/KalmanFilter.h>

namespace gtsam {

	/// Auxiliary function to solve factor graph and return pointer to root conditional
	GaussianConditional* solve(GaussianFactorGraph& factorGraph) {

		// Solve the factor graph
		const bool useQR = true; // make sure we use QR (numerically stable)
		GaussianSequentialSolver solver(factorGraph, useQR);
		GaussianBayesNet::shared_ptr bayesNet = solver.eliminate();

		// As this is a filter, all we need is the posterior P(x_t),
		// so we just keep the root of the Bayes net
		// We need to create a new density, because we always keep the index at 0
		const GaussianConditional::shared_ptr& r = bayesNet->back();
		return new GaussianConditional(0, r->get_d(), r->get_R(), r->get_sigmas());
	}

	/* ************************************************************************* */
	KalmanFilter::KalmanFilter(const Vector& x, const SharedDiagonal& model) :
			n_(x.size()), I_(eye(n_, n_)) {

		// Create a factor graph f(x0), eliminate it into P(x0)
		GaussianFactorGraph factorGraph;
		factorGraph.add(0, I_, x, model);
		density_.reset(solve(factorGraph));
	}

	/* ************************************************************************* */
	Vector KalmanFilter::mean() const {
		// Solve for mean
		Index nVars = 1;
		VectorValues x(nVars, n_);
		density_->rhs(x);
		density_->solveInPlace(x);
		return x[0];
	}

	/* ************************************************************************* */
	Matrix KalmanFilter::information() const {
		return density_->computeInformation();
	}

	/* ************************************************************************* */
	Matrix KalmanFilter::covariance() const {
		return inverse(information());
	}

	/* ************************************************************************* */
	void KalmanFilter::predict(const Matrix& F, const Matrix& B, const Vector& u,
			const SharedDiagonal& model) {
		// We will create a small factor graph f1-(x0)-f2-(x1)
		// where factor f1 is just the prior from time t0, P(x0)
		// and   factor f2 is from the motion model
		GaussianFactorGraph factorGraph;

		// push back f1
		factorGraph.push_back(density_->toFactor());

		// The factor related to the motion model is defined as
		// f2(x_{t},x_{t+1}) = (F*x_{t} + B*u - x_{t+1}) * Q^-1 * (F*x_{t} + B*u - x_{t+1})^T
		factorGraph.add(0, -F, 1, I_, B * u, model);

		// Eliminate graph in order x0, x1, to get Bayes net P(x0|x1)P(x1)
		density_.reset(solve(factorGraph));
	}

	/* ************************************************************************* */
	void KalmanFilter::predict2(const Matrix& A0, const Matrix& A1, const Vector& b,
			const SharedDiagonal& model) {

		// Exactly the same schem as in predict:
		GaussianFactorGraph factorGraph;
		factorGraph.push_back(density_->toFactor());

		// However, now the factor related to the motion model is defined as
		// f2(x_{t},x_{t+1}) = |A0*x_{t} + A1*x_{t+1} - b|^2
		factorGraph.add(0, A0, 1, A1, b, model);
		density_.reset(solve(factorGraph));
	}

	/* ************************************************************************* */
	void KalmanFilter::update(const Matrix& H, const Vector& z,
			const SharedDiagonal& model) {
		// We will create a small factor graph f1-(x0)-f2
		// where factor f1 is the predictive density
		// and   factor f2 is from the measurement model
		GaussianFactorGraph factorGraph;

		// push back f1
		factorGraph.push_back(density_->toFactor());

		// The factor related to the measurements would be defined as
		// f2 = (h(x_{t}) - z_{t}) * R^-1 * (h(x_{t}) - z_{t})^T
		//    = (x_{t} - z_{t}) * R^-1 * (x_{t} - z_{t})^T
		factorGraph.add(0, H, z, model);

		// Eliminate graph in order x0, x1, to get Bayes net P(x0|x1)P(x1)
		density_.reset(solve(factorGraph));
	}

/* ************************************************************************* */

} // \namespace gtsam

