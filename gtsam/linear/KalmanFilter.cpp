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
#include <gtsam/linear/SharedGaussian.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/base/Testable.h>

namespace gtsam {

	using namespace std;

	/// Auxiliary function to solve factor graph and return pointer to root conditional
	GaussianConditional::shared_ptr solve(GaussianFactorGraph& factorGraph,
			bool useQR) {

		// Solve the factor graph
		GaussianSequentialSolver solver(factorGraph, useQR);
		GaussianBayesNet::shared_ptr bayesNet = solver.eliminate();

		// As this is a filter, all we need is the posterior P(x_t),
		// so we just keep the root of the Bayes net
		return bayesNet->back();
	}

	/* ************************************************************************* */
	KalmanFilter::KalmanFilter(size_t n,
			const GaussianConditional::shared_ptr& density, Factorization method) :
			n_(n), I_(eye(n_, n_)), method_(method), density_(density) {
	}

	/* ************************************************************************* */
	KalmanFilter KalmanFilter::add(GaussianFactor* newFactor) {

		// Create a factor graph
		GaussianFactorGraph factorGraph;

		// push back previous solution and new factor
		factorGraph.push_back(density_->toFactor());
		factorGraph.push_back(GaussianFactor::shared_ptr(newFactor));

		// Eliminate graph in order x0, x1, to get Bayes net P(x0|x1)P(x1)
		return KalmanFilter(n_, solve(factorGraph, method_ == QR), method_);
	}

	/* ************************************************************************* */
	KalmanFilter::KalmanFilter(const Vector& x0, const SharedDiagonal& P0,
			Factorization method) :
			n_(x0.size()), I_(eye(n_, n_)), method_(method) {

		// Create a factor graph f(x0), eliminate it into P(x0)
		GaussianFactorGraph factorGraph;
		factorGraph.add(0, I_, x0, P0); // |x-x0|^2_diagSigma
		density_ = solve(factorGraph, method_ == QR);
	}

	/* ************************************************************************* */
	KalmanFilter::KalmanFilter(const Vector& x, const Matrix& P0,
			Factorization method) :
			n_(x.size()), I_(eye(n_, n_)), method_(method) {

		// Create a factor graph f(x0), eliminate it into P(x0)
		GaussianFactorGraph factorGraph;
		// 0.5*(x-x0)'*inv(Sigma)*(x-x0)
		HessianFactor::shared_ptr factor(new HessianFactor(0, x, P0));
		factorGraph.push_back(factor);
		density_ = solve(factorGraph, method_ == QR);
	}

	/* ************************************************************************* */
	void KalmanFilter::print(const string& s) const {
		cout << s << "\n";
		density_->print("density: ");
		Vector m = mean();
		Matrix P = covariance();
		gtsam::print(m, "mean: ");
		gtsam::print(P, "covariance: ");
	}

	/* ************************************************************************* */
	Vector KalmanFilter::mean() const {
		// Solve for mean
		VectorValues x;
		Index k = step();
		// a VectorValues that only has a value for k: cannot be printed
    x.insert(k, Vector(n_));
		density_->rhs(x);
		density_->solveInPlace(x);
		return x[k];
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
	KalmanFilter KalmanFilter::predict(const Matrix& F, const Matrix& B,
			const Vector& u, const SharedDiagonal& model) {

		// The factor related to the motion model is defined as
		// f2(x_{t},x_{t+1}) = (F*x_{t} + B*u - x_{t+1}) * Q^-1 * (F*x_{t} + B*u - x_{t+1})^T
		Index k = step();
		return add(new JacobianFactor(k, -F, k+1, I_, B * u, model));
	}

	/* ************************************************************************* */
	KalmanFilter KalmanFilter::predictQ(const Matrix& F, const Matrix& B,
			const Vector& u, const Matrix& Q) {

#ifndef NDEBUG
		int n = F.cols();
		assert(F.rows() == n);
		assert(B.rows() == n);
		assert(B.cols() == u.size());
		assert(Q.rows() == n);
		assert(Q.cols() == n);
#endif

		// The factor related to the motion model is defined as
		// f2(x_{t},x_{t+1}) = (F*x_{t} + B*u - x_{t+1}) * Q^-1 * (F*x_{t} + B*u - x_{t+1})^T
		// See documentation in HessianFactor, we have A1 = -F,  A2 = I_, b = B*u:
		// TODO: starts to seem more elaborate than straight-up KF equations?
		Matrix M = inverse(Q), Ft = trans(F);
		Matrix G12 = -Ft * M, G11 = -G12 * F, G22 = M;
		Vector b = B * u, g2 = M * b, g1 = -Ft * g2;
		double f = dot(b, g2);
		Index k = step();
		return add(new HessianFactor(k, k+1, G11, G12, g1, G22, g2, f));
	}

	/* ************************************************************************* */
	KalmanFilter KalmanFilter::predict2(const Matrix& A0, const Matrix& A1,
			const Vector& b, const SharedDiagonal& model) {
		// Nhe factor related to the motion model is defined as
		// f2(x_{t},x_{t+1}) = |A0*x_{t} + A1*x_{t+1} - b|^2
		Index k = step();
		return add(new JacobianFactor(k, A0, k+1, A1, b, model));
	}

	/* ************************************************************************* */
	KalmanFilter KalmanFilter::update(const Matrix& H, const Vector& z,
			const SharedDiagonal& model) {
		// The factor related to the measurements would be defined as
		// f2 = (h(x_{t}) - z_{t}) * R^-1 * (h(x_{t}) - z_{t})^T
		//    = (x_{t} - z_{t}) * R^-1 * (x_{t} - z_{t})^T
		Index k = step();
		return add(new JacobianFactor(k, H, z, model));
	}

/* ************************************************************************* */

} // \namespace gtsam

