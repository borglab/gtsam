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
 * Implemented using factor graphs, i.e., does Cholesky-based SRIF, really.
 *
 * @date Sep 3, 2011
 * @author Stephen Williams
 * @author Frank Dellaert
 */

#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/KalmanFilter.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/base/Testable.h>

#include <boost/make_shared.hpp>

namespace gtsam {

	using namespace std;

	/// Auxiliary function to solve factor graph and return pointer to root conditional
	KalmanFilter::State solve(const GaussianFactorGraph& factorGraph,
			bool useQR) {

		// Solve the factor graph
		GaussianSequentialSolver solver(factorGraph, useQR);
		GaussianBayesNet::shared_ptr bayesNet = solver.eliminate();

		// As this is a filter, all we need is the posterior P(x_t),
		// so we just keep the root of the Bayes net
		GaussianConditional::shared_ptr conditional = bayesNet->back();
		// TODO: awful ! A copy constructor followed by ANOTHER copy constructor in make_shared?
		return boost::make_shared<GaussianDensity>(*conditional);
	}

	/* ************************************************************************* */
	KalmanFilter::State fuse(const KalmanFilter::State& p,
			GaussianFactor* newFactor, bool useQR) {

		// Create a factor graph
		GaussianFactorGraph factorGraph;

		// push back previous solution and new factor
		factorGraph.push_back(p->toFactor());
		factorGraph.push_back(GaussianFactor::shared_ptr(newFactor));

		// Eliminate graph in order x0, x1, to get Bayes net P(x0|x1)P(x1)
		return solve(factorGraph, useQR);
	}

	/* ************************************************************************* */
	KalmanFilter::State KalmanFilter::init(const Vector& x0,
			const SharedDiagonal& P0) {

		// Create a factor graph f(x0), eliminate it into P(x0)
		GaussianFactorGraph factorGraph;
		factorGraph.add(0, I_, x0, P0); // |x-x0|^2_diagSigma
		return solve(factorGraph, useQR());
	}

	/* ************************************************************************* */
	KalmanFilter::State KalmanFilter::init(const Vector& x, const Matrix& P0) {

		// Create a factor graph f(x0), eliminate it into P(x0)
		GaussianFactorGraph factorGraph;
		// 0.5*(x-x0)'*inv(Sigma)*(x-x0)
		HessianFactor::shared_ptr factor(new HessianFactor(0, x, P0));
		factorGraph.push_back(factor);
		return solve(factorGraph, useQR());
	}

	/* ************************************************************************* */
	void KalmanFilter::print(const string& s) const {
		cout << "KalmanFilter " << s << ", dim = " << n_ << endl;
	}

	/* ************************************************************************* */
	KalmanFilter::State KalmanFilter::predict(const State& p, const Matrix& F,
			const Matrix& B, const Vector& u, const SharedDiagonal& model) {

		// The factor related to the motion model is defined as
		// f2(x_{t},x_{t+1}) = (F*x_{t} + B*u - x_{t+1}) * Q^-1 * (F*x_{t} + B*u - x_{t+1})^T
		Index k = step(p);
		return fuse(p, new JacobianFactor(k, -F, k + 1, I_, B * u, model), useQR());
	}

	/* ************************************************************************* */
	KalmanFilter::State KalmanFilter::predictQ(const State& p, const Matrix& F,
			const Matrix& B, const Vector& u, const Matrix& Q) {

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
		Index k = step(p);
		return fuse(p, new HessianFactor(k, k + 1, G11, G12, g1, G22, g2, f),
				useQR());
	}

	/* ************************************************************************* */
	KalmanFilter::State KalmanFilter::predict2(const State& p, const Matrix& A0,
			const Matrix& A1, const Vector& b, const SharedDiagonal& model) {
		// Nhe factor related to the motion model is defined as
		// f2(x_{t},x_{t+1}) = |A0*x_{t} + A1*x_{t+1} - b|^2
		Index k = step(p);
		return fuse(p, new JacobianFactor(k, A0, k + 1, A1, b, model), useQR());
	}

	/* ************************************************************************* */
	KalmanFilter::State KalmanFilter::update(const State& p, const Matrix& H,
			const Vector& z, const SharedDiagonal& model) {
		// The factor related to the measurements would be defined as
		// f2 = (h(x_{t}) - z_{t}) * R^-1 * (h(x_{t}) - z_{t})^T
		//    = (x_{t} - z_{t}) * R^-1 * (x_{t} - z_{t})^T
		Index k = step(p);
		return fuse(p, new JacobianFactor(k, H, z, model), useQR());
	}

  /* ************************************************************************* */
  KalmanFilter::State KalmanFilter::updateQ(const State& p, const Matrix& H, const Vector& z,
      const Matrix& Q) {
    Index k = step(p);
    Matrix M = inverse(Q), Ht = trans(H);
    Matrix G = Ht * M * H;
    Vector g = Ht * M * z;
    double f = dot(z, M * z);
    return fuse(p, new HessianFactor(k, G, g, f), useQR());
  }

/* ************************************************************************* */

} // \namespace gtsam

