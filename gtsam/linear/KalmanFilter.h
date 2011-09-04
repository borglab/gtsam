/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * testKalmanFilter.cpp
 *
 * Simple linear Kalman filter.
 * Implemented using factor graphs, i.e., does LDL-based SRIF, really.
 *
 *  Created on: Sep 3, 2011
 *  @Author: Stephen Williams
 *  @Author: Frank Dellaert
 */

#include <gtsam/linear/GaussianConditional.h>

namespace gtsam {

	class KalmanFilter {
	private:

		size_t n_; /** dimensionality of state */
		Matrix I_; /** identity matrix of size n*n */

		/** The Kalman filter posterior density is a Gaussian Conditional with no parents */
		GaussianConditional::shared_ptr density_;

	public:

		/**
		 * Constructor from prior density at time k=0
		 * In Kalman Filter notation, these are is x_{0|0} and P_{0|0}
		 * @param x estimate at time 0
		 * @param P covariance at time 0, restricted to diagonal Gaussian 'model' for now
		 */
		KalmanFilter(const Vector& x, const SharedDiagonal& model);

		/** Return mean of posterior P(x|Z) at given all measurements Z */
		Vector mean() const;

		/** Return information matrix of posterior P(x|Z) at given all measurements Z */
		Matrix information() const;

		/** Return covariance of posterior P(x|Z) at given all measurements Z */
		Matrix covariance() const;

		/**
		 * Predict the state P(x_{t+1}|Z^t)
		 *   In Kalman Filter notation, this is x_{t+1|t} and P_{t+1|t}
		 *   After the call, that is the density that can be queried.
		 * Details and parameters:
		 *   In a linear Kalman Filter, the motion model is f(x_{t}) = F*x_{t} + B*u_{t} + w
		 *   where F is the state transition model/matrix, B is the control input model,
		 *   and w is zero-mean, Gaussian white noise with covariance Q.
		 *   Q is normally derived as G*w*G^T where w models uncertainty of some physical property,
		 *   such as velocity or acceleration, and G is derived from physics.
		 *   In the current implementation, the noise model for w is restricted to a diagonal.
		 *   TODO: allow for a G
		 */
		void predict(const Matrix& F, const Matrix& B, const Vector& u,
				const SharedDiagonal& model);

		/**
		 * Update Kalman filter with a measurement
		 * For the Kalman Filter, the measurement function, h(x_{t}) = z_{t}
		 * will be of the form h(x_{t}) = H*x_{t} + v
		 * where H is the observation model/matrix, and v is zero-mean,
		 * Gaussian white noise with covariance R.
		 * Currently, R is restricted to diagonal Gaussians (model parameter)
		 */
		void update(const Matrix& H, const Vector& z, const SharedDiagonal& model);

	};

}// \namespace gtsam

/* ************************************************************************* */

