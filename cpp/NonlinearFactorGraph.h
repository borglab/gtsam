/**
 * @file    NonlinearFactorGraph.h
 * @brief   Factor Graph Constsiting of non-linear factors
 * @author  Frank Dellaert
 * @author  Carlos Nieto
 * @author  Christian Potthast
 */

// \callgraph

#pragma once

#include <math.h>

#include "NonlinearFactor.h"
#include "GaussianFactorGraph.h"

namespace gtsam {

	/**
	 * A non-linear factor graph is templated on a configuration, but the factor type
	 * is fixed as a NonlinearFactor. The configurations are typically (in SAM) more general
	 * than just vectors, e.g., Rot3 or Pose3, which are objects in non-linear manifolds.
	 * Linearizing the non-linear factor graph creates a linear factor graph on the 
	 * tangent vector space at the linearization point. Because the tangent space is a true
	 * vector space, the config type will be an VectorConfig in that linearized
	 */
	template<class Config>
	class NonlinearFactorGraph: public FactorGraph<NonlinearFactor<Config> > {

	public:

		/** unnormalized error */
		double error(const Config& c) const;

		/** all individual errors */
		Vector error_vector(const Config& c) const;

		/** Unnormalized probability. O(n) */
		double probPrime(const Config& c) const {
			return exp(-0.5 * error(c));
		}

		/**
		 * linearize a nonlinear factor graph
		 */
		GaussianFactorGraph linearize(const Config& config) const;

		/**
		 * shared pointer versions for MATLAB
		 */
		boost::shared_ptr<GaussianFactorGraph>
				linearize_(const Config& config) const;

	};

} // namespace
