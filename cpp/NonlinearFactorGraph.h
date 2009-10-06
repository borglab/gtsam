/**
 * @file    NonlinearFactorGraph.h
 * @brief   Factor Graph Constsiting of non-linear factors
 * @author  Frank Dellaert
 * @author  Carlos Nieto
 * @author  Christian Potthast
 */

// \callgraph

#pragma once

#include "NonlinearFactor.h"
#include "FactorGraph.h"

namespace gtsam {

	/**
	 * A non-linear factor graph is templated on a configuration, but the factor type
	 * is fixed as a NonLinearFactor. The configurations are typically (in SAM) more general
	 * than just vectors, e.g., Rot3 or Pose3, which are objects in non-linear manifolds.
	 * Linearizing the non-linear factor graph creates a linear factor graph on the 
	 * tangent vector space at the linearization point. Because the tangent space is a true
	 * vector space, the config type will be an FGConfig in that linearized
 	 */
	template<class Config>
	class NonlinearFactorGraph: public FactorGraph<NonlinearFactor<Config> ,Config> {

	public:

		/**
		 * linearize a nonlinear factor graph
		 */
		LinearFactorGraph linearize(const Config& config) const;

	};

} // namespace
