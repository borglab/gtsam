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

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/inference/SymbolicFactorGraph.h>
#include <gtsam/nonlinear/Ordering.h>

namespace gtsam {

	/**
	 * A non-linear factor graph is templated on a configuration, but the factor type
	 * is fixed as a NonlinearFactor. The configurations are typically (in SAM) more general
	 * than just vectors, e.g., Rot3 or Pose3, which are objects in non-linear manifolds.
	 * Linearizing the non-linear factor graph creates a linear factor graph on the 
	 * tangent vector space at the linearization point. Because the tangent space is a true
	 * vector space, the config type will be an VectorConfig in that linearized factor graph.
	 */
	template<class Config>
	class NonlinearFactorGraph: public FactorGraph<NonlinearFactor<Config> > {

	public:

	  typedef FactorGraph<NonlinearFactor<Config> > Base;
		typedef typename boost::shared_ptr<NonlinearFactor<Config> > sharedFactor;

    /** print just calls base class */
    void print(const std::string& str = "NonlinearFactorGraph: ") const;

		/** unnormalized error */
		double error(const Config& c) const;

		/** all individual errors */
		Vector unwhitenedError(const Config& c) const;

		/** Unnormalized probability. O(n) */
		double probPrime(const Config& c) const {
			return exp(-0.5 * error(c));
		}

		template<class F>
		void add(const F& factor) {
			push_back(boost::shared_ptr<F>(new F(factor)));
		}

		/**
		 * Create a symbolic factor graph using an existing ordering
		 */
		SymbolicFactorGraph::shared_ptr symbolic(const Config& config, const Ordering& ordering) const;

		/**
		 * Create a symbolic factor graph and initial variable ordering that can
		 * be used for graph operations like determining a fill-reducing ordering.
		 * The graph and ordering should be permuted after such a fill-reducing
		 * ordering is found.
		 */
		std::pair<SymbolicFactorGraph::shared_ptr, Ordering::shared_ptr>
		symbolic(const Config& config) const;

    /**
     * Compute a fill-reducing ordering using COLAMD.  This returns the
     * ordering and a VariableIndex, which can later be re-used to save
     * computation.
     */
		std::pair<Ordering::shared_ptr, GaussianVariableIndex<>::shared_ptr>
		orderingCOLAMD(const Config& config) const;

		/**
		 * linearize a nonlinear factor graph
		 */
		boost::shared_ptr<GaussianFactorGraph>
				linearize(const Config& config, const Ordering& ordering) const;

	};

} // namespace
