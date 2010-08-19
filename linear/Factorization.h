/**
 * @file    Factorization
 * @brief   Template Linear solver class that uses a Gaussian Factor Graph
 * @author  Kai Ni
 * @author  Frank Dellaert
 */ 

// $Id: GaussianFactorGraph.h,v 1.24 2009/08/14 20:48:51 acunning Exp $

#pragma once

#include <stdexcept>
#include <gtsam/linear/GaussianFactorGraph.h>

namespace gtsam {

	class Ordering;

  /**
   * A linear system solver using factorization
   */
  template <class NonlinearGraph, class Config>
  class Factorization {
  private:
  	boost::shared_ptr<const Ordering> ordering_;
  	bool useOldEliminate_;

  public:
  	Factorization(boost::shared_ptr<const Ordering> ordering, bool old=true)
		: ordering_(ordering), useOldEliminate_(old) {
  		if (!ordering) throw std::invalid_argument("Factorization constructor: ordering = NULL");
  	}

  	/**
  	 * solve for the optimal displacement in the tangent space, and then solve
  	 * the resulted linear system
  	 */
  	VectorConfig optimize(GaussianFactorGraph& fg) const {
  		return fg.optimize(*ordering_, useOldEliminate_);
  	}

		/**
		 * linearize the non-linear graph around the current config
		 */
  	boost::shared_ptr<GaussianFactorGraph> linearize(const NonlinearGraph& g, const Config& config) const {
  		return g.linearize(config);
  	}

  	/**
  	 * Does not do anything here in Factorization.
  	 */
  	boost::shared_ptr<Factorization> prepareLinear(const GaussianFactorGraph& fg) const {
  		return boost::shared_ptr<Factorization>(new Factorization(*this));
  	}
  };

}
