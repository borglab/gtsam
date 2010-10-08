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
#include <gtsam/inference/inference-inl.h>

namespace gtsam {

	class Ordering;

  /**
   * A linear system solver using factorization
   */
  template <class NonlinearGraph, class Config>
  class Factorization {
  private:
  	boost::shared_ptr<Ordering> ordering_;

  public:
  	Factorization(boost::shared_ptr<Ordering> ordering)
		: ordering_(ordering) {
  		if (!ordering) throw std::invalid_argument("Factorization constructor: ordering = NULL");
  	}

  	/**
  	 * solve for the optimal displacement in the tangent space, and then solve
  	 * the resulted linear system
  	 */
  	VectorConfig optimize(GaussianFactorGraph& fg) const {
  	  return gtsam::optimize(*Inference::Eliminate(fg));
  	}

		/**
		 * linearize the non-linear graph around the current config
		 */
  	boost::shared_ptr<GaussianFactorGraph> linearize(const NonlinearGraph& g, const Config& config) const {
  		return g.linearize(config, *ordering_);
  	}

  	/**
  	 * Does not do anything here in Factorization.
  	 */
  	boost::shared_ptr<Factorization> prepareLinear(const GaussianFactorGraph& fg) const {
  		return boost::shared_ptr<Factorization>(new Factorization(*this));
  	}

  	/** expmap the Config given the stored Ordering */
  	Config expmap(const Config& config, const VectorConfig& delta) const {
  	  return config.expmap(delta, *ordering_);
  	}
  };

}
