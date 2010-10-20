/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

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
  template <class NONLINEARGRAPH, class VALUES>
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
  	VectorValues optimize(GaussianFactorGraph& fg) const {
  	  return gtsam::optimize(*Inference::Eliminate(fg));
  	}

		/**
		 * linearize the non-linear graph around the current config
		 */
  	boost::shared_ptr<GaussianFactorGraph> linearize(const NONLINEARGRAPH& g, const VALUES& config) const {
  		return g.linearize(config, *ordering_);
  	}

  	/**
  	 * Does not do anything here in Factorization.
  	 */
  	boost::shared_ptr<Factorization> prepareLinear(const GaussianFactorGraph& fg) const {
  		return boost::shared_ptr<Factorization>(new Factorization(*this));
  	}

  	/** expmap the Values given the stored Ordering */
  	VALUES expmap(const VALUES& config, const VectorValues& delta) const {
  	  return config.expmap(delta, *ordering_);
  	}
  };

}
