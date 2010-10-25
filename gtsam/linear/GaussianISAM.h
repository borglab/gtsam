/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianISAM
 * @brief   Linear ISAM only
 * @author  Michael Kaess
 */

// \callgraph

#pragma once

#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/inference/ISAM.h>

namespace gtsam {

class GaussianISAM : public ISAM<GaussianConditional> {

  std::deque<size_t, boost::fast_pool_allocator<size_t> > dims_;

public:

  /** Create an empty Bayes Tree */
  GaussianISAM() : ISAM<GaussianConditional>() {}

  /** Create a Bayes Tree from a Bayes Net */
  GaussianISAM(const GaussianBayesNet& bayesNet) : ISAM<GaussianConditional>(bayesNet) {}

  /** Override update_internal to also keep track of variable dimensions. */
  template<class FACTORGRAPH>
  void update_internal(const FACTORGRAPH& newFactors, Cliques& orphans) {

    ISAM<GaussianConditional>::update_internal(newFactors, orphans);

    // update dimensions
    BOOST_FOREACH(const typename FACTORGRAPH::sharedFactor& factor, newFactors) {
      for(typename FACTORGRAPH::Factor::const_iterator key = factor->begin(); key != factor->end(); ++key) {
        if(*key >= dims_.size())
          dims_.resize(*key + 1);
        if(dims_[*key] == 0)
          dims_[*key] = factor->getDim(key);
        else
          assert(dims_[*key] == factor->getDim(key));
      }
    }
  }

  template<class FACTORGRAPH>
  void update(const FACTORGRAPH& newFactors) {
    Cliques orphans;
    this->update_internal(newFactors, orphans);
  }

  void clear() {
    ISAM<GaussianConditional>::clear();
    dims_.clear();
  }

  friend VectorValues optimize(const GaussianISAM&);

};

	// recursively optimize this conditional and all subtrees
	void optimize(const GaussianISAM::sharedClique& clique, VectorValues& result);

	// optimize the BayesTree, starting from the root
	VectorValues optimize(const GaussianISAM& bayesTree);

}/// namespace gtsam
