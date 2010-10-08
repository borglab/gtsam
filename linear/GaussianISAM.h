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
  template<class FactorGraph>
  void update_internal(const FactorGraph& newFactors, Cliques& orphans) {

    ISAM<GaussianConditional>::update_internal(newFactors, orphans);

    // update dimensions
    BOOST_FOREACH(const typename FactorGraph::sharedFactor& factor, newFactors) {
      for(typename FactorGraph::factor_type::const_iterator key = factor->begin(); key != factor->end(); ++key) {
        if(*key >= dims_.size())
          dims_.resize(*key + 1);
        if(dims_[*key] == 0)
          dims_[*key] = factor->getDim(key);
        else
          assert(dims_[*key] == factor->getDim(key));
      }
    }
  }

  template<class FactorGraph>
  void update(const FactorGraph& newFactors) {
    Cliques orphans;
    this->update_internal(newFactors, orphans);
  }

  void clear() {
    ISAM<GaussianConditional>::clear();
    dims_.clear();
  }

  friend VectorConfig optimize(const GaussianISAM&);

};

	// recursively optimize this conditional and all subtrees
	void optimize(const GaussianISAM::sharedClique& clique, VectorConfig& result);

	// optimize the BayesTree, starting from the root
	VectorConfig optimize(const GaussianISAM& bayesTree);

}/// namespace gtsam
