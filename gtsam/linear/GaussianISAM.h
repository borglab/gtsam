/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianISAM.h
 * @brief   Linear ISAM only
 * @author  Michael Kaess
 */

// \callgraph

#pragma once

#include <boost/optional.hpp>

#include <gtsam/inference/ISAM.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianJunctionTree.h>

namespace gtsam {

class GaussianISAM : public ISAM<GaussianConditional> {

	typedef ISAM<GaussianConditional> Super;
  std::deque<size_t, boost::fast_pool_allocator<size_t> > dims_;

public:

  typedef std::deque<size_t, boost::fast_pool_allocator<size_t> > Dims;

  /** Create an empty Bayes Tree */
  GaussianISAM() : Super() {}

  /** Create a Bayes Tree from a Bayes Net */
//  GaussianISAM(const GaussianBayesNet& bayesNet) : Super(bayesNet) {}
  GaussianISAM(const BayesTree<GaussianConditional>& bayesTree) : Super(bayesTree) {
    GaussianJunctionTree::countDims(bayesTree, dims_);
  }

  /** Override update_internal to also keep track of variable dimensions. */
  template<class FACTORGRAPH>
  void update_internal(const FACTORGRAPH& newFactors, Cliques& orphans) {
    Super::update_internal(newFactors, orphans, &EliminateQR); // TODO: why does this force QR?
    update_dimensions(newFactors);
  }

  template<class FACTORGRAPH>
  void update(const FACTORGRAPH& newFactors) {
    Cliques orphans;
    this->update_internal(newFactors, orphans);
  }

  template<class FACTORGRAPH>
  inline void update_dimensions(const FACTORGRAPH& newFactors) {
  	BOOST_FOREACH(const typename FACTORGRAPH::sharedFactor& factor, newFactors) {
  		for(typename FACTORGRAPH::FactorType::const_iterator key = factor->begin(); key != factor->end(); ++key) {
  			if(*key >= dims_.size())
  				dims_.resize(*key + 1);
  			if(dims_[*key] == 0)
  				dims_[*key] = factor->getDim(key);
  			else
  				assert(dims_[*key] == factor->getDim(key));
  		}
  	}
  }

  void clear() {
    Super::clear();
    dims_.clear();
  }

  // access
  const Dims& dims() const { return dims_; } ///< Const access to dimensions structure
  Dims& dims() { return dims_; } ///< non-const access to dimensions structure (advanced interface)

  friend VectorValues optimize(const GaussianISAM&);

	/** return marginal on any variable as a factor, Bayes net, or mean/cov */
  GaussianFactor::shared_ptr marginalFactor(Index j) const;
	BayesNet<GaussianConditional>::shared_ptr marginalBayesNet(Index key) const;
  Matrix marginalCovariance(Index key) const;

  /** return joint between two variables, as a Bayes net */
  BayesNet<GaussianConditional>::shared_ptr jointBayesNet(Index key1, Index key2) const;

	/** return the conditional P(S|Root) on the separator given the root */
	static BayesNet<GaussianConditional> shortcut(sharedClique clique, sharedClique root);

}; // \class GaussianISAM

// optimize the BayesTree, starting from the root
VectorValues optimize(const GaussianISAM& isam);

} // \namespace gtsam
