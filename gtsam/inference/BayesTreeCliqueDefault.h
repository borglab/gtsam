/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BayesTreeCliqueDefault.h
 * @brief   A standard BayesTree clique.  iSAM2 uses a specialized clique.
 * @author  Frank Dellaert
 * @author  Richard Roberts
 */

#pragma once

#include <gtsam/inference/BayesTreeCliqueBaseUnordered.h>

namespace gtsam {

  /* ************************************************************************* */
  /**
   * A Clique in the tree is an incomplete Bayes net: the variables
   * in the Bayes net are the frontal nodes, and the variables conditioned
   * on are the separator. We also have pointers up and down the tree.
   *
   * Since our Conditional class already handles multiple frontal variables,
   * this Clique contains exactly 1 conditional.
   *
   * This is the default clique type in a BayesTree, but some algorithms, like
   * iSAM2 (see ISAM2Clique), use a different clique type in order to store
   * extra data along with the clique.
   */
  template<class FACTORGRAPH, class BAYESNET>
  struct BayesTreeCliqueDefaultUnordered :
    public BayesTreeCliqueBaseUnordered<BayesTreeCliqueDefaultUnordered<FACTORGRAPH,BAYESNET>, FACTORGRAPH, BAYESNET> {
  public:
    typedef typename BAYESNET::ConditionalType ConditionalType;
    typedef typename FACTORGRAPH::FactorType FactorType;
    typedef BayesTreeCliqueDefaultUnordered<FACTORGRAPH,BAYESNET> This;
    typedef BayesTreeCliqueBaseUnordered<This, FACTORGRAPH, BAYESNET> Base;
    typedef boost::shared_ptr<This> shared_ptr;
    typedef boost::weak_ptr<This> weak_ptr;
    BayesTreeCliqueDefaultUnordered() {}
    BayesTreeCliqueDefaultUnordered(const boost::shared_ptr<ConditionalType>& conditional) : Base(conditional) {}

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    }
  };

}
