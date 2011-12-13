/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianISAM2
 * @brief   Full non-linear ISAM
 * @author  Michael Kaess
 */

#include <gtsam/inference/FactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>

using namespace std;
using namespace gtsam;

#include <boost/bind.hpp>

namespace gtsam {

  /* ************************************************************************* */
  VectorValues gradient(const BayesTree<GaussianConditional>& bayesTree, const VectorValues& x0) {
    return gradient(FactorGraph<JacobianFactor>(bayesTree), x0);
  }

  /* ************************************************************************* */
  void gradientAtZero(const BayesTree<GaussianConditional>& bayesTree, VectorValues& g) {
    gradientAtZero(FactorGraph<JacobianFactor>(bayesTree), g);
  }

  /* ************************************************************************* */
  VectorValues gradient(const BayesTree<GaussianConditional, ISAM2Clique<GaussianConditional> >& bayesTree, const VectorValues& x0) {
    return gradient(FactorGraph<JacobianFactor>(bayesTree));
  }

  /* ************************************************************************* */
  void gradientAtZero(const BayesTree<GaussianConditional, ISAM2Clique<GaussianConditional> >& bayesTree, VectorValues& g) {
    // Zero-out gradient
    g.setZero();

    // Sum up contributions for each clique
    typedef boost::shared_ptr<ISAM2Clique<GaussianConditional> > sharedClique;
    BOOST_FOREACH(const sharedClique& clique, bayesTree.nodes()) {
      // Loop through variables in each clique, adding contributions
      int variablePosition = 0;
      for(GaussianConditional::const_iterator jit = clique->conditional()->beginFrontals(); jit != clique->conditional()->endFrontals(); ++jit) {
        const int dim = clique->conditional()->dim(jit);
        x0[*jit] += clique->gradientContribution().segment(variablePosition, dim);
        variablePosition += dim;
      }
    }
  }

}
