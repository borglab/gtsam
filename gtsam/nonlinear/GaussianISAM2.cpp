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
#include <gtsam/nonlinear/GaussianISAM2.h>

using namespace std;
using namespace gtsam;

#include <boost/bind.hpp>

namespace gtsam {

  /* ************************************************************************* */
  VectorValues gradient(const BayesTree<GaussianConditional, ISAM2Clique<GaussianConditional> >& bayesTree, const VectorValues& x0) {
    return gradient(FactorGraph<JacobianFactor>(bayesTree), x0);
  }

  /* ************************************************************************* */
  static void gradientAtZeroTreeAdder(const boost::shared_ptr<ISAM2Clique<GaussianConditional> >& root, VectorValues& g) {
    // Loop through variables in each clique, adding contributions
    int variablePosition = 0;
    for(GaussianConditional::const_iterator jit = root->conditional()->begin(); jit != root->conditional()->end(); ++jit) {
      const int dim = root->conditional()->dim(jit);
      g[*jit] += root->gradientContribution().segment(variablePosition, dim);
      variablePosition += dim;
    }

    // Recursively add contributions from children
    typedef boost::shared_ptr<ISAM2Clique<GaussianConditional> > sharedClique;
    BOOST_FOREACH(const sharedClique& child, root->children()) {
      gradientAtZeroTreeAdder(child, g);
    }
  }

  /* ************************************************************************* */
  void gradientAtZero(const BayesTree<GaussianConditional, ISAM2Clique<GaussianConditional> >& bayesTree, VectorValues& g) {
    // Zero-out gradient
    g.setZero();

    // Sum up contributions for each clique
    gradientAtZeroTreeAdder(bayesTree.root(), g);
  }

}
