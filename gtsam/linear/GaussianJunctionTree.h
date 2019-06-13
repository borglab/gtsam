/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file GaussianJunctionTree.h
 * @date Mar 29, 2013
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/inference/JunctionTree.h>

namespace gtsam {

  // Forward declarations
  class GaussianEliminationTree;

  /**
   * A junction tree specialized to Gaussian factors, i.e., it is a cluster tree with Gaussian
   * factors stored in each cluster. It can be eliminated into a Gaussian Bayes tree with the same
   * structure, which is essentially doing multifrontal sparse matrix factorization.
   *
   * \addtogroup Multifrontal
   * \nosubgrouping
   */
  class GTSAM_EXPORT GaussianJunctionTree :
    public JunctionTree<GaussianBayesTree, GaussianFactorGraph> {
  public:
    typedef JunctionTree<GaussianBayesTree, GaussianFactorGraph> Base; ///< Base class
    typedef GaussianJunctionTree This; ///< This class
    typedef boost::shared_ptr<This> shared_ptr; ///< Shared pointer to this class

    /**
    * Build the elimination tree of a factor graph using pre-computed column structure.
    * @param factorGraph The factor graph for which to build the elimination tree
    * @param structure The set of factors involving each variable.  If this is not
    * precomputed, you can call the Create(const FactorGraph<DERIVEDFACTOR>&)
    * named constructor instead.
    * @return The elimination tree
    */
    GaussianJunctionTree(const GaussianEliminationTree& eliminationTree);
  };

}
