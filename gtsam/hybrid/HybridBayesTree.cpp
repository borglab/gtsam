/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    HybridBayesTree.cpp
 * @brief   Hybrid Bayes Tree, the result of eliminating a
 * HybridJunctionTree
 * @date Mar 11, 2022
 * @author  Fan Jiang
 */

#include <gtsam/base/treeTraversal-inst.h>
#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridBayesTree.h>
#include <gtsam/inference/BayesTree-inst.h>
#include <gtsam/inference/BayesTreeCliqueBase-inst.h>

namespace gtsam {

// Instantiate base class
template class BayesTreeCliqueBase<HybridBayesTreeClique,
                                   HybridGaussianFactorGraph>;
template class BayesTree<HybridBayesTreeClique>;

/* ************************************************************************* */
bool HybridBayesTree::equals(const This& other, double tol) const {
  return Base::equals(other, tol);
}

/* ************************************************************************* */
HybridValues HybridBayesTree::optimize() const {
  HybridBayesNet hbn;
  DiscreteBayesNet dbn;

  KeyVector added_keys;

  // Iterate over all the nodes in the BayesTree
  for (auto&& node : nodes()) {
    // Check if conditional being added is already in the Bayes net.
    if (std::find(added_keys.begin(), added_keys.end(), node.first) ==
        added_keys.end()) {
      // Access the clique and get the underlying hybrid conditional
      HybridBayesTreeClique::shared_ptr clique = node.second;
      HybridConditional::shared_ptr conditional = clique->conditional();

      // Record the key being added
      added_keys.insert(added_keys.end(), conditional->frontals().begin(),
                        conditional->frontals().end());

      if (conditional->isDiscrete()) {
        // If discrete, we use it to compute the MPE
        dbn.push_back(conditional->asDiscreteConditional());

      } else {
        // Else conditional is hybrid or continuous-only,
        // so we directly add it to the Hybrid Bayes net.
        hbn.push_back(conditional);
      }
    }
  }
  // Get the MPE
  DiscreteValues mpe = DiscreteFactorGraph(dbn).optimize();
  // Given the MPE, compute the optimal continuous values.
  GaussianBayesNet gbn = hbn.choose(mpe);

  // If TBB is enabled, the bayes net order gets reversed,
  // so we pre-reverse it
#ifdef GTSAM_USE_TBB
  auto reversed = boost::adaptors::reverse(gbn);
  gbn = GaussianBayesNet(reversed.begin(), reversed.end());
#endif

  return HybridValues(mpe, gbn.optimize());
}

/* ************************************************************************* */
VectorValues HybridBayesTree::optimize(const DiscreteValues& assignment) const {
  GaussianBayesNet gbn;

  KeyVector added_keys;

  // Iterate over all the nodes in the BayesTree
  for (auto&& node : nodes()) {
    // Check if conditional being added is already in the Bayes net.
    if (std::find(added_keys.begin(), added_keys.end(), node.first) ==
        added_keys.end()) {
      // Access the clique and get the underlying hybrid conditional
      HybridBayesTreeClique::shared_ptr clique = node.second;
      HybridConditional::shared_ptr conditional = clique->conditional();

      // Record the key being added
      added_keys.insert(added_keys.end(), conditional->frontals().begin(),
                        conditional->frontals().end());

      // If conditional is hybrid (and not discrete-only), we get the Gaussian
      // Conditional corresponding to the assignment and add it to the Gaussian
      // Bayes Net.
      if (conditional->isHybrid()) {
        auto gm = conditional->asMixture();
        GaussianConditional::shared_ptr gaussian_conditional =
            (*gm)(assignment);

        gbn.push_back(gaussian_conditional);

      } else if (conditional->isContinuous()) {
        // If conditional is Gaussian, we simply add it to the Bayes net.
        gbn.push_back(conditional->asGaussian());
      }
    }
  }

  // If TBB is enabled, the bayes net order gets reversed,
  // so we pre-reverse it
#ifdef GTSAM_USE_TBB
  auto reversed = boost::adaptors::reverse(gbn);
  gbn = GaussianBayesNet(reversed.begin(), reversed.end());
#endif

  // Return the optimized bayes net.
  return gbn.optimize();
}

}  // namespace gtsam
