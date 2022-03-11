//
// Created by Fan Jiang on 3/11/22.
//

#include <gtsam/hybrid/HybridEliminationTree.h>
#include <gtsam/hybrid/HybridJunctionTree.h>
#include <gtsam/hybrid/HybridFactorGraph.h>

#include <gtsam/hybrid/HybridGaussianFactor.h>

#include <gtsam/inference/EliminateableFactorGraph-inst.h>

namespace gtsam {

template class EliminateableFactorGraph<HybridFactorGraph>;

/* ************************************************************************ */
std::pair<HybridConditional::shared_ptr, HybridFactor::shared_ptr>  //
EliminateHybrid(const HybridFactorGraph& factors,
                  const Ordering& frontalKeys) {
  // NOTE(fan): Because we are in the Conditional Gaussian regime there are only
  // few cases: continuous variable, we make a GM if there are hybrid factors;
  // continuous variable, we make a GF if there are no hybrid factors;
  // discrete variable, no continuous factor is allowed (escapes CG regime), so
  // we panic, if discrete only we do the discrete elimination.

  // The issue here is that, how can we know which variable is discrete if we
  // unify Values? Obviously we can tell using the factors, but is that fast?

  // PRODUCT: multiply all factors
  gttic(product);
  HybridGaussianFactor product(JacobianFactor(0, I_3x3, Z_3x1));
//  for (auto&& factor : factors) product = (*factor) * product;
  gttoc(product);

  // sum out frontals, this is the factor on the separator
  gttic(sum);
//  HybridFactor::shared_ptr sum = product.sum(frontalKeys);
  gttoc(sum);

  // Ordering keys for the conditional so that frontalKeys are really in front
//  Ordering orderedKeys;
//  orderedKeys.insert(orderedKeys.end(), frontalKeys.begin(),
//                     frontalKeys.end());
//  orderedKeys.insert(orderedKeys.end(), sum->keys().begin(),
//                     sum->keys().end());

  // now divide product/sum to get conditional
  gttic(divide);
//  auto conditional =
//      boost::make_shared<HybridConditional>(product, *sum, orderedKeys);
  gttoc(divide);

//  return std::make_pair(conditional, sum);
  return std::make_pair(boost::make_shared<HybridConditional>(), boost::make_shared<HybridGaussianFactor>(product));
}

}
