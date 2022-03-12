//
// Created by Fan Jiang on 3/11/22.
//

#include <gtsam/hybrid/HybridEliminationTree.h>
#include <gtsam/hybrid/HybridJunctionTree.h>
#include <gtsam/hybrid/HybridFactorGraph.h>

#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/hybrid/HybridDiscreteFactor.h>

#include <gtsam/inference/EliminateableFactorGraph-inst.h>

namespace gtsam {

template
class EliminateableFactorGraph<HybridFactorGraph>;

/* ************************************************************************ */
std::pair<HybridConditional::shared_ptr, HybridFactor::shared_ptr>  //
EliminateHybrid(const HybridFactorGraph &factors,
                const Ordering &frontalKeys) {
  // NOTE(fan): Because we are in the Conditional Gaussian regime there are only
  // few cases: continuous variable, we make a GM if there are hybrid factors;
  // continuous variable, we make a GF if there are no hybrid factors;
  // discrete variable, no continuous factor is allowed (escapes CG regime), so
  // we panic, if discrete only we do the discrete elimination.

  // The issue here is that, how can we know which variable is discrete if we
  // unify Values? Obviously we can tell using the factors, but is that fast?

  // In the case of multifrontal, we will need to use a constrained ordering
  // so that the discrete parts will be guaranteed to be eliminated last!

  // PRODUCT: multiply all factors
  gttic(product);
  KeySet allKeys;
  // TODO: we do a mock by just doing the correct key thing
  std::cout << "Begin Eliminate\n";
  for (auto &&factor : factors) {
    std::cout << ">>> Eliminating: ";
    factor->printKeys();
    allKeys.insert(factor->begin(), factor->end());
  }
  for (auto &k : frontalKeys) {
    allKeys.erase(k);
  }

  HybridConditional sum(allKeys.size(), Ordering(allKeys));
//  HybridDiscreteFactor product(DiscreteConditional());
//  for (auto&& factor : factors) product = (*factor) * product;
  gttoc(product);

  // sum out frontals, this is the factor on the separator
  gttic(sum);
//  HybridFactor::shared_ptr sum = product.sum(frontalKeys);
  gttoc(sum);

  // Ordering keys for the conditional so that frontalKeys are really in front
  Ordering orderedKeys;
  orderedKeys.insert(orderedKeys.end(), frontalKeys.begin(),
                     frontalKeys.end());
  orderedKeys.insert(orderedKeys.end(), sum.keys().begin(),
                     sum.keys().end());

  // now divide product/sum to get conditional
  gttic(divide);
//  auto conditional =
//      boost::make_shared<HybridConditional>(product, *sum, orderedKeys);
  gttoc(divide);

//  return std::make_pair(conditional, sum);
  return std::make_pair(boost::make_shared<HybridConditional>(frontalKeys.size(),
                                                              orderedKeys),
                        boost::make_shared<HybridConditional>(std::move(sum)));
}

void HybridFactorGraph::add(JacobianFactor &&factor) {
  FactorGraph::add(boost::make_shared<HybridGaussianFactor>(std::move(factor)));
}
}
