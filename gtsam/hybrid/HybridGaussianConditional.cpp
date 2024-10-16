/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   HybridGaussianConditional.cpp
 * @brief  A hybrid conditional in the Conditional Linear Gaussian scheme
 * @author Fan Jiang
 * @author Varun Agrawal
 * @author Frank Dellaert
 * @date   Mar 12, 2022
 */

#include <gtsam/base/utilities.h>
#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/hybrid/HybridGaussianConditional.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/inference/Conditional-inst.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>

#include <cstddef>

namespace gtsam {
/* *******************************************************************************/
/**
 * @brief Helper struct for constructing HybridGaussianConditional objects
 *
 * This struct contains the following fields:
 * - nrFrontals: Optional size_t for number of frontal variables
 * - pairs: FactorValuePairs for storing conditionals with their negLogConstant
 * - conditionals: Conditionals for storing conditionals. TODO(frank): kill!
 * - minNegLogConstant: minimum negLogConstant, computed here, subtracted in
 * constructor
 */
struct HybridGaussianConditional::Helper {
  std::optional<size_t> nrFrontals;
  FactorValuePairs pairs;
  double minNegLogConstant;

  using GC = GaussianConditional;
  using P = std::vector<std::pair<Vector, double>>;

  /// Construct from a vector of mean and sigma pairs, plus extra args.
  template <typename... Args>
  explicit Helper(const DiscreteKey &mode, const P &p, Args &&...args) {
    nrFrontals = 1;
    minNegLogConstant = std::numeric_limits<double>::infinity();

    std::vector<GaussianFactorValuePair> fvs;
    std::vector<GC::shared_ptr> gcs;
    fvs.reserve(p.size());
    gcs.reserve(p.size());
    for (auto &&[mean, sigma] : p) {
      auto gaussianConditional =
          GC::sharedMeanAndStddev(std::forward<Args>(args)..., mean, sigma);
      double value = gaussianConditional->negLogConstant();
      minNegLogConstant = std::min(minNegLogConstant, value);
      fvs.emplace_back(gaussianConditional, value);
      gcs.push_back(gaussianConditional);
    }

    pairs = FactorValuePairs({mode}, fvs);
  }

  /// Construct from tree of GaussianConditionals.
  explicit Helper(const Conditionals &conditionals)
      : minNegLogConstant(std::numeric_limits<double>::infinity()) {
    auto func = [this](const GC::shared_ptr &gc) -> GaussianFactorValuePair {
      if (!gc) return {nullptr, std::numeric_limits<double>::infinity()};
      if (!nrFrontals) nrFrontals = gc->nrFrontals();
      double value = gc->negLogConstant();
      minNegLogConstant = std::min(minNegLogConstant, value);
      return {gc, value};
    };
    pairs = FactorValuePairs(conditionals, func);
    if (!nrFrontals.has_value()) {
      throw std::runtime_error(
          "HybridGaussianConditional: need at least one frontal variable. "
          "Provided conditionals do not contain any frontal variables.");
    }
  }
};

/* *******************************************************************************/
HybridGaussianConditional::HybridGaussianConditional(
    const DiscreteKeys &discreteParents, const Helper &helper)
    : BaseFactor(discreteParents,
                 FactorValuePairs(helper.pairs,
                                  [&](const GaussianFactorValuePair &
                                          pair) {  // subtract minNegLogConstant
                                    return GaussianFactorValuePair{
                                        pair.first,
                                        pair.second - helper.minNegLogConstant};
                                  })),
      BaseConditional(*helper.nrFrontals),
      negLogConstant_(helper.minNegLogConstant) {}

HybridGaussianConditional::HybridGaussianConditional(
    const DiscreteKey &discreteParent,
    const std::vector<GaussianConditional::shared_ptr> &conditionals)
    : HybridGaussianConditional(DiscreteKeys{discreteParent},
                                Conditionals({discreteParent}, conditionals)) {}

HybridGaussianConditional::HybridGaussianConditional(
    const DiscreteKey &discreteParent, Key key,  //
    const std::vector<std::pair<Vector, double>> &parameters)
    : HybridGaussianConditional(DiscreteKeys{discreteParent},
                                Helper(discreteParent, parameters, key)) {}

HybridGaussianConditional::HybridGaussianConditional(
    const DiscreteKey &discreteParent, Key key,  //
    const Matrix &A, Key parent,
    const std::vector<std::pair<Vector, double>> &parameters)
    : HybridGaussianConditional(
          DiscreteKeys{discreteParent},
          Helper(discreteParent, parameters, key, A, parent)) {}

HybridGaussianConditional::HybridGaussianConditional(
    const DiscreteKey &discreteParent, Key key,  //
    const Matrix &A1, Key parent1, const Matrix &A2, Key parent2,
    const std::vector<std::pair<Vector, double>> &parameters)
    : HybridGaussianConditional(
          DiscreteKeys{discreteParent},
          Helper(discreteParent, parameters, key, A1, parent1, A2, parent2)) {}

HybridGaussianConditional::HybridGaussianConditional(
    const DiscreteKeys &discreteParents,
    const HybridGaussianConditional::Conditionals &conditionals)
    : HybridGaussianConditional(discreteParents, Helper(conditionals)) {}

/* *******************************************************************************/
const HybridGaussianConditional::Conditionals
HybridGaussianConditional::conditionals() const {
  return Conditionals(factors(), [](const auto& pair) {
    return std::dynamic_pointer_cast<GaussianConditional>(pair.first);
  });
}

/* *******************************************************************************/
size_t HybridGaussianConditional::nrComponents() const {
  size_t total = 0;
  factors().visit([&total](const auto& node) {
    if (node.first) total += 1;
  });
  return total;
}

/* *******************************************************************************/
GaussianConditional::shared_ptr HybridGaussianConditional::choose(
    const DiscreteValues& discreteValues) const {
  auto& [ptr, _] = factors()(discreteValues);
  if (!ptr) return nullptr;
  auto conditional = std::dynamic_pointer_cast<GaussianConditional>(ptr);
  if (conditional)
    return conditional;
  else
    throw std::logic_error(
        "A HybridGaussianConditional unexpectedly contained a non-conditional");
}

/* *******************************************************************************/
bool HybridGaussianConditional::equals(const HybridFactor &lf,
                                       double tol) const {
  const This *e = dynamic_cast<const This *>(&lf);
  if (e == nullptr) return false;

  // Factors existence and scalar values are checked in BaseFactor::equals.
  // Here we check additionally that the factors *are* conditionals and are equal.
  auto compareFunc = [tol](const GaussianFactorValuePair& pair1,
                           const GaussianFactorValuePair& pair2) {
    auto c1 = std::dynamic_pointer_cast<GaussianConditional>(pair1.first),
         c2 = std::dynamic_pointer_cast<GaussianConditional>(pair2.first);
    return (!c1 && !c2) || (c1 && c2 && c1->equals(*c2, tol));
  };
  return Base::equals(*e, tol) && factors().equals(e->factors(), compareFunc);
}

/* *******************************************************************************/
void HybridGaussianConditional::print(const std::string &s,
                                      const KeyFormatter &formatter) const {
  std::cout << (s.empty() ? "" : s + "\n");
  BaseConditional::print("", formatter);
  std::cout << " Discrete Keys = ";
  for (auto &dk : discreteKeys()) {
    std::cout << "(" << formatter(dk.first) << ", " << dk.second << "), ";
  }
  std::cout << std::endl
            << " logNormalizationConstant: " << -negLogConstant() << std::endl
            << std::endl;
  factors().print(
      "", [&](Key k) { return formatter(k); },
      [&](const GaussianFactorValuePair &pair) -> std::string {
        RedirectCout rd;
        if (auto gf = std::dynamic_pointer_cast<GaussianConditional>(pair.first)) {
          gf->print("", formatter);
          return rd.str();
        } else {
          return "nullptr";
        }
      });
}

/* ************************************************************************* */
KeyVector HybridGaussianConditional::continuousParents() const {
  // Get all parent keys:
  const auto range = parents();
  KeyVector continuousParentKeys(range.begin(), range.end());
  // Loop over all discrete keys:
  for (const auto &discreteKey : discreteKeys()) {
    const Key key = discreteKey.first;
    // remove that key from continuousParentKeys:
    continuousParentKeys.erase(std::remove(continuousParentKeys.begin(),
                                           continuousParentKeys.end(), key),
                               continuousParentKeys.end());
  }
  return continuousParentKeys;
}

/* ************************************************************************* */
bool HybridGaussianConditional::allFrontalsGiven(
    const VectorValues &given) const {
  for (auto &&kv : given) {
    if (given.find(kv.first) == given.end()) {
      return false;
    }
  }
  return true;
}

/* ************************************************************************* */
std::shared_ptr<HybridGaussianFactor> HybridGaussianConditional::likelihood(
    const VectorValues &given) const {
  if (!allFrontalsGiven(given)) {
    throw std::runtime_error(
        "HybridGaussianConditional::likelihood: given values are missing some "
        "frontals.");
  }

  const DiscreteKeys discreteParentKeys = discreteKeys();
  const KeyVector continuousParentKeys = continuousParents();
  const HybridGaussianFactor::FactorValuePairs likelihoods(
      factors(),
      [&](const GaussianFactorValuePair &pair) -> GaussianFactorValuePair {
        if (auto conditional =
                std::dynamic_pointer_cast<GaussianConditional>(pair.first)) {
          const auto likelihood_m = conditional->likelihood(given);
          // scalar is already correct.
          assert(pair.second ==
                 conditional->negLogConstant() - negLogConstant_);
          return {likelihood_m, pair.second};
        } else {
          return {nullptr, std::numeric_limits<double>::infinity()};
        }
      });
  return std::make_shared<HybridGaussianFactor>(discreteParentKeys,
                                                likelihoods);
}

/* ************************************************************************* */
std::set<DiscreteKey> DiscreteKeysAsSet(const DiscreteKeys &discreteKeys) {
  std::set<DiscreteKey> s(discreteKeys.begin(), discreteKeys.end());
  return s;
}

/* *******************************************************************************/
HybridGaussianConditional::shared_ptr HybridGaussianConditional::prune(
    const DecisionTreeFactor &discreteProbs) const {
  // Find keys in discreteProbs.keys() but not in this->keys():
  std::set<Key> mine(this->keys().begin(), this->keys().end());
  std::set<Key> theirs(discreteProbs.keys().begin(),
                       discreteProbs.keys().end());
  std::vector<Key> diff;
  std::set_difference(theirs.begin(), theirs.end(), mine.begin(), mine.end(),
                      std::back_inserter(diff));

  // Find maximum probability value for every combination of our keys.
  Ordering keys(diff);
  auto max = discreteProbs.max(keys);

  // Check the max value for every combination of our keys.
  // If the max value is 0.0, we can prune the corresponding conditional.
  auto pruner =
      [&](const Assignment<Key> &choices,
          const GaussianFactorValuePair &pair) -> GaussianFactorValuePair {
    if (max->evaluate(choices) == 0.0)
      return {nullptr, std::numeric_limits<double>::infinity()};
    else
      return pair;
  };

  FactorValuePairs prunedConditionals = factors().apply(pruner);
  return std::shared_ptr<HybridGaussianConditional>(
      new HybridGaussianConditional(discreteKeys(), nrFrontals_,
                                    prunedConditionals, negLogConstant_));
}

/* *******************************************************************************/
double HybridGaussianConditional::logProbability(
    const HybridValues& values) const {
  auto [factor, _] = factors()(values.discrete());
  if (auto conditional = std::dynamic_pointer_cast<GaussianConditional>(factor))
    return conditional->logProbability(values.continuous());
  else
    throw std::logic_error(
        "A HybridGaussianConditional unexpectedly contained a non-conditional");
}

/* *******************************************************************************/
double HybridGaussianConditional::evaluate(const HybridValues& values) const {
  auto [factor, _] = factors()(values.discrete());
  if (auto conditional = std::dynamic_pointer_cast<GaussianConditional>(factor))
    return conditional->evaluate(values.continuous());
  else
    throw std::logic_error(
        "A HybridGaussianConditional unexpectedly contained a non-conditional");
}

}  // namespace gtsam
