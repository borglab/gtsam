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
#include <gtsam/hybrid/HybridGaussianProductFactor.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/inference/Conditional-inst.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>

#include <cstddef>

namespace gtsam {
/* *******************************************************************************/
struct HybridGaussianConditional::Helper {
  std::optional<size_t> nrFrontals;
  FactorValuePairs pairs;
  Conditionals conditionals;
  double minNegLogConstant;

  using GC = GaussianConditional;
  using P = std::vector<std::pair<Vector, double>>;

  /// Construct from a vector of mean and sigma pairs, plus extra args.
  template <typename... Args>
  explicit Helper(const DiscreteKey& mode, const P& p, Args&&... args) {
    nrFrontals = 1;
    minNegLogConstant = std::numeric_limits<double>::infinity();

    std::vector<GaussianFactorValuePair> fvs;
    std::vector<GC::shared_ptr> gcs;
    fvs.reserve(p.size());
    gcs.reserve(p.size());
    for (auto&& [mean, sigma] : p) {
      auto gaussianConditional = GC::sharedMeanAndStddev(std::forward<Args>(args)..., mean, sigma);
      double value = gaussianConditional->negLogConstant();
      minNegLogConstant = std::min(minNegLogConstant, value);
      fvs.emplace_back(gaussianConditional, value);
      gcs.push_back(gaussianConditional);
    }

    conditionals = Conditionals({mode}, gcs);
    pairs = FactorValuePairs({mode}, fvs);
  }

  /// Construct from tree of GaussianConditionals.
  explicit Helper(const Conditionals& conditionals)
      : conditionals(conditionals), minNegLogConstant(std::numeric_limits<double>::infinity()) {
    auto func = [this](const GC::shared_ptr& c) -> GaussianFactorValuePair {
      double value = 0.0;
      if (c) {
        if (!nrFrontals.has_value()) {
          nrFrontals = c->nrFrontals();
        }
        value = c->negLogConstant();
        minNegLogConstant = std::min(minNegLogConstant, value);
      }
      return {std::dynamic_pointer_cast<GaussianFactor>(c), value};
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
HybridGaussianConditional::HybridGaussianConditional(const DiscreteKeys& discreteParents,
                                                     const Helper& helper)
    : BaseFactor(discreteParents, helper.pairs),
      BaseConditional(*helper.nrFrontals),
      conditionals_(helper.conditionals),
      negLogConstant_(helper.minNegLogConstant) {}

HybridGaussianConditional::HybridGaussianConditional(
    const DiscreteKey& discreteParent,
    const std::vector<GaussianConditional::shared_ptr>& conditionals)
    : HybridGaussianConditional(DiscreteKeys{discreteParent},
                                Conditionals({discreteParent}, conditionals)) {}

HybridGaussianConditional::HybridGaussianConditional(
    const DiscreteKey& discreteParent,
    Key key,  //
    const std::vector<std::pair<Vector, double>>& parameters)
    : HybridGaussianConditional(DiscreteKeys{discreteParent},
                                Helper(discreteParent, parameters, key)) {}

HybridGaussianConditional::HybridGaussianConditional(
    const DiscreteKey& discreteParent,
    Key key,  //
    const Matrix& A,
    Key parent,
    const std::vector<std::pair<Vector, double>>& parameters)
    : HybridGaussianConditional(DiscreteKeys{discreteParent},
                                Helper(discreteParent, parameters, key, A, parent)) {}

HybridGaussianConditional::HybridGaussianConditional(
    const DiscreteKey& discreteParent,
    Key key,  //
    const Matrix& A1,
    Key parent1,
    const Matrix& A2,
    Key parent2,
    const std::vector<std::pair<Vector, double>>& parameters)
    : HybridGaussianConditional(DiscreteKeys{discreteParent},
                                Helper(discreteParent, parameters, key, A1, parent1, A2, parent2)) {
}

HybridGaussianConditional::HybridGaussianConditional(
    const DiscreteKeys& discreteParents,
    const HybridGaussianConditional::Conditionals& conditionals)
    : HybridGaussianConditional(discreteParents, Helper(conditionals)) {}

/* *******************************************************************************/
const HybridGaussianConditional::Conditionals& HybridGaussianConditional::conditionals() const {
  return conditionals_;
}

/* *******************************************************************************/
HybridGaussianProductFactor HybridGaussianConditional::asProductFactor() const {
  auto wrap = [this](const std::shared_ptr<GaussianConditional>& gc)
      -> std::pair<GaussianFactorGraph, double> {
    // First check if conditional has not been pruned
    if (gc) {
      const double Cgm_Kgcm = gc->negLogConstant() - this->negLogConstant_;
      // If there is a difference in the covariances, we need to account for
      // that since the error is dependent on the mode.
      if (Cgm_Kgcm > 0.0) {
        // We add a constant factor which will be used when computing
        // the probability of the discrete variables.
        Vector c(1);
        c << std::sqrt(2.0 * Cgm_Kgcm);
        auto constantFactor = std::make_shared<JacobianFactor>(c);
        return {GaussianFactorGraph{gc, constantFactor}, Cgm_Kgcm};
      } else {
        // The scalar can be zero.
        // TODO(Frank): after hiding is gone, this should be only case here.
        return {GaussianFactorGraph{gc}, Cgm_Kgcm};
      }
    } else {
      // If the conditional is pruned, return an empty GaussianFactorGraph with zero scalar sum
      // TODO(Frank): Could we just return an *empty* GaussianFactorGraph?
      return {GaussianFactorGraph{nullptr}, 0.0};
    }
  };
  return {{conditionals_, wrap}};
}

/* *******************************************************************************/
size_t HybridGaussianConditional::nrComponents() const {
  size_t total = 0;
  conditionals_.visit([&total](const GaussianFactor::shared_ptr& node) {
    if (node) total += 1;
  });
  return total;
}

/* *******************************************************************************/
GaussianConditional::shared_ptr HybridGaussianConditional::choose(
    const DiscreteValues& discreteValues) const {
  auto& ptr = conditionals_(discreteValues);
  if (!ptr) return nullptr;
  auto conditional = std::dynamic_pointer_cast<GaussianConditional>(ptr);
  if (conditional)
    return conditional;
  else
    throw std::logic_error("A HybridGaussianConditional unexpectedly contained a non-conditional");
}

/* *******************************************************************************/
bool HybridGaussianConditional::equals(const HybridFactor& lf, double tol) const {
  const This* e = dynamic_cast<const This*>(&lf);
  if (e == nullptr) return false;

  // This will return false if either conditionals_ is empty or e->conditionals_
  // is empty, but not if both are empty or both are not empty:
  if (conditionals_.empty() ^ e->conditionals_.empty()) return false;

  // Check the base and the factors:
  return BaseFactor::equals(*e, tol) &&
         conditionals_.equals(e->conditionals_, [tol](const auto& f1, const auto& f2) {
           return (!f1 && !f2) || (f1 && f2 && f1->equals(*f2, tol));
         });
}

/* *******************************************************************************/
void HybridGaussianConditional::print(const std::string& s, const KeyFormatter& formatter) const {
  std::cout << (s.empty() ? "" : s + "\n");
  BaseConditional::print("", formatter);
  std::cout << " Discrete Keys = ";
  for (auto& dk : discreteKeys()) {
    std::cout << "(" << formatter(dk.first) << ", " << dk.second << "), ";
  }
  std::cout << std::endl
            << " logNormalizationConstant: " << -negLogConstant() << std::endl
            << std::endl;
  conditionals_.print(
      "",
      [&](Key k) { return formatter(k); },
      [&](const GaussianConditional::shared_ptr& gf) -> std::string {
        RedirectCout rd;
        if (gf && !gf->empty()) {
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
  for (const auto& discreteKey : discreteKeys()) {
    const Key key = discreteKey.first;
    // remove that key from continuousParentKeys:
    continuousParentKeys.erase(
        std::remove(continuousParentKeys.begin(), continuousParentKeys.end(), key),
        continuousParentKeys.end());
  }
  return continuousParentKeys;
}

/* ************************************************************************* */
bool HybridGaussianConditional::allFrontalsGiven(const VectorValues& given) const {
  for (auto&& kv : given) {
    if (given.find(kv.first) == given.end()) {
      return false;
    }
  }
  return true;
}

/* ************************************************************************* */
std::shared_ptr<HybridGaussianFactor> HybridGaussianConditional::likelihood(
    const VectorValues& given) const {
  if (!allFrontalsGiven(given)) {
    throw std::runtime_error(
        "HybridGaussianConditional::likelihood: given values are missing some "
        "frontals.");
  }

  const DiscreteKeys discreteParentKeys = discreteKeys();
  const KeyVector continuousParentKeys = continuousParents();
  const HybridGaussianFactor::FactorValuePairs likelihoods(
      conditionals_,
      [&](const GaussianConditional::shared_ptr& conditional) -> GaussianFactorValuePair {
        const auto likelihood_m = conditional->likelihood(given);
        const double Cgm_Kgcm = conditional->negLogConstant() - negLogConstant_;
        return {likelihood_m, Cgm_Kgcm};
      });
  return std::make_shared<HybridGaussianFactor>(discreteParentKeys, likelihoods);
}

/* ************************************************************************* */
std::set<DiscreteKey> DiscreteKeysAsSet(const DiscreteKeys& discreteKeys) {
  std::set<DiscreteKey> s(discreteKeys.begin(), discreteKeys.end());
  return s;
}

/* *******************************************************************************/
HybridGaussianConditional::shared_ptr HybridGaussianConditional::prune(
    const DecisionTreeFactor& discreteProbs) const {
  // Find keys in discreteProbs.keys() but not in this->keys():
  std::set<Key> mine(this->keys().begin(), this->keys().end());
  std::set<Key> theirs(discreteProbs.keys().begin(), discreteProbs.keys().end());
  std::vector<Key> diff;
  std::set_difference(
      theirs.begin(), theirs.end(), mine.begin(), mine.end(), std::back_inserter(diff));

  // Find maximum probability value for every combination of our keys.
  Ordering keys(diff);
  auto max = discreteProbs.max(keys);

  // Check the max value for every combination of our keys.
  // If the max value is 0.0, we can prune the corresponding conditional.
  auto pruner =
      [&](const Assignment<Key>& choices,
          const GaussianConditional::shared_ptr& conditional) -> GaussianConditional::shared_ptr {
    return (max->evaluate(choices) == 0.0) ? nullptr : conditional;
  };

  auto pruned_conditionals = conditionals_.apply(pruner);
  return std::make_shared<HybridGaussianConditional>(discreteKeys(), pruned_conditionals);
}

/* *******************************************************************************/
double HybridGaussianConditional::logProbability(const HybridValues& values) const {
  auto conditional = conditionals_(values.discrete());
  return conditional->logProbability(values.continuous());
}

/* *******************************************************************************/
double HybridGaussianConditional::evaluate(const HybridValues& values) const {
  auto conditional = conditionals_(values.discrete());
  return conditional->evaluate(values.continuous());
}

}  // namespace gtsam
