/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2022, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   HybridBayesNet.cpp
 * @brief  A Bayes net of Gaussian Conditionals indexed by discrete keys.
 * @author Fan Jiang
 * @author Varun Agrawal
 * @author Shangjie Xue
 * @author Frank Dellaert
 * @date   January 2022
 */

#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/discrete/TableDistribution.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridValues.h>

#include <memory>

namespace gtsam {

/* ************************************************************************* */
void HybridBayesNet::print(const std::string &s,
                           const KeyFormatter &formatter) const {
  Base::print(s, formatter);
}

/* ************************************************************************* */
bool HybridBayesNet::equals(const This &bn, double tol) const {
  return Base::equals(bn, tol);
}

/* ************************************************************************* */
HybridBayesNet HybridBayesNet::prune(
    size_t maxNrLeaves, const std::optional<double> &marginalThreshold,
    DiscreteValues *fixedValues) const {
#if GTSAM_HYBRID_TIMING
  gttic_(HybridPruning);
#endif
  // Collect all the discrete conditionals. Could be small if already pruned.
  const DiscreteBayesNet marginal = discreteMarginal();

  // We use a separate value here since we need this to perform `restrict` on
  // the HybridConditionals later.
  DiscreteValues fixed;
  // Prune discrete Bayes net
  DiscreteBayesNet prunedBN =
      marginal.prune(maxNrLeaves, marginalThreshold, &fixed);

  // Multiply into one big conditional.
  // NOTE: This is cheap since marginal.prune above creates a
  // DBN with a single joint conditional.
  DiscreteConditional pruned = prunedBN.joint();

  // Set the fixed values if requested.
  if (marginalThreshold && fixedValues) {
    if (!fixedValues->empty()) {
      throw std::invalid_argument(
          "HybridBayesNet::prune: fixedValues should be empty since it is "
          "a purely output argument.");
    }
    *fixedValues = fixed;
  }

  HybridBayesNet result;
  result.reserve(size());

  // Go through all the Gaussian conditionals, restrict them according to
  // fixed values, and then prune further.
  for (std::shared_ptr<HybridConditional> conditional : *this) {
    if (conditional->isDiscrete()) continue;

    // No-op if not a HybridGaussianConditional.
    if (marginalThreshold) {
      conditional = std::static_pointer_cast<HybridConditional>(
          conditional->restrict(fixed));
    }

    // Now decide on type what to do:
    if (auto hgc = conditional->asHybrid()) {
      // Prune the hybrid Gaussian conditional!
      auto prunedHybridGaussianConditional = hgc->prune(pruned);
      if (!prunedHybridGaussianConditional) {
        throw std::runtime_error(
            "A HybridGaussianConditional had all its conditionals pruned");
      }
      // Type-erase and add to the pruned Bayes Net fragment.
      result.push_back(prunedHybridGaussianConditional);
    } else if (conditional->isContinuous()) {
      // Add the non-Hybrid GaussianConditional conditional
      result.push_back(conditional);
    } else
      throw std::runtime_error(
          "HybrdiBayesNet::prune: Unknown HybridConditional type.");
  }

#if GTSAM_HYBRID_TIMING
  gttoc_(HybridPruning);
#endif

  // Add the pruned discrete conditionals to the result.
  for (const DiscreteConditional::shared_ptr &discrete : prunedBN)
    result.push_back(discrete);

  return result;
}

/* ************************************************************************* */
DiscreteBayesNet HybridBayesNet::discreteMarginal() const {
  DiscreteBayesNet result;
  for (auto &&conditional : *this) {
    if (auto dc = conditional->asDiscrete()) {
      result.push_back(dc);
    }
  }
  return result;
}

/* ************************************************************************* */
GaussianBayesNet HybridBayesNet::choose(
    const DiscreteValues &assignment) const {
  GaussianBayesNet gbn;
  for (auto &&conditional : *this) {
    if (auto gm = conditional->asHybrid()) {
      // If conditional is hybrid, select based on assignment.
      gbn.push_back(gm->choose(assignment));
    } else if (auto gc = conditional->asGaussian()) {
      // If continuous only, add Gaussian conditional.
      gbn.push_back(gc);
    } else if (auto dc = conditional->asDiscrete()) {
      // If conditional is discrete-only, we simply continue.
      continue;
    }
  }

  return gbn;
}

/* ************************************************************************* */
DiscreteValues HybridBayesNet::mpe() const {
  // Collect all the discrete factors to compute MPE
  DiscreteFactorGraph discrete_fg;

  for (auto &&conditional : *this) {
    if (conditional->isDiscrete()) {
      if (auto dtc = conditional->asDiscrete<TableDistribution>()) {
        // The number of keys should be small so should not
        // be expensive to convert to DiscreteConditional.
        discrete_fg.push_back(DiscreteConditional(dtc->nrFrontals(),
                                                  dtc->toDecisionTreeFactor()));
      } else {
        discrete_fg.push_back(conditional->asDiscrete());
      }
    }
  }

  return discrete_fg.optimize();
}

/* ************************************************************************* */
HybridValues HybridBayesNet::optimize() const {
  // Solve for the MPE
  DiscreteValues mpe = this->mpe();

  // Given the MPE, compute the optimal continuous values.
  return HybridValues(optimize(mpe), mpe);
}

/* ************************************************************************* */
VectorValues HybridBayesNet::optimize(const DiscreteValues &assignment) const {
  GaussianBayesNet gbn = choose(assignment);

  // Check if there exists a nullptr in the GaussianBayesNet
  // If yes, return an empty VectorValues
  if (std::find(gbn.begin(), gbn.end(), nullptr) != gbn.end()) {
    return VectorValues();
  }
  return gbn.optimize();
}

/* ************************************************************************* */
HybridValues HybridBayesNet::sample(const HybridValues &given,
                                    std::mt19937_64 *rng) const {
  DiscreteBayesNet dbn;
  for (auto &&conditional : *this) {
    if (conditional->isDiscrete()) {
      // If conditional is discrete-only, we add to the discrete Bayes net.
      dbn.push_back(conditional->asDiscrete());
    }
  }
  // Sample a discrete assignment.
  const DiscreteValues assignment = dbn.sample(given.discrete(), rng);
  // Select the continuous Bayes net corresponding to the assignment.
  GaussianBayesNet gbn = choose(assignment);
  // Sample from the Gaussian Bayes net.
  VectorValues sample = gbn.sample(given.continuous(), rng);
  return {sample, assignment};
}

/* ************************************************************************* */
HybridValues HybridBayesNet::sample(std::mt19937_64 *rng) const {
  HybridValues given;
  return sample(given, rng);
}

/* ************************************************************************* */
AlgebraicDecisionTree<Key> HybridBayesNet::errorTree(
    const VectorValues &continuousValues) const {
  AlgebraicDecisionTree<Key> result(0.0);

  // Iterate over each conditional.
  for (auto &&conditional : *this) {
    result = result + conditional->errorTree(continuousValues);
  }

  return result;
}

/* ************************************************************************* */
double HybridBayesNet::negLogConstant(
    const std::optional<DiscreteValues> &discrete) const {
  double negLogNormConst = 0.0;
  // Iterate over each conditional.
  for (auto &&conditional : *this) {
    if (discrete.has_value()) {
      if (auto gm = conditional->asHybrid()) {
        negLogNormConst += gm->choose(*discrete)->negLogConstant();
      } else if (auto gc = conditional->asGaussian()) {
        negLogNormConst += gc->negLogConstant();
      } else if (auto dc = conditional->asDiscrete()) {
        negLogNormConst += dc->choose(*discrete)->negLogConstant();
      } else {
        throw std::runtime_error(
            "Unknown conditional type when computing negLogConstant");
      }
    } else {
      negLogNormConst += conditional->negLogConstant();
    }
  }
  return negLogNormConst;
}

/* ************************************************************************* */
AlgebraicDecisionTree<Key> HybridBayesNet::discretePosterior(
    const VectorValues &continuousValues) const {
  AlgebraicDecisionTree<Key> errors = this->errorTree(continuousValues);
  AlgebraicDecisionTree<Key> p =
      errors.apply([](double error) { return exp(-error); });
  return p / p.sum();
}

/* ************************************************************************* */
double HybridBayesNet::evaluate(const HybridValues &values) const {
  return exp(logProbability(values));
}

/* ************************************************************************* */
HybridGaussianFactorGraph HybridBayesNet::toFactorGraph(
    const VectorValues &measurements) const {
  HybridGaussianFactorGraph fg;

  // For all nodes in the Bayes net, if its frontal variable is in measurements,
  // replace it by a likelihood factor:
  for (auto &&conditional : *this) {
    if (conditional->frontalsIn(measurements)) {
      if (auto gc = conditional->asGaussian()) {
        fg.push_back(gc->likelihood(measurements));
      } else if (auto gm = conditional->asHybrid()) {
        fg.push_back(gm->likelihood(measurements));
      } else {
        throw std::runtime_error("Unknown conditional type");
      }
    } else {
      fg.push_back(conditional);
    }
  }
  return fg;
}

}  // namespace gtsam
