/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file HybridConditional.cpp
 *  @date Mar 11, 2022
 *  @author Fan Jiang
 */

#include <gtsam/hybrid/HybridConditional.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/inference/Conditional-inst.h>
#include <gtsam/inference/Key.h>

namespace gtsam {

/* ************************************************************************ */
HybridConditional::HybridConditional(const KeyVector &continuousFrontals,
                                     const DiscreteKeys &discreteFrontals,
                                     const KeyVector &continuousParents,
                                     const DiscreteKeys &discreteParents)
    : HybridConditional(
          CollectKeys(
              {continuousFrontals.begin(), continuousFrontals.end()},
              KeyVector{continuousParents.begin(), continuousParents.end()}),
          CollectDiscreteKeys(
              {discreteFrontals.begin(), discreteFrontals.end()},
              {discreteParents.begin(), discreteParents.end()}),
          continuousFrontals.size() + discreteFrontals.size()) {}

/* ************************************************************************ */
HybridConditional::HybridConditional(
    const std::shared_ptr<GaussianConditional> &continuousConditional)
    : HybridConditional(continuousConditional->keys(), {},
                        continuousConditional->nrFrontals()) {
  inner_ = continuousConditional;
}

/* ************************************************************************ */
HybridConditional::HybridConditional(
    const std::shared_ptr<DiscreteConditional> &discreteConditional)
    : HybridConditional({}, discreteConditional->discreteKeys(),
                        discreteConditional->nrFrontals()) {
  inner_ = discreteConditional;
}

/* ************************************************************************ */
HybridConditional::HybridConditional(
    const std::shared_ptr<GaussianMixture> &gaussianMixture)
    : BaseFactor(KeyVector(gaussianMixture->keys().begin(),
                           gaussianMixture->keys().begin() +
                               gaussianMixture->nrContinuous()),
                 gaussianMixture->discreteKeys()),
      BaseConditional(gaussianMixture->nrFrontals()) {
  inner_ = gaussianMixture;
}

/* ************************************************************************ */
void HybridConditional::print(const std::string &s,
                              const KeyFormatter &formatter) const {
  std::cout << s;

  if (inner_) {
    inner_->print("", formatter);

  } else {
    if (isContinuous()) std::cout << "Continuous ";
    if (isDiscrete()) std::cout << "Discrete ";
    if (isHybrid()) std::cout << "Hybrid ";
    BaseConditional::print("", formatter);

    std::cout << "P(";
    size_t index = 0;
    const size_t N = keys().size();
    const size_t contN = N - discreteKeys_.size();
    while (index < N) {
      if (index > 0) {
        if (index == nrFrontals_)
          std::cout << " | ";
        else
          std::cout << ", ";
      }
      if (index < contN) {
        std::cout << formatter(keys()[index]);
      } else {
        auto &dk = discreteKeys_[index - contN];
        std::cout << "(" << formatter(dk.first) << ", " << dk.second << ")";
      }
      index++;
    }
  }
}

/* ************************************************************************ */
bool HybridConditional::equals(const HybridFactor &other, double tol) const {
  const This *e = dynamic_cast<const This *>(&other);
  if (e == nullptr) return false;
  if (auto gm = asMixture()) {
    auto other = e->asMixture();
    return other != nullptr && gm->equals(*other, tol);
  }
  if (auto gc = asGaussian()) {
    auto other = e->asGaussian();
    return other != nullptr && gc->equals(*other, tol);
  }
  if (auto dc = asDiscrete()) {
    auto other = e->asDiscrete();
    return other != nullptr && dc->equals(*other, tol);
  }

  return inner_ ? (e->inner_ ? inner_->equals(*(e->inner_), tol) : false)
                : !(e->inner_);
}

/* ************************************************************************ */
double HybridConditional::error(const HybridValues &values) const {
  if (auto gc = asGaussian()) {
    return gc->error(values.continuous());
  }
  if (auto gm = asMixture()) {
    return gm->error(values);
  }
  if (auto dc = asDiscrete()) {
    return dc->error(values.discrete());
  }
  throw std::runtime_error(
      "HybridConditional::error: conditional type not handled");
}

/* ************************************************************************ */
double HybridConditional::logProbability(const HybridValues &values) const {
  if (auto gc = asGaussian()) {
    return gc->logProbability(values.continuous());
  }
  if (auto gm = asMixture()) {
    return gm->logProbability(values);
  }
  if (auto dc = asDiscrete()) {
    return dc->logProbability(values.discrete());
  }
  throw std::runtime_error(
      "HybridConditional::logProbability: conditional type not handled");
}

/* ************************************************************************ */
double HybridConditional::logNormalizationConstant() const {
  if (auto gc = asGaussian()) {
    return gc->logNormalizationConstant();
  }
  if (auto gm = asMixture()) {
    return gm->logNormalizationConstant(); // 0.0!
  }
  if (auto dc = asDiscrete()) {
    return dc->logNormalizationConstant(); // 0.0!
  }
  throw std::runtime_error(
      "HybridConditional::logProbability: conditional type not handled");
}

/* ************************************************************************ */
double HybridConditional::evaluate(const HybridValues &values) const {
  return std::exp(logProbability(values));
}

}  // namespace gtsam
