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
    boost::shared_ptr<GaussianConditional> continuousConditional)
    : HybridConditional(continuousConditional->keys(), {},
                        continuousConditional->nrFrontals()) {
  inner_ = continuousConditional;
}

/* ************************************************************************ */
HybridConditional::HybridConditional(
    boost::shared_ptr<DiscreteConditional> discreteConditional)
    : HybridConditional({}, discreteConditional->discreteKeys(),
                        discreteConditional->nrFrontals()) {
  inner_ = discreteConditional;
}

/* ************************************************************************ */
HybridConditional::HybridConditional(
    boost::shared_ptr<GaussianMixture> gaussianMixture)
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
  return e != nullptr && BaseFactor::equals(*e, tol);
}

}  // namespace gtsam
