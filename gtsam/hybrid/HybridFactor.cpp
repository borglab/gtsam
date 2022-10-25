/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file HybridFactor.cpp
 *  @date Mar 11, 2022
 *  @author Fan Jiang
 */

#include <gtsam/hybrid/HybridFactor.h>

namespace gtsam {

/* ************************************************************************ */
KeyVector CollectKeys(const KeyVector &continuousKeys,
                      const DiscreteKeys &discreteKeys) {
  KeyVector allKeys;
  std::copy(continuousKeys.begin(), continuousKeys.end(),
            std::back_inserter(allKeys));
  std::transform(discreteKeys.begin(), discreteKeys.end(),
                 std::back_inserter(allKeys),
                 [](const DiscreteKey &k) { return k.first; });
  return allKeys;
}

/* ************************************************************************ */
KeyVector CollectKeys(const KeyVector &keys1, const KeyVector &keys2) {
  KeyVector allKeys;
  std::copy(keys1.begin(), keys1.end(), std::back_inserter(allKeys));
  std::copy(keys2.begin(), keys2.end(), std::back_inserter(allKeys));
  return allKeys;
}

/* ************************************************************************ */
DiscreteKeys CollectDiscreteKeys(const DiscreteKeys &key1,
                                 const DiscreteKeys &key2) {
  DiscreteKeys allKeys;
  std::copy(key1.begin(), key1.end(), std::back_inserter(allKeys));
  std::copy(key2.begin(), key2.end(), std::back_inserter(allKeys));
  return allKeys;
}

/* ************************************************************************ */
HybridFactor::HybridFactor(const KeyVector &keys)
    : Base(keys), isContinuous_(true), continuousKeys_(keys) {}

/* ************************************************************************ */
HybridFactor::HybridFactor(const KeyVector &continuousKeys,
                           const DiscreteKeys &discreteKeys)
    : Base(CollectKeys(continuousKeys, discreteKeys)),
      isDiscrete_((continuousKeys.size() == 0) && (discreteKeys.size() != 0)),
      isContinuous_((continuousKeys.size() != 0) && (discreteKeys.size() == 0)),
      isHybrid_((continuousKeys.size() != 0) && (discreteKeys.size() != 0)),
      discreteKeys_(discreteKeys),
      continuousKeys_(continuousKeys) {}

/* ************************************************************************ */
HybridFactor::HybridFactor(const DiscreteKeys &discreteKeys)
    : Base(CollectKeys({}, discreteKeys)),
      isDiscrete_(true),
      discreteKeys_(discreteKeys),
      continuousKeys_({}) {}

/* ************************************************************************ */
bool HybridFactor::equals(const HybridFactor &lf, double tol) const {
  const This *e = dynamic_cast<const This *>(&lf);
  return e != nullptr && Base::equals(*e, tol) &&
         isDiscrete_ == e->isDiscrete_ && isContinuous_ == e->isContinuous_ &&
         isHybrid_ == e->isHybrid_ && continuousKeys_ == e->continuousKeys_ &&
         discreteKeys_ == e->discreteKeys_;
}

/* ************************************************************************ */
void HybridFactor::print(const std::string &s,
                         const KeyFormatter &formatter) const {
  std::cout << s;
  if (isContinuous_) std::cout << "Continuous ";
  if (isDiscrete_) std::cout << "Discrete ";
  if (isHybrid_) std::cout << "Hybrid ";
  std::cout << "[";
  for (size_t c = 0; c < continuousKeys_.size(); c++) {
    std::cout << formatter(continuousKeys_.at(c));
    if (c < continuousKeys_.size() - 1) {
      std::cout << " ";
    } else {
      std::cout << (discreteKeys_.size() > 0 ? "; " : "");
    }
  }
  for (size_t d = 0; d < discreteKeys_.size(); d++) {
    std::cout << formatter(discreteKeys_.at(d).first);
    if (d < discreteKeys_.size() - 1) {
      std::cout << " ";
    }
  }
  std::cout << "]";
}

}  // namespace gtsam
