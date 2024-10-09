/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file HybridGaussianProductFactor.h
 *  @date Oct 2, 2024
 *  @author Frank Dellaert
 *  @author Varun Agrawal
 */

#include <gtsam/base/types.h>
#include <gtsam/discrete/DecisionTree.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/hybrid/HybridGaussianProductFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>

#include <string>

namespace gtsam {

using Y = GaussianFactorGraphValuePair;

/* *******************************************************************************/
static Y add(const Y& y1, const Y& y2) {
  GaussianFactorGraph result = y1.first;
  result.push_back(y2.first);
  return {result, y1.second + y2.second};
};

/* *******************************************************************************/
HybridGaussianProductFactor operator+(const HybridGaussianProductFactor& a,
                                      const HybridGaussianProductFactor& b) {
  return a.empty() ? b : HybridGaussianProductFactor(a.apply(b, add));
}

/* *******************************************************************************/
HybridGaussianProductFactor HybridGaussianProductFactor::operator+(
    const HybridGaussianFactor& factor) const {
  return *this + factor.asProductFactor();
}

/* *******************************************************************************/
HybridGaussianProductFactor HybridGaussianProductFactor::operator+(
    const GaussianFactor::shared_ptr& factor) const {
  return *this + HybridGaussianProductFactor(factor);
}

/* *******************************************************************************/
HybridGaussianProductFactor& HybridGaussianProductFactor::operator+=(
    const GaussianFactor::shared_ptr& factor) {
  *this = *this + factor;
  return *this;
}

/* *******************************************************************************/
HybridGaussianProductFactor& HybridGaussianProductFactor::operator+=(
    const HybridGaussianFactor& factor) {
  *this = *this + factor;
  return *this;
}

/* *******************************************************************************/
void HybridGaussianProductFactor::print(const std::string& s,
                                        const KeyFormatter& formatter) const {
  // KeySet keys;
  // auto printer = [&](const Y& y) {
  //   if (keys.empty()) keys = y.first.keys();
  //   return "Graph of size " + std::to_string(y.first.size()) +
  //          ", scalar sum: " + std::to_string(y.second);
  // };
  // Base::print(s, formatter, printer);
  // if (!keys.empty()) {
  //   std::cout << s << " Keys:";
  //   for (auto&& key : keys) std::cout << " " << formatter(key);
  //   std::cout << "." << std::endl;
  // }
  std::cout << "HybridGaussianProductFactor" << std::endl;
}

/* *******************************************************************************/
bool HybridGaussianProductFactor::equals(
    const HybridGaussianProductFactor& other, double tol) const {
  return Base::equals(other, [tol](const Y& a, const Y& b) {
    return a.first.equals(b.first, tol) && std::abs(a.second - b.second) < tol;
  });
}

/* *******************************************************************************/
HybridGaussianProductFactor HybridGaussianProductFactor::removeEmpty() const {
  auto emptyGaussian = [](const Y& y) {
    bool hasNull =
        std::any_of(y.first.begin(), y.first.end(),
                    [](const GaussianFactor::shared_ptr& ptr) { return !ptr; });
    return hasNull ? Y{GaussianFactorGraph(), 0.0} : y;
  };
  return {Base(*this, emptyGaussian)};
}

}  // namespace gtsam
