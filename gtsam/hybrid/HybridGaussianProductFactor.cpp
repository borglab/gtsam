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

namespace gtsam {

static GaussianFactorGraph add(const GaussianFactorGraph &graph1,
                               const GaussianFactorGraph &graph2) {
  auto result = graph1;
  result.push_back(graph2);
  return result;
};

HybridGaussianProductFactor operator+(const HybridGaussianProductFactor &a,
                                      const HybridGaussianProductFactor &b) {
  return a.empty() ? b : HybridGaussianProductFactor(a.apply(b, add));
}

HybridGaussianProductFactor HybridGaussianProductFactor::operator+(
    const HybridGaussianFactor &factor) const {
  return *this + factor.asProductFactor();
}

HybridGaussianProductFactor HybridGaussianProductFactor::operator+(
    const GaussianFactor::shared_ptr &factor) const {
  return *this + HybridGaussianProductFactor(factor);
}

HybridGaussianProductFactor &HybridGaussianProductFactor::operator+=(
    const GaussianFactor::shared_ptr &factor) {
  *this = *this + factor;
  return *this;
}

HybridGaussianProductFactor &
HybridGaussianProductFactor::operator+=(const HybridGaussianFactor &factor) {
  *this = *this + factor;
  return *this;
}

void HybridGaussianProductFactor::print(const std::string &s,
                                        const KeyFormatter &formatter) const {
  KeySet keys;
  auto printer = [&](const Y &graph) {
    if (keys.size() == 0)
      keys = graph.keys();
    return "Graph of size " + std::to_string(graph.size());
  };
  Base::print(s, formatter, printer);
  if (keys.size() > 0) {
    std::stringstream ss;
    ss << s << " Keys:";
    for (auto &&key : keys)
      ss << " " << formatter(key);
    std::cout << ss.str() << "." << std::endl;
  }
}

HybridGaussianProductFactor HybridGaussianProductFactor::removeEmpty() const {
  auto emptyGaussian = [](const GaussianFactorGraph &graph) {
    bool hasNull =
        std::any_of(graph.begin(), graph.end(),
                    [](const GaussianFactor::shared_ptr &ptr) { return !ptr; });
    return hasNull ? GaussianFactorGraph() : graph;
  };
  return {Base(*this, emptyGaussian)};
}

} // namespace gtsam
