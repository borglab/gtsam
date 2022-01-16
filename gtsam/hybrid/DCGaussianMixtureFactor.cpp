/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DCGaussianMixtureFactor.cpp
 * @brief Gaussian Mixture formed after linearizing DCGaussianFactor
 * @date Jan 10, 2022
 * @author Varun Agrawal
 * @author Frank Dellaert
 */

#include <gtsam/hybrid/DCGaussianMixtureFactor.h>
#include <regex>
#include <numeric>
#include <gtsam/base/utilities.h>

namespace gtsam {

/* *******************************************************************************/
void DCGaussianMixtureFactor::printKeys(
    const std::string& s, const KeyFormatter& keyFormatter) const {
  std::cout << (s.empty() ? "" : s + " ");
  std::cout << "[";

  KeyVector dKeys = discreteKeys_.indices();
  for (Key key : keys()) {
    if (std::find(dKeys.begin(), dKeys.end(), key) == dKeys.end()) {
      std::cout << " " << keyFormatter(key);
    }
  }

  std::cout << "; ";
  for (Key dk : dKeys) std::cout << keyFormatter(dk) << " ";
  std::cout << "]";
  std::cout << "{\n";
}

/* *******************************************************************************/
void DCGaussianMixtureFactor::print(const std::string& s,
                                    const KeyFormatter& keyFormatter) const {
  printKeys(s, keyFormatter);

  auto valueFormatter = [](const GaussianFactor::shared_ptr& v) {
    auto hessianFactor = boost::dynamic_pointer_cast<HessianFactor>(v);
    if (hessianFactor) {
      RedirectCout rd;
      hessianFactor->print();
      auto contents = rd.str();
      auto lines =
          std::vector<std::string>{std::sregex_token_iterator(contents.begin(), contents.end(), std::regex("\n"), -1),
                                   std::sregex_token_iterator()};
      auto indented = std::accumulate(lines.begin(), lines.end(), std::string(),
                                      [](const std::string &a, const std::string &b) -> std::string {
                                        return a + "\n    " + b;
                                      });
      return (boost::format("Hessian factor on %d keys: \n%s\n") % v->size() % indented).str();
    }

    auto jacobianFactor = boost::dynamic_pointer_cast<JacobianFactor>(v);
    if (jacobianFactor) {
      RedirectCout rd;
      jacobianFactor->print();
      auto contents = rd.str();
      auto lines =
          std::vector<std::string>{std::sregex_token_iterator(contents.begin(), contents.end(), std::regex("\n"), -1),
                                   std::sregex_token_iterator()};
      auto indented = std::accumulate(lines.begin(), lines.end(), std::string("    ----"),
                                      [](const std::string &a, const std::string &b) -> std::string {
                                        return a + (a.length() > 0 ? "\n    ----" : "") + b;
                                      });
      return (boost::format("Jacobian factor on %d keys: \n%s\n") % v->size() % indented).str();
    }
    return (boost::format("Gaussian factor on %d keys") % v->size()).str();
  };
  factors_.print("", keyFormatter, valueFormatter);
  std::cout << "}";
  std::cout << "\n";
}

using Sum = DecisionTree<Key, GaussianFactorGraph>;

/* *******************************************************************************/
Sum DCGaussianMixtureFactor::addTo(const Sum& sum) const {
  using Y = GaussianFactorGraph;
  auto add = [](const Y& graph1, const Y& graph2) {
    auto result = graph1;
    result.push_back(graph2);
    return result;
  };
  const Sum wrapped = wrappedFactors();
  return sum.empty() ? wrapped : sum.apply(wrapped, add);
}

/* *******************************************************************************/
Sum DCGaussianMixtureFactor::wrappedFactors() const {
  auto wrap = [](const GaussianFactor::shared_ptr& factor) {
    GaussianFactorGraph result;
    result.push_back(factor);
    return result;
  };
  return Sum(factors_, wrap);
}

/* *******************************************************************************/
bool DCGaussianMixtureFactor::equals(const DCFactor &f, double tol) const {
  if (typeid(f) == typeid(DCGaussianMixtureFactor)) {
    // TODO: actually do the proper comparison!
    return true;
  }
  return false;
}

/* *******************************************************************************/

}  // namespace gtsam
