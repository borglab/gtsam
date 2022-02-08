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

#include <gtsam/base/utilities.h>
#include <gtsam/hybrid/DCGaussianMixtureFactor.h>

namespace gtsam {

/* *******************************************************************************/
void DCGaussianMixtureFactor::printKeys(
    const std::string &s, const KeyFormatter &keyFormatter) const {
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
void DCGaussianMixtureFactor::print(const std::string &s,
                                    const KeyFormatter &keyFormatter) const {
  printKeys(s, keyFormatter);

  auto valueFormatter = [](const GaussianFactor::shared_ptr &v) {
    auto printCapture = [](const GaussianFactor::shared_ptr &p) {
      RedirectCout rd;
      p->print();
      std::string s = rd.str();
      return s;
    };

    std::string format_template = "Gaussian factor on %d keys: \n%s\n";
    if (auto hessianFactor = boost::dynamic_pointer_cast<HessianFactor>(v)) {
      format_template = "Hessian factor on %d keys: \n%s\n";
    }
    if (auto jacobianFactor = boost::dynamic_pointer_cast<JacobianFactor>(v)) {
      format_template = "Jacobian factor on %d keys: \n%s\n";
    }

    return (boost::format(format_template) % v->size() % printCapture(v)).str();
  };

  factors_.print("", keyFormatter, valueFormatter);
  std::cout << "}";
  std::cout << "\n";
}

using Sum = DecisionTree<Key, GaussianFactorGraph>;

/* *******************************************************************************/
Sum DCGaussianMixtureFactor::addTo(const Sum &sum) const {
  using Y = GaussianFactorGraph;
  auto add = [](const Y &graph1, const Y &graph2) {
    auto result = graph1;
    result.push_back(graph2);
    return result;
  };
  const Sum wrapped = wrappedFactors();
  return sum.empty() ? wrapped : sum.apply(wrapped, add);
}

/* *******************************************************************************/
Sum DCGaussianMixtureFactor::wrappedFactors() const {
  auto wrap = [](const GaussianFactor::shared_ptr &factor) {
    GaussianFactorGraph result;
    result.push_back(factor);
    return result;
  };
  return {factors_, wrap};
}

/* *******************************************************************************/
bool DCGaussianMixtureFactor::equals(const DCFactor &f, double tol) const {
  const DCGaussianMixtureFactor *other;
  if ((other = dynamic_cast<const DCGaussianMixtureFactor *>(&f))) {
    return factors_.equals(other->factors_);
  }
  return false;
}
const DCGaussianMixtureFactor::Factors &DCGaussianMixtureFactor::factors() {
  return factors_;
}

/* *******************************************************************************/

}  // namespace gtsam
