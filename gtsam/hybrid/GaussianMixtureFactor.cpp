/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   GaussianMixtureFactor.cpp
 * @brief  A set of Gaussian factors indexed by a set of discrete keys.
 * @author Fan Jiang
 * @author Varun Agrawal
 * @author Frank Dellaert
 * @date   Mar 12, 2022
 */

#include <gtsam/base/utilities.h>
#include <gtsam/discrete/DecisionTree-inl.h>
#include <gtsam/discrete/DecisionTree.h>
#include <gtsam/hybrid/GaussianMixtureFactor.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>

namespace gtsam {

/**
 * @brief Helper function to correct the [A|b] matrices in the factor components
 * with the normalizer values.
 * This is done by storing the normalizer value in
 * the `b` vector as an additional row.
 *
 * @param factors DecisionTree of GaussianFactor shared pointers.
 * @param varyingNormalizers Flag indicating the normalizers are different for
 * each component.
 * @return GaussianMixtureFactor::Factors
 */
GaussianMixtureFactor::Factors correct(
    const GaussianMixtureFactor::Factors &factors, bool varyingNormalizers) {
  if (!varyingNormalizers) {
    return factors;
  }

  // First compute all the sqrt(|2 pi Sigma|) terms
  auto computeNormalizers = [](const GaussianMixtureFactor::sharedFactor &gf) {
    auto jf = std::dynamic_pointer_cast<JacobianFactor>(gf);
    // If we have, say, a Hessian factor, then no need to do anything
    if (!jf) return 0.0;

    auto model = jf->get_model();
    // If there is no noise model, there is nothing to do.
    if (!model) {
      return 0.0;
    }
    // Since noise models are Gaussian, we can get the logDeterminant using the
    // same trick as in GaussianConditional
    double logDetR =
        model->R().diagonal().unaryExpr([](double x) { return log(x); }).sum();
    double logDeterminantSigma = -2.0 * logDetR;

    size_t n = model->dim();
    constexpr double log2pi = 1.8378770664093454835606594728112;
    return n * log2pi + logDeterminantSigma;
  };

  AlgebraicDecisionTree<Key> log_normalizers =
      DecisionTree<Key, double>(factors, computeNormalizers);

  // Find the minimum value so we can "proselytize" to positive values.
  // Done because we can't have sqrt of negative numbers.
  double min_log_normalizer = log_normalizers.min();
  log_normalizers = log_normalizers.apply(
      [&min_log_normalizer](double n) { return n - min_log_normalizer; });

  // Finally, update the [A|b] matrices.
  auto update = [&log_normalizers](
                    const Assignment<Key> &assignment,
                    const GaussianMixtureFactor::sharedFactor &gf) {
    auto jf = std::dynamic_pointer_cast<JacobianFactor>(gf);
    if (!jf) return gf;
    // If there is no noise model, there is nothing to do.
    if (!jf->get_model()) return gf;
    // If the log_normalizer is 0, do nothing
    if (log_normalizers(assignment) == 0.0) return gf;

    GaussianFactorGraph gfg;
    gfg.push_back(jf);

    Vector c(1);
    c << std::sqrt(log_normalizers(assignment));
    auto constantFactor = std::make_shared<JacobianFactor>(c);

    gfg.push_back(constantFactor);
    return std::dynamic_pointer_cast<GaussianFactor>(
        std::make_shared<JacobianFactor>(gfg));
  };
  return factors.apply(update);
}

/* *******************************************************************************/
GaussianMixtureFactor::GaussianMixtureFactor(const KeyVector &continuousKeys,
                                             const DiscreteKeys &discreteKeys,
                                             const Factors &factors,
                                             bool varyingNormalizers)
    : Base(continuousKeys, discreteKeys),
      factors_(correct(factors, varyingNormalizers)) {}

/* *******************************************************************************/
bool GaussianMixtureFactor::equals(const HybridFactor &lf, double tol) const {
  const This *e = dynamic_cast<const This *>(&lf);
  if (e == nullptr) return false;

  // This will return false if either factors_ is empty or e->factors_ is empty,
  // but not if both are empty or both are not empty:
  if (factors_.empty() ^ e->factors_.empty()) return false;

  // Check the base and the factors:
  return Base::equals(*e, tol) &&
         factors_.equals(e->factors_,
                         [tol](const sharedFactor &f1, const sharedFactor &f2) {
                           return f1->equals(*f2, tol);
                         });
}

/* *******************************************************************************/
void GaussianMixtureFactor::print(const std::string &s,
                                  const KeyFormatter &formatter) const {
  std::cout << (s.empty() ? "" : s + "\n");
  std::cout << "GaussianMixtureFactor" << std::endl;
  HybridFactor::print("", formatter);
  std::cout << "{\n";
  if (factors_.empty()) {
    std::cout << "  empty" << std::endl;
  } else {
    factors_.print(
        "", [&](Key k) { return formatter(k); },
        [&](const sharedFactor &gf) -> std::string {
          RedirectCout rd;
          std::cout << ":\n";
          if (gf) {
            gf->print("", formatter);
            return rd.str();
          } else {
            return "nullptr";
          }
        });
  }
  std::cout << "}" << std::endl;
}

/* *******************************************************************************/
GaussianMixtureFactor::sharedFactor GaussianMixtureFactor::operator()(
    const DiscreteValues &assignment) const {
  return factors_(assignment);
}

/* *******************************************************************************/
GaussianFactorGraphTree GaussianMixtureFactor::add(
    const GaussianFactorGraphTree &sum) const {
  using Y = GaussianFactorGraph;
  auto add = [](const Y &graph1, const Y &graph2) {
    auto result = graph1;
    result.push_back(graph2);
    return result;
  };
  const auto tree = asGaussianFactorGraphTree();
  return sum.empty() ? tree : sum.apply(tree, add);
}

/* *******************************************************************************/
GaussianFactorGraphTree GaussianMixtureFactor::asGaussianFactorGraphTree()
    const {
  auto wrap = [](const sharedFactor &gf) { return GaussianFactorGraph{gf}; };
  return {factors_, wrap};
}

/* *******************************************************************************/
AlgebraicDecisionTree<Key> GaussianMixtureFactor::errorTree(
    const VectorValues &continuousValues) const {
  // functor to convert from sharedFactor to double error value.
  auto errorFunc = [&continuousValues](const sharedFactor &gf) {
    return gf->error(continuousValues);
  };
  DecisionTree<Key, double> error_tree(factors_, errorFunc);
  return error_tree;
}

/* *******************************************************************************/
double GaussianMixtureFactor::error(const HybridValues &values) const {
  const sharedFactor gf = factors_(values.discrete());
  return gf->error(values.continuous());
}
/* *******************************************************************************/

}  // namespace gtsam
