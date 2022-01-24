/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   GaussianMixture.cpp
 * @brief  Discrete-continuous conditional density
 * @author Frank Dellaert
 * @date   December 2021
 */

#include <gtsam/base/utilities.h>
#include <gtsam/hybrid/GaussianMixture.h>
#include <gtsam/inference/Conditional.h>

#include <numeric>
#include <regex>

namespace gtsam {

// Instantiate base class
template class Conditional<DCGaussianMixtureFactor, GaussianMixture>;

GaussianMixture::GaussianMixture(size_t nrFrontals,
                                 const KeyVector &continuousKeys,
                                 const DiscreteKeys &discreteKeys,
                                 const Conditionals &conditionals)
    : BaseFactor(
          continuousKeys, discreteKeys,
          // TODO     Keys(conditionals), discreteParentKeys,
          Factors(
              conditionals,
              [nrFrontals](const GaussianConditional::shared_ptr &p) {
                if (p->nrFrontals() != nrFrontals)
                  throw std::invalid_argument(
                      (boost::format(
                           "GaussianMixture() received a conditional with "
                           "invalid number %d of frontals (should be %d).") %
                       nrFrontals % p->nrFrontals())
                          .str());
                return boost::dynamic_pointer_cast<GaussianFactor>(p);
              })),
      BaseConditional(nrFrontals) {}

GaussianConditional::shared_ptr GaussianMixture::operator()(
    DiscreteValues &discreteVals) const {
  auto conditional =
      boost::dynamic_pointer_cast<GaussianConditional>(factors_(discreteVals));
  if (conditional)
    return conditional;
  else
    throw std::logic_error(
        "A GaussianMixture unexpectedly contained a non-conditional");
}

void GaussianMixture::print(const std::string &s,
                            const KeyFormatter &keyFormatter) const {
  std::cout << (s.empty() ? "" : s + " ");
  std::cout << "[";

  for (Key key : frontals()) std::cout << keyFormatter(key) << " ";
  std::cout << "| ";
  for (Key key : parents()) std::cout << keyFormatter(key) << " ";

  std::cout << "]";
  std::cout << "{\n";

  auto valueFormatter = [](const GaussianFactor::shared_ptr &v) {
    auto indenter = [](const GaussianFactor::shared_ptr &p) {
      RedirectCout rd;
      p->print();
      auto contents = rd.str();
      auto re = std::regex("\n");
      auto lines = std::vector<std::string>{
          std::sregex_token_iterator(contents.begin(), contents.end(), re, -1),
          std::sregex_token_iterator()};
      return std::accumulate(
          lines.begin(), lines.end(), std::string(),
          [](const std::string &a, const std::string &b) -> std::string {
            return a + "\n    " + b;
          });
    };

    auto hessianFactor = boost::dynamic_pointer_cast<HessianFactor>(v);
    if (hessianFactor) {
      return (boost::format("Hessian factor on %d keys: \n%s\n") % v->size() %
              indenter(v))
          .str();
    }

    auto jacobianFactor = boost::dynamic_pointer_cast<JacobianFactor>(v);
    if (jacobianFactor) {
      return (boost::format("Jacobian factor on %d keys: \n%s\n") % v->size() %
              indenter(v))
          .str();
    }
    return (boost::format("Gaussian factor on %d keys") % v->size()).str();
  };

  factors_.print("", keyFormatter, valueFormatter);
  std::cout << "}";
  std::cout << "\n";
}

bool GaussianMixture::equals(const DCFactor &f, double tol) const {
  const GaussianMixture *other;
  if (!(other = dynamic_cast<const GaussianMixture *>(&f))) {
    return false;
  } else {
    return factors_.equals(other->factors_,
                           [tol](const GaussianFactor::shared_ptr &a,
                                 const GaussianFactor::shared_ptr &b) {
                             return a->equals(*b, tol);
                           });
  }
}

}  // namespace gtsam
