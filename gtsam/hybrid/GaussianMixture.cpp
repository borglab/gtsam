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

namespace gtsam {

// Instantiate base class
template class Conditional<DCGaussianMixtureFactor, GaussianMixture>;

void GaussianMixture::print(const std::string &s,
                            const KeyFormatter &keyFormatter) const {
  std::cout << (s.empty() ? "" : s + " ");
  std::cout << "GaussianMixture [";

  for (Key key : frontals()) std::cout << keyFormatter(key) << " ";
  std::cout << "| ";
  for (Key key : parents()) std::cout << keyFormatter(key) << " ";

  std::cout << "]";
  std::cout << "{\n";

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
    if (!v) return std::string {"nullptr\n"};
    return (boost::format(format_template) % v->size() % printCapture(v)).str();
  };

  factors_.print("", keyFormatter, valueFormatter);
  std::cout << "}\n";
}

bool GaussianMixture::equals(const DCFactor &f, double tol) const {
  const GaussianMixture *other;
  if (!(other = dynamic_cast<const GaussianMixture *>(&f))) {
    return false;
  } else {
    return factors_.equals(other->factors_,
                           [tol](const GaussianFactor::shared_ptr &a,
                                 const GaussianFactor::shared_ptr &b) {
                             if (a == nullptr && b == nullptr) return true;
                             if (a == nullptr || b == nullptr) return false;
                             return a->equals(*b, tol);
                           });
  }
}

}  // namespace gtsam
