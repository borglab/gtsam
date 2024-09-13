/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   HybridNonlinearFactorGraph.cpp
 * @brief  Nonlinear hybrid factor graph that uses type erasure
 * @author Varun Agrawal
 * @date   May 28, 2022
 */

#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/TableFactor.h>
#include <gtsam/hybrid/GaussianMixture.h>
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
#include <gtsam/hybrid/HybridNonlinearFactorGraph.h>
#include <gtsam/hybrid/MixtureFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/* ************************************************************************* */
void HybridNonlinearFactorGraph::print(const std::string& s,
                                       const KeyFormatter& keyFormatter) const {
  // Base::print(str, keyFormatter);
  std::cout << (s.empty() ? "" : s + " ") << std::endl;
  std::cout << "size: " << size() << std::endl;
  for (size_t i = 0; i < factors_.size(); i++) {
    std::stringstream ss;
    ss << "factor " << i << ": ";
    if (factors_[i]) {
      factors_[i]->print(ss.str(), keyFormatter);
      std::cout << std::endl;
    }
  }
}

/* ************************************************************************* */
void HybridNonlinearFactorGraph::printErrors(
    const HybridValues& values, const std::string& str,
    const KeyFormatter& keyFormatter,
    const std::function<bool(const Factor* /*factor*/, double /*whitenedError*/,
                             size_t /*index*/)>& printCondition) const {
  std::cout << str << "size: " << size() << std::endl << std::endl;

  std::stringstream ss;

  for (size_t i = 0; i < factors_.size(); i++) {
    auto&& factor = factors_[i];
    std::cout << "Factor " << i << ": ";

    // Clear the stringstream
    ss.str(std::string());

    if (auto mf = std::dynamic_pointer_cast<MixtureFactor>(factor)) {
      if (factor == nullptr) {
        std::cout << "nullptr"
                  << "\n";
      } else {
        factor->print(ss.str(), keyFormatter);
        std::cout << "error = ";
        mf->errorTree(values.nonlinear()).print("", keyFormatter);
        std::cout << std::endl;
      }
    } else if (auto gmf =
                   std::dynamic_pointer_cast<HybridGaussianFactor>(factor)) {
      if (factor == nullptr) {
        std::cout << "nullptr"
                  << "\n";
      } else {
        factor->print(ss.str(), keyFormatter);
        std::cout << "error = ";
        gmf->errorTree(values.continuous()).print("", keyFormatter);
        std::cout << std::endl;
      }
    } else if (auto gm = std::dynamic_pointer_cast<GaussianMixture>(factor)) {
      if (factor == nullptr) {
        std::cout << "nullptr"
                  << "\n";
      } else {
        factor->print(ss.str(), keyFormatter);
        std::cout << "error = ";
        gm->errorTree(values.continuous()).print("", keyFormatter);
        std::cout << std::endl;
      }
    } else if (auto nf = std::dynamic_pointer_cast<NonlinearFactor>(factor)) {
      const double errorValue = (factor != nullptr ? nf->error(values) : .0);
      if (!printCondition(factor.get(), errorValue, i))
        continue;  // User-provided filter did not pass

      if (factor == nullptr) {
        std::cout << "nullptr"
                  << "\n";
      } else {
        factor->print(ss.str(), keyFormatter);
        std::cout << "error = " << errorValue << "\n";
      }
    } else if (auto gf = std::dynamic_pointer_cast<GaussianFactor>(factor)) {
      const double errorValue = (factor != nullptr ? gf->error(values) : .0);
      if (!printCondition(factor.get(), errorValue, i))
        continue;  // User-provided filter did not pass

      if (factor == nullptr) {
        std::cout << "nullptr"
                  << "\n";
      } else {
        factor->print(ss.str(), keyFormatter);
        std::cout << "error = " << errorValue << "\n";
      }
    } else if (auto df = std::dynamic_pointer_cast<DiscreteFactor>(factor)) {
      if (factor == nullptr) {
        std::cout << "nullptr"
                  << "\n";
      } else {
        factor->print(ss.str(), keyFormatter);
        std::cout << "error = ";
        df->errorTree().print("", keyFormatter);
        std::cout << std::endl;
      }

    } else {
      continue;
    }

    std::cout << "\n";
  }
  std::cout.flush();
}

/* ************************************************************************* */
HybridGaussianFactorGraph::shared_ptr HybridNonlinearFactorGraph::linearize(
    const Values& continuousValues) const {
  using std::dynamic_pointer_cast;

  // create an empty linear FG
  auto linearFG = std::make_shared<HybridGaussianFactorGraph>();

  linearFG->reserve(size());

  // linearize all hybrid factors
  for (auto& f : factors_) {
    // First check if it is a valid factor
    if (!f) {
      continue;
    }
    // Check if it is a nonlinear mixture factor
    if (auto mf = dynamic_pointer_cast<MixtureFactor>(f)) {
      const HybridGaussianFactor::shared_ptr& gmf =
          mf->linearize(continuousValues);
      linearFG->push_back(gmf);
    } else if (auto nlf = dynamic_pointer_cast<NonlinearFactor>(f)) {
      const GaussianFactor::shared_ptr& gf = nlf->linearize(continuousValues);
      linearFG->push_back(gf);
    } else if (dynamic_pointer_cast<DiscreteFactor>(f)) {
      // If discrete-only: doesn't need linearization.
      linearFG->push_back(f);
    } else if (auto gmf = dynamic_pointer_cast<HybridGaussianFactor>(f)) {
      linearFG->push_back(gmf);
    } else if (auto gm = dynamic_pointer_cast<GaussianMixture>(f)) {
      linearFG->push_back(gm);
    } else if (dynamic_pointer_cast<GaussianFactor>(f)) {
      linearFG->push_back(f);
    } else {
      auto& fr = *f;
      throw std::invalid_argument(
          std::string("HybridNonlinearFactorGraph::linearize: factor type "
                      "not handled: ") +
          demangle(typeid(fr).name()));
    }
  }
  return linearFG;
}

}  // namespace gtsam
