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

#include <gtsam/hybrid/GaussianMixture.h>
#include <gtsam/inference/Conditional.h>

namespace gtsam {

// Instantiate base class
template class Conditional<DCGaussianMixtureFactor, GaussianMixture>;

void GaussianMixture::print(const std::string& s,
                            const KeyFormatter& keyFormatter) const {
  std::cout << (s.empty() ? "" : s + " ");
  std::cout << "[";
  for (Key key : keys()) {
    std::cout << " " << keyFormatter(key);
  }
  std::cout << "; ";
  for (auto&& dk : discreteKeys_) std::cout << keyFormatter(dk.first) << " ";
  std::cout << " ]";
  std::cout << "{\n";
  auto valueFormatter =
      [](const GaussianFactor::shared_ptr& factor) -> std::string {
    if (auto conditional =
            boost::dynamic_pointer_cast<GaussianConditional>(factor))
      return (boost::format(
                  "Gaussian conditional on %d frontals given %d parents") %
              conditional->nrFrontals() % conditional->nrParents())
          .str();
    else
      return "";
  };
  factors_.print("", keyFormatter, valueFormatter);
  std::cout << "}";
  std::cout << "\n";
}
}  // namespace gtsam
