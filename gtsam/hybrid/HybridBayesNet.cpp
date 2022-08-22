/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   HybridBayesNet.cpp
 * @brief  A bayes net of Gaussian Conditionals indexed by discrete keys.
 * @author Fan Jiang
 * @author Varun Agrawal
 * @author Shangjie Xue
 * @date   January 2022
 */

#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/hybrid/HybridLookupDAG.h>

namespace gtsam {

/* ************************************************************************* */
GaussianMixture::shared_ptr HybridBayesNet::atGaussian(size_t i) const {
  return boost::dynamic_pointer_cast<GaussianMixture>(factors_.at(i)->inner());
}

/* ************************************************************************* */
DiscreteConditional::shared_ptr HybridBayesNet::atDiscrete(size_t i) const {
  return boost::dynamic_pointer_cast<DiscreteConditional>(
      factors_.at(i)->inner());
}

/* ************************************************************************* */
GaussianBayesNet HybridBayesNet::choose(
    const DiscreteValues &assignment) const {
  GaussianBayesNet gbn;
  for (size_t idx = 0; idx < size(); idx++) {
    GaussianMixture gm = *this->atGaussian(idx);
    gbn.push_back(gm(assignment));
  }
  return gbn;
}

/* *******************************************************************************/
HybridValues HybridBayesNet::optimize() const {
  auto dag = HybridLookupDAG::FromBayesNet(*this);
  return dag.argmax();
}

}  // namespace gtsam
