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
 * @author Shangjie Xue
 * @date   January 2022
 */

#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/hybrid/HybridLookupDAG.h>

namespace gtsam {

/* *******************************************************************************/
HybridValues HybridBayesNet::optimize() const {
  auto dag = HybridLookupDAG::FromBayesNet(*this);
  return dag.argmax();
}

}  // namespace gtsam