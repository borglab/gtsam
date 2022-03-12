/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   CLGaussianConditional.h
 * @brief  A hybrid conditional in the Conditional Linear Gaussian scheme
 * @author Fan Jiang
 * @date   Mar 12, 2022
 */

#include <gtsam/inference/Conditional.h>
#include <gtsam/hybrid/HybridFactor.h>

namespace gtsam {
class CLGaussianConditional : public HybridFactor, public Conditional<HybridFactor, CLGaussianConditional> {
public:
  using This = CLGaussianConditional;
  using shared_ptr = boost::shared_ptr<CLGaussianConditional>;
  using BaseFactor = HybridFactor;


};
}