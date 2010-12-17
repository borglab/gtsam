/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianFactor.cpp
 * @brief   Linear Factor....A Gaussian
 * @brief   linearFactor
 * @author  Richard Roberts, Christian Potthast
 */

#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/inference/FactorGraph-inl.h>

#include <boost/shared_ptr.hpp>

#include <stdexcept>

using namespace std;

namespace gtsam {

  /* ************************************************************************* */
  std::pair<GaussianBayesNet::shared_ptr, GaussianFactor::shared_ptr> GaussianFactor::CombineAndEliminate(
      const FactorGraph<GaussianFactor>& factors, size_t nrFrontals, SolveMethod solveMethod) {

    // If any JacobianFactors have constrained noise models, we have to convert
    // all factors to JacobianFactors.  Otherwise, we can convert all factors
    // to HessianFactors.  This is because QR can handle constrained noise
    // models but Cholesky cannot.

    // Decide whether to use QR or Cholesky
    bool useQR;
    if(solveMethod == SOLVE_QR) {
      useQR = true;
    } else if(solveMethod == SOLVE_PREFER_CHOLESKY) {
      // Check if any JacobianFactors have constrained noise models.
      useQR = false;
      BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, factors) {
        JacobianFactor::shared_ptr jacobianFactor(boost::dynamic_pointer_cast<JacobianFactor>(factor));
        if(jacobianFactor && jacobianFactor->get_model()->isConstrained()) {
          useQR = true;
          break;
        }
      }
    }

    // Convert all factors to the appropriate type and call the type-specific CombineAndEliminate.
    if(useQR) {
      FactorGraph<JacobianFactor> jacobianFactors;
      jacobianFactors.reserve(factors.size());
      BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, factors) {
        if(factor) {
          JacobianFactor::shared_ptr jacobianFactor(boost::dynamic_pointer_cast<JacobianFactor>(factor));
          if(jacobianFactor)
            jacobianFactors.push_back(jacobianFactor);
          else {
            HessianFactor::shared_ptr hessianFactor(boost::dynamic_pointer_cast<HessianFactor>(factor));
            if(!hessianFactor) throw std::invalid_argument(
                "In GaussianFactor::CombineAndEliminate, factor is neither a JacobianFactor nor a HessianFactor.");
              jacobianFactors.push_back(JacobianFactor::shared_ptr(new JacobianFactor(*hessianFactor)));
          }
        }
      }
      return JacobianFactor::CombineAndEliminate(jacobianFactors, nrFrontals);

    } else {
      FactorGraph<HessianFactor> hessianFactors;
      hessianFactors.reserve(factors.size());
      BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, factors) {
        if(factor) {
          HessianFactor::shared_ptr hessianFactor(boost::dynamic_pointer_cast<HessianFactor>(factor));
          if(hessianFactor)
            hessianFactors.push_back(hessianFactor);
          else {
            JacobianFactor::shared_ptr jacobianFactor(boost::dynamic_pointer_cast<JacobianFactor>(factor));
            if(!jacobianFactor) throw std::invalid_argument(
                "In GaussianFactor::CombineAndEliminate, factor is neither a JacobianFactor nor a HessianFactor.");
              hessianFactors.push_back(HessianFactor::shared_ptr(new HessianFactor(*jacobianFactor)));
          }
        }
      }
      return HessianFactor::CombineAndEliminate(hessianFactors, nrFrontals);
    }
  }

}
