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

#include <gtsam/base/debug.h>
#include <gtsam/base/timing.h>
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
  pair<GaussianBayesNet::shared_ptr, GaussianFactor::shared_ptr> GaussianFactor::CombineAndEliminate(
      const FactorGraph<GaussianFactor>& factors, size_t nrFrontals, SolveMethod solveMethod) {

    const bool debug = ISDEBUG("GaussianFactor::CombineAndEliminate");

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
      if(debug) cout << "Using QR:";

      tic(1, "convert to Jacobian");
      FactorGraph<JacobianFactor> jacobianFactors;
      jacobianFactors.reserve(factors.size());
      BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, factors) {
        if(factor) {
          JacobianFactor::shared_ptr jacobianFactor(boost::dynamic_pointer_cast<JacobianFactor>(factor));
          if(jacobianFactor) {
            jacobianFactors.push_back(jacobianFactor);
            if(debug) jacobianFactor->print("Existing JacobianFactor: ");
          } else {
            HessianFactor::shared_ptr hessianFactor(boost::dynamic_pointer_cast<HessianFactor>(factor));
            if(!hessianFactor) throw std::invalid_argument(
                "In GaussianFactor::CombineAndEliminate, factor is neither a JacobianFactor nor a HessianFactor.");
            jacobianFactors.push_back(JacobianFactor::shared_ptr(new JacobianFactor(*hessianFactor)));
            if(debug) {
              cout << "Converted HessianFactor to JacobianFactor:\n";
              hessianFactor->print("HessianFactor: ");
              jacobianFactors.back()->print("JacobianFactor: ");
            }
          }
        }
      }
      toc(1, "convert to Jacobian");
      tic(2, "Jacobian CombineAndEliminate");
      pair<GaussianBayesNet::shared_ptr, GaussianFactor::shared_ptr> ret(
          JacobianFactor::CombineAndEliminate(jacobianFactors, nrFrontals));
      toc(2, "Jacobian CombineAndEliminate");
      return ret;

    } else {

      const bool checkCholesky = ISDEBUG("GaussianFactor::CombineAndEliminate Check Cholesky");

      FactorGraph<HessianFactor> hessianFactors;
      tic(1, "convert to Hessian");
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
            HessianFactor::shared_ptr convertedHessianFactor;
            try {
              convertedHessianFactor.reset(new HessianFactor(*jacobianFactor));
              if(checkCholesky)
                if(!assert_equal(HessianFactor(*jacobianFactor), HessianFactor(JacobianFactor(*convertedHessianFactor)), 1e-3))
                  throw runtime_error("Conversion between Jacobian and Hessian incorrect");
            } catch(const exception& e) {
              cout << "Exception converting to Hessian: " << e.what() << endl;
              jacobianFactor->print("jacobianFactor: ");
              convertedHessianFactor->print("convertedHessianFactor: ");
              SETDEBUG("choleskyPartial", true);
              SETDEBUG("choleskyCareful", true);
              HessianFactor(JacobianFactor(*convertedHessianFactor)).print("Jacobian->Hessian->Jacobian->Hessian: ");
              throw;
            }
            hessianFactors.push_back(convertedHessianFactor);
          }
        }
      }
      toc(1, "convert to Hessian");

      pair<GaussianBayesNet::shared_ptr, GaussianFactor::shared_ptr> ret;
      try {
        tic(2, "Hessian CombineAndEliminate");
        ret = HessianFactor::CombineAndEliminate(hessianFactors, nrFrontals);
        toc(2, "Hessian CombineAndEliminate");
      } catch(const exception& e) {
        cout << "Exception in HessianFactor::CombineAndEliminate: " << e.what() << endl;
        SETDEBUG("HessianFactor::CombineAndEliminate", true);
        SETDEBUG("updateATA", true);
        SETDEBUG("JacobianFactor::eliminate", true);
        SETDEBUG("JacobianFactor::Combine", true);
        SETDEBUG("choleskyPartial", true);
        factors.print("Combining factors: ");
        HessianFactor::CombineAndEliminate(hessianFactors, nrFrontals);
        throw;
      }

      if(checkCholesky) {
        pair<GaussianBayesNet::shared_ptr, GaussianFactor::shared_ptr> expected;
        FactorGraph<JacobianFactor> jacobianFactors;
        try {
          // Compare with QR
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
                JacobianFactor::shared_ptr convertedJacobianFactor(new JacobianFactor(*hessianFactor));
                //            if(!assert_equal(*hessianFactor, HessianFactor(*convertedJacobianFactor), 1e-3))
                //              throw runtime_error("Conversion between Jacobian and Hessian incorrect");
                jacobianFactors.push_back(convertedJacobianFactor);
              }
            }
          }
          expected = JacobianFactor::CombineAndEliminate(jacobianFactors, nrFrontals);
        } catch(...) {
          cout << "Exception in QR" << endl;
          throw;
        }

        HessianFactor actual_factor(*ret.second);
        HessianFactor expected_factor(*expected.second);
          if(!assert_equal(*expected.first, *ret.first, 100.0) || !assert_equal(expected_factor, actual_factor, 1.0)) {
            cout << "Cholesky and QR do not agree" << endl;

            SETDEBUG("HessianFactor::CombineAndEliminate", true);
            SETDEBUG("updateATA", true);
            SETDEBUG("JacobianFactor::eliminate", true);
            SETDEBUG("JacobianFactor::Combine", true);
            jacobianFactors.print("Jacobian Factors: ");
            JacobianFactor::CombineAndEliminate(jacobianFactors, nrFrontals);
            HessianFactor::CombineAndEliminate(hessianFactors, nrFrontals);
            factors.print("Combining factors: ");

            throw runtime_error("Cholesky and QR do not agree");
        }
      }

      return ret;
    }

  }

}
