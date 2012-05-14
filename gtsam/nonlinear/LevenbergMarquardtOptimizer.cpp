/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LevenbergMarquardtOptimizer.cpp
 * @brief   
 * @author  Richard Roberts
 * @created Feb 26, 2012
 */

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam/base/cholesky.h> // For NegativeMatrixException
#include <gtsam/linear/GaussianMultifrontalSolver.h>
#include <gtsam/linear/GaussianSequentialSolver.h>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
void LevenbergMarquardtOptimizer::iterate() const {

  // Linearize graph
  GaussianFactorGraph::shared_ptr linear = graph_->linearize(state_.values, *params_.ordering);

  // Check whether to use QR
  bool useQR;
  if(params_.factorization == LevenbergMarquardtParams::LDL)
    useQR = false;
  else if(params_.factorization == LevenbergMarquardtParams::QR)
    useQR = true;
  else
    throw runtime_error("Optimization parameter is invalid: LevenbergMarquardtParams::factorization");

  // Pull out parameters we'll use
  const NonlinearOptimizerParams::Verbosity nloVerbosity = params_.verbosity;
  const LevenbergMarquardtParams::LMVerbosity lmVerbosity = params_.lmVerbosity;

  // Keep increasing lambda until we make make progress
  while(true) {
    if (lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA)
      cout << "trying lambda = " << lambda << endl;

    // Add prior-factors
    // TODO: replace this dampening with a backsubstitution approach
    GaussianFactorGraph dampedSystem(*linear);
    {
      double sigma = 1.0 / sqrt(lambda);
      dampedSystem.reserve(dampedSystem.size() + dimensions_.size());
      // for each of the variables, add a prior
      for(Index j=0; j<dimensions_.size(); ++j) {
        size_t dim = (dimensions_)[j];
        Matrix A = eye(dim);
        Vector b = zero(dim);
        SharedDiagonal model = noiseModel::Isotropic::Sigma(dim,sigma);
        GaussianFactor::shared_ptr prior(new JacobianFactor(j, A, b, model));
        dampedSystem.push_back(prior);
      }
    }
    if (lmVerbosity >= LevenbergMarquardtParams::DAMPED) dampedSystem.print("damped");

    // Try solving
    try {

      // Optimize
      VectorValues::shared_ptr delta;
      if(params_.elimination == LevenbergMarquardtParams::MULTIFRONTAL)
        delta = GaussianMultifrontalSolver(dampedSystem, useQR).optimize();
      else if(params_.elimination == LevenbergMarquardtParams::SEQUENTIAL)
        delta = GaussianSequentialSolver(dampedSystem, useQR).optimize();
      else
        throw runtime_error("Optimization parameter is invalid: LevenbergMarquardtParams::elimination");

      if (lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA) cout << "linear delta norm = " << delta->vector().norm() << endl;
      if (lmVerbosity >= LevenbergMarquardtParams::TRYDELTA) delta->print("delta");

      // update values
      Values newValues = state_.values.retract(*delta, *params_.ordering);

      // create new optimization state with more adventurous lambda
      double error = graph_.error(newValues);

      if (lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA) cout << "next error = " << error << endl;

      if(error <= state_.error) {
        state_.values.swap(newValues);
        state_.error = error;
        state_.lambda /= params_.lambdaFactor;
        break;
      } else {
        // Either we're not cautious, or the same lambda was worse than the current error.
        // The more adventurous lambda was worse too, so make lambda more conservative
        // and keep the same values.
        if(state_.lambda >= params_.lambdaUpperBound) {
          if(nloVerbosity >= NonlinearOptimizerParams::ERROR)
            cout << "Warning:  Levenberg-Marquardt giving up because cannot decrease error with maximum lambda" << endl;
          break;
        } else {
          state_.lambda *= params_.lambdaFactor;
        }
      }
    } catch(const NegativeMatrixException& e) {
      if(lmVerbosity >= LevenbergMarquardtParams::LAMBDA)
        cout << "Negative matrix, increasing lambda" << endl;
      // Either we're not cautious, or the same lambda was worse than the current error.
      // The more adventurous lambda was worse too, so make lambda more conservative
      // and keep the same values.
      if(lambda >= params_.lambdaUpperBound) {
        if(nloVerbosity >= NonlinearOptimizerParams::ERROR)
          cout << "Warning:  Levenberg-Marquardt giving up because cannot decrease error with maximum lambda" << endl;
        break;
      } else {
        state_.lambda *= params_.lambdaFactor;
      }
    } catch(...) {
      throw;
    }
  } // end while

  // Increment the iteration counter
  ++state_.iterations;
}

} /* namespace gtsam */
