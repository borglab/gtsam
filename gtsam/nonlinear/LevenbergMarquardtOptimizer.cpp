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
 * @date	Feb 26, 2012
 */

#include <cmath>

#include <gtsam/linear/linearExceptions.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/SuccessiveLinearizationOptimizer.h>

#include <boost/algorithm/string.hpp>
#include <string>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
LevenbergMarquardtParams::VerbosityLM LevenbergMarquardtParams::verbosityLMTranslator(const std::string &src) const {
  std::string s = src;  boost::algorithm::to_upper(s);
  if (s == "SILENT") return LevenbergMarquardtParams::SILENT;
  if (s == "LAMBDA") return LevenbergMarquardtParams::LAMBDA;
  if (s == "TRYLAMBDA") return LevenbergMarquardtParams::TRYLAMBDA;
  if (s == "TRYCONFIG") return LevenbergMarquardtParams::TRYCONFIG;
  if (s == "TRYDELTA") return LevenbergMarquardtParams::TRYDELTA;
  if (s == "DAMPED") return LevenbergMarquardtParams::DAMPED;

  /* default is silent */
  return LevenbergMarquardtParams::SILENT;
}

/* ************************************************************************* */
std::string LevenbergMarquardtParams::verbosityLMTranslator(VerbosityLM value) const {
  std::string s;
  switch (value) {
  case LevenbergMarquardtParams::SILENT:    s = "SILENT" ;     break;
  case LevenbergMarquardtParams::LAMBDA:    s = "LAMBDA" ;     break;
  case LevenbergMarquardtParams::TRYLAMBDA: s = "TRYLAMBDA" ;  break;
  case LevenbergMarquardtParams::TRYCONFIG: s = "TRYCONFIG" ;  break;
  case LevenbergMarquardtParams::TRYDELTA:  s = "TRYDELTA" ;   break;
  case LevenbergMarquardtParams::DAMPED:    s = "DAMPED" ;     break;
  default:                                  s = "UNDEFINED" ;  break;
  }
  return s;
}

/* ************************************************************************* */
void LevenbergMarquardtParams::print(const std::string& str) const {
  SuccessiveLinearizationParams::print(str);
  std::cout << "              lambdaInitial: " << lambdaInitial << "\n";
  std::cout << "               lambdaFactor: " << lambdaFactor << "\n";
  std::cout << "           lambdaUpperBound: " << lambdaUpperBound << "\n";
  std::cout << "                verbosityLM: " << verbosityLMTranslator(verbosityLM) << "\n";
  std::cout.flush();
}

/* ************************************************************************* */
void LevenbergMarquardtOptimizer::iterate() {

  // Linearize graph
  GaussianFactorGraph::shared_ptr linear = graph_.linearize(state_.values, *params_.ordering);

  // Pull out parameters we'll use
  const NonlinearOptimizerParams::Verbosity nloVerbosity = params_.verbosity;
  const LevenbergMarquardtParams::VerbosityLM lmVerbosity = params_.verbosityLM;

  // Keep increasing lambda until we make make progress
  while(true) {
    if (lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA)
      cout << "trying lambda = " << state_.lambda << endl;

    // Add prior-factors
    // TODO: replace this dampening with a backsubstitution approach
    GaussianFactorGraph dampedSystem(*linear);
    {
      double sigma = 1.0 / std::sqrt(state_.lambda);
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
      // Solve Damped Gaussian Factor Graph
      const VectorValues delta = solveGaussianFactorGraph(dampedSystem, params_);

      if (lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA) cout << "linear delta norm = " << delta.vector().norm() << endl;
      if (lmVerbosity >= LevenbergMarquardtParams::TRYDELTA) delta.print("delta");

      // update values
      Values newValues = state_.values.retract(delta, *params_.ordering);

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
    } catch(const IndeterminantLinearSystemException& e) {
      if(lmVerbosity >= LevenbergMarquardtParams::LAMBDA)
        cout << "Negative matrix, increasing lambda" << endl;
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
    } catch(...) {
      throw;
    }
  } // end while

  // Increment the iteration counter
  ++state_.iterations;
}

} /* namespace gtsam */
