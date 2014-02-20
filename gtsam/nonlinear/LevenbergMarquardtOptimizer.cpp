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
 * @date  Feb 26, 2012
 */

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/Errors.h>

#include <boost/algorithm/string.hpp>
#include <string>
#include <cmath>
#include <fstream>

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
  case LevenbergMarquardtParams::SILENT:         s = "SILENT" ;     break;
  case LevenbergMarquardtParams::TERMINATION:    s = "TERMINATION" ;     break;
  case LevenbergMarquardtParams::LAMBDA:         s = "LAMBDA" ;     break;
  case LevenbergMarquardtParams::TRYLAMBDA:      s = "TRYLAMBDA" ;  break;
  case LevenbergMarquardtParams::TRYCONFIG:      s = "TRYCONFIG" ;  break;
  case LevenbergMarquardtParams::TRYDELTA:       s = "TRYDELTA" ;   break;
  case LevenbergMarquardtParams::DAMPED:         s = "DAMPED" ;     break;
  default:                                       s = "UNDEFINED" ;  break;
  }
  return s;
}

/* ************************************************************************* */
void LevenbergMarquardtParams::print(const std::string& str) const {
  NonlinearOptimizerParams::print(str);
  std::cout << "              lambdaInitial: " << lambdaInitial << "\n";
  std::cout << "               lambdaFactor: " << lambdaFactor << "\n";
  std::cout << "           lambdaUpperBound: " << lambdaUpperBound << "\n";
  std::cout << "           lambdaLowerBound: " << lambdaLowerBound << "\n";
  std::cout << "     disableInnerIterations: " << disableInnerIterations << "\n";
  std::cout << "           minModelFidelity: " << minModelFidelity << "\n";
  std::cout << "            diagonalDamping: " << diagonalDamping << "\n";
  std::cout << "                verbosityLM: " << verbosityLMTranslator(verbosityLM) << "\n";
  std::cout.flush();
}

/* ************************************************************************* */
GaussianFactorGraph::shared_ptr LevenbergMarquardtOptimizer::linearize() const {
  return graph_.linearize(state_.values);
}

/* ************************************************************************* */
void LevenbergMarquardtOptimizer::increaseLambda(double stepQuality){
  state_.lambda *= params_.lambdaFactor;
}

/* ************************************************************************* */
void LevenbergMarquardtOptimizer::decreaseLambda(double stepQuality){
  state_.lambda /= params_.lambdaFactor;
}

/* ************************************************************************* */
GaussianFactorGraph LevenbergMarquardtOptimizer::buildDampedSystem(
    const GaussianFactorGraph& linear) {

  //Set two parameters as Ceres, will move out later
  static const double min_diagonal_ = 1e-6;
  static const double max_diagonal_ = 1e32;

  gttic(damp);
  if (params_.verbosityLM >= LevenbergMarquardtParams::DAMPED)
    cout << "building damped system with lambda " << state_.lambda << endl;
  GaussianFactorGraph dampedSystem = linear;
  {
    double sigma = 1.0 / std::sqrt(state_.lambda);
    dampedSystem.reserve(dampedSystem.size() + state_.values.size());
    // Only retrieve diagonal vector when reuse_diagonal = false
    if (params_.diagonalDamping && params_.reuse_diagonal_ == false) {
      state_.hessianDiagonal = linear.hessianDiagonal();
    }
    // for each of the variables, add a prior
    BOOST_FOREACH(const Values::KeyValuePair& key_value, state_.values) {
      size_t dim = key_value.value.dim();
      Matrix A = Matrix::Identity(dim, dim);
      //Replace the identity matrix with diagonal of Hessian
      if (params_.diagonalDamping) {
        A.diagonal() = state_.hessianDiagonal.at(key_value.key);
        for (size_t aa = 0; aa < dim; aa++) {
          if (params_.reuse_diagonal_ == false)
            A(aa, aa) = std::min(std::max(A(aa, aa), min_diagonal_),
                max_diagonal_);
          A(aa, aa) = sqrt(A(aa, aa));
        }
      }
      Vector b = Vector::Zero(dim);
      SharedDiagonal model = noiseModel::Isotropic::Sigma(dim, sigma);
      dampedSystem += boost::make_shared<JacobianFactor>(key_value.key, A, b,
          model);
    }
  }
  gttoc(damp);
  return dampedSystem;
}

/* ************************************************************************* */
void LevenbergMarquardtOptimizer::iterate() {

  gttic (LM_iterate);

  // Pull out parameters we'll use
  const NonlinearOptimizerParams::Verbosity nloVerbosity = params_.verbosity;
  const LevenbergMarquardtParams::VerbosityLM lmVerbosity = params_.verbosityLM;

  // Linearize graph
  if(lmVerbosity >= LevenbergMarquardtParams::DAMPED)
    cout << "linearizing = " << endl;
  GaussianFactorGraph::shared_ptr linear = linearize();

  double modelFidelity = 0.0;

  // Keep increasing lambda until we make make progress
  while (true) {
    ++state_.totalNumberInnerIterations;

    if (lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA)
      cout << "trying lambda = " << state_.lambda << endl;

    // Build damped system for this lambda (adds prior factors that make it like gradient descent)
    GaussianFactorGraph dampedSystem = buildDampedSystem(*linear);

    // Try solving
    try {
      // Log current error/lambda to file
      if (!params_.logFile.empty()) {
        ofstream os(params_.logFile.c_str(), ios::app);

        boost::posix_time::ptime currentTime = boost::posix_time::microsec_clock::universal_time();

        os << state_.iterations << "," << 1e-6 * (currentTime - state_.startTime).total_microseconds() << ","
            << state_.error << "," << state_.lambda << endl;
      }

      // Solve Damped Gaussian Factor Graph
      const VectorValues delta = solve(dampedSystem, state_.values, params_);

      if (lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA) cout << "linear delta norm = " << delta.norm() << endl;
      if (lmVerbosity >= LevenbergMarquardtParams::TRYDELTA) delta.print("delta");

      // update values
      gttic (retract);
      Values newValues = state_.values.retract(delta);
      gttoc(retract);

      // create new optimization state with more adventurous lambda
      gttic (compute_error);

      if(lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA) cout << "calculating error" << endl;
      double error = graph_.error(newValues);
      gttoc(compute_error);

      if (lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA) cout << "next error = " << error << endl;

      // cost change in the original, possibly nonlinear system (old - new)
      double costChange = state_.error - error;

      // cost change in the linearized system (old - new)
      double linearizedCostChange = state_.error - linear->error(delta);

      // checking similarity between change in original and linearized cost
      modelFidelity = costChange / linearizedCostChange;

      if (error < state_.error) {
        state_.values.swap(newValues);
        state_.error = error;
        if(modelFidelity > params_.minModelFidelity){
          decreaseLambda(modelFidelity);
        }else{
          if(state_.lambda < params_.lambdaUpperBound)
            increaseLambda(modelFidelity);
        }
        break;
      } else {
        // Either we're not cautious, or the same lambda was worse than the current error.
        // The more adventurous lambda was worse too, so make lambda more conservative
        // and keep the same values.
        if(state_.lambda >= params_.lambdaUpperBound) {
          if(nloVerbosity >= NonlinearOptimizerParams::TERMINATION)
            cout << "Warning:  Levenberg-Marquardt giving up because cannot decrease error with maximum lambda" << endl;
          break;
        } else {
          if (lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA)
            cout << "increasing lambda: old error (" << state_.error << ") new error (" << error << ")"  << endl;

          increaseLambda(modelFidelity);
        }
//        bool converged = checkConvergence(_params().relativeErrorTol, _params().absoluteErrorTol, _params().errorTol, state_.error, error);
//        cout << " Inner iteration - converged " << converged << endl;
//        cout << "current error " << state_.error << endl;
//        cout << "new error " <<  error << endl;
//        cout << "costChange " << costChange << endl;
//        cout << "linearizedCostChange " << linearizedCostChange << endl;
//        cout << "modelFidelity " << modelFidelity << endl;
      }
    } catch (IndeterminantLinearSystemException& e) {
      (void) e; // Prevent unused variable warning
      if(lmVerbosity >= LevenbergMarquardtParams::TERMINATION)
        cout << "Negative matrix, increasing lambda" << endl;

      // Either we're not cautious, or the same lambda was worse than the current error.
      // The more adventurous lambda was worse too, so make lambda more conservative
      // and keep the same values.
      if(state_.lambda >= params_.lambdaUpperBound) {
        if(nloVerbosity >= NonlinearOptimizerParams::TERMINATION)
          cout << "Warning:  Levenberg-Marquardt giving up because cannot decrease error with maximum lambda" << endl;
        break;
      } else {
        increaseLambda(modelFidelity);
      }
    }

    if(params_.disableInnerIterations)
      break;
    // Frank asks: why would we do that?
    //    catch(...) {
    //      throw;
    //    }
  } // end while

  if (lmVerbosity >= LevenbergMarquardtParams::LAMBDA)
    cout << "using lambda = " << state_.lambda << endl;

  // Increment the iteration counter
  ++state_.iterations;
}

/* ************************************************************************* */
LevenbergMarquardtParams LevenbergMarquardtOptimizer::ensureHasOrdering(
    LevenbergMarquardtParams params, const NonlinearFactorGraph& graph) const
{
  if(!params.ordering)
    params.ordering = Ordering::COLAMD(graph);
  return params;
}

} /* namespace gtsam */

