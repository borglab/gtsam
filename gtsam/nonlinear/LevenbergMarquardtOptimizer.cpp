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
#include <boost/range/adaptor/map.hpp>
#include <string>
#include <cmath>
#include <fstream>

using namespace std;


namespace gtsam {

using boost::adaptors::map_values;

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
  if(params_.useFixedLambdaFactor_){
    state_.lambda *= params_.lambdaFactor;
  }else{
    state_.lambda *= params_.lambdaFactor;
    params_.lambdaFactor *= 2.0;
    // reuse_diagonal_ = true;
  }
}

/* ************************************************************************* */
void LevenbergMarquardtOptimizer::decreaseLambda(double stepQuality){

  if(params_.useFixedLambdaFactor_){
    state_.lambda /= params_.lambdaFactor;
  }else{
    // CHECK_GT(step_quality, 0.0);
    state_.lambda *= std::max(1.0 / 3.0, 1.0 - pow(2.0 * stepQuality - 1.0, 3));
    params_.lambdaFactor = 2.0;
    // reuse_diagonal_ = false;
  }
  state_.lambda = std::max(params_.lambdaLowerBound, state_.lambda);
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
      BOOST_FOREACH(Vector& v, state_.hessianDiagonal | map_values)
      {
     	  for( size_t aa = 0 ; aa < v.size() ; aa++)
    	  {
    		  v(aa) = std::min(std::max(v(aa), min_diagonal_),
    	                max_diagonal_);
    		  v(aa) = sqrt(v(aa));
    	  }
      }
    }
    // for each of the variables, add a prior
    BOOST_FOREACH(const Values::KeyValuePair& key_value, state_.values) {
      size_t dim = key_value.value.dim();
      Matrix A = Matrix::Identity(dim, dim);
      //Replace the identity matrix with diagonal of Hessian
      if (params_.diagonalDamping) {
        A.diagonal() = state_.hessianDiagonal.at(key_value.key);
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
  if(lmVerbosity >= LevenbergMarquardtParams::DAMPED) cout << "linearizing = " << endl;
  GaussianFactorGraph::shared_ptr linear = linearize();

  double modelFidelity = 0.0;

  // Keep increasing lambda until we make make progress
  while (true) {

    if (lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA) cout << "trying lambda = " << state_.lambda << endl;

    // Build damped system for this lambda (adds prior factors that make it like gradient descent)
    GaussianFactorGraph dampedSystem = buildDampedSystem(*linear);

    // Try solving
    try {
      // Log current error/lambda to file
      if (!params_.logFile.empty()) {
        ofstream os(params_.logFile.c_str(), ios::app);

        boost::posix_time::ptime currentTime = boost::posix_time::microsec_clock::universal_time();

        os << state_.totalNumberInnerIterations << "," << 1e-6 * (currentTime - state_.startTime).total_microseconds() << ","
            << state_.error << "," << state_.lambda << endl;
      }

      ++state_.totalNumberInnerIterations;

      // Solve Damped Gaussian Factor Graph
      const VectorValues delta = solve(dampedSystem, state_.values, params_);

      if (lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA) cout << "linear delta norm = " << delta.norm() << endl;
      if (lmVerbosity >= LevenbergMarquardtParams::TRYDELTA) delta.print("delta");

      // cost change in the linearized system (old - new)
      double newlinearizedError = linear->error(delta);

      double linearizedCostChange = state_.error - newlinearizedError;

      double error;
      Values newValues;
      bool step_is_successful = false;

      if(linearizedCostChange >= 0){
        // step is valid

        // not implemented
        // iteration_summary.step_norm = (x - x_plus_delta).norm();
        // iteration_summary.step_norm <= step_size_tolerance -> return

        // iteration_summary.cost_change =  cost - new_cost;
        // update values
        gttic (retract);
        newValues = state_.values.retract(delta);
        gttoc(retract);

        // create new optimization state with more adventurous lambda
        gttic (compute_error);
        if(lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA) cout << "calculating error" << endl;
        error = graph_.error(newValues);
        gttoc(compute_error);

        // cost change in the original, possibly nonlinear system (old - new)
        double costChange = state_.error - error;

        double absolute_function_tolerance = params_.relativeErrorTol * state_.error;
        if (fabs(costChange) < absolute_function_tolerance) break; // TODO: check is break is correct

        // fidelity of linearized model VS original system between (relative_decrease in ceres)
        modelFidelity = costChange / linearizedCostChange;

        step_is_successful = modelFidelity > params_.minModelFidelity;

        if (lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA){
          cout << "current error " << state_.error << endl;
          cout << "new error " <<  error << endl;
          cout << "costChange " << costChange << endl;
          cout << "new error in linearized model " <<  newlinearizedError << endl;
          cout << "linearizedCostChange " << linearizedCostChange << endl;
          cout << "modelFidelity " << modelFidelity << endl;
          cout << "step_is_successful " << step_is_successful << endl;
        }
      }

      if(step_is_successful){
        state_.values.swap(newValues);
        state_.error = error;
        decreaseLambda(modelFidelity);
        break;
      }else{
        if (lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA)
          cout << "increasing lambda: old error (" << state_.error << ") new error (" << error << ")"  << endl;
        increaseLambda(modelFidelity);

        // check if lambda is too big
        if(state_.lambda >= params_.lambdaUpperBound) {
          if(nloVerbosity >= NonlinearOptimizerParams::TERMINATION)
            cout << "Warning:  Levenberg-Marquardt giving up because cannot decrease error with maximum lambda" << endl;
          break;
        }
      }

    } catch (IndeterminantLinearSystemException& e) {
      (void) e; // Prevent unused variable warning
      if(lmVerbosity >= LevenbergMarquardtParams::TERMINATION) cout << "Negative matrix, increasing lambda" << endl;

      // Either we're not cautious, or the same lambda was worse than the current error.
      // The more adventurous lambda was worse too, so make lambda more conservative and keep the same values.
      if(state_.lambda >= params_.lambdaUpperBound) {
        if(nloVerbosity >= NonlinearOptimizerParams::TERMINATION)
          cout << "Warning:  Levenberg-Marquardt giving up because cannot decrease error with maximum lambda" << endl;
        break;
      } else {
        cout << " THIS SHOULD NOT HAPPEN IN SMART FACTOR CERES PROJECT " << endl;
        increaseLambda(modelFidelity);
      }
    }

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

