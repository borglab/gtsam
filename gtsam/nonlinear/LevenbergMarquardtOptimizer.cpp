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
 * @author  Luca Carlone
 * @date  Feb 26, 2012
 */

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/Errors.h>
#include <gtsam/base/timing.h>

#include <boost/algorithm/string.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/format.hpp>

#include <fstream>
#include <limits>
#include <string>
#include <cmath>

using namespace std;

namespace gtsam {

using boost::adaptors::map_values;

/* ************************************************************************* */
LevenbergMarquardtParams::VerbosityLM LevenbergMarquardtParams::verbosityLMTranslator(
    const std::string &src) {
  std::string s = src;
  boost::algorithm::to_upper(s);
  if (s == "SILENT")
    return LevenbergMarquardtParams::SILENT;
  if (s == "SUMMARY")
    return LevenbergMarquardtParams::SUMMARY;
  if (s == "LAMBDA")
    return LevenbergMarquardtParams::LAMBDA;
  if (s == "TRYLAMBDA")
    return LevenbergMarquardtParams::TRYLAMBDA;
  if (s == "TRYCONFIG")
    return LevenbergMarquardtParams::TRYCONFIG;
  if (s == "TRYDELTA")
    return LevenbergMarquardtParams::TRYDELTA;
  if (s == "DAMPED")
    return LevenbergMarquardtParams::DAMPED;

  /* default is silent */
  return LevenbergMarquardtParams::SILENT;
}

/* ************************************************************************* */
std::string LevenbergMarquardtParams::verbosityLMTranslator(
    VerbosityLM value) {
  std::string s;
  switch (value) {
  case LevenbergMarquardtParams::SILENT:
    s = "SILENT";
    break;
  case LevenbergMarquardtParams::SUMMARY:
    s = "SUMMARY";
    break;
  case LevenbergMarquardtParams::TERMINATION:
    s = "TERMINATION";
    break;
  case LevenbergMarquardtParams::LAMBDA:
    s = "LAMBDA";
    break;
  case LevenbergMarquardtParams::TRYLAMBDA:
    s = "TRYLAMBDA";
    break;
  case LevenbergMarquardtParams::TRYCONFIG:
    s = "TRYCONFIG";
    break;
  case LevenbergMarquardtParams::TRYDELTA:
    s = "TRYDELTA";
    break;
  case LevenbergMarquardtParams::DAMPED:
    s = "DAMPED";
    break;
  default:
    s = "UNDEFINED";
    break;
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
  std::cout << "           minModelFidelity: " << minModelFidelity << "\n";
  std::cout << "            diagonalDamping: " << diagonalDamping << "\n";
  std::cout << "                minDiagonal: " << minDiagonal << "\n";
  std::cout << "                maxDiagonal: " << maxDiagonal << "\n";
  std::cout << "                verbosityLM: "
      << verbosityLMTranslator(verbosityLM) << "\n";
  std::cout.flush();
}

/* ************************************************************************* */
GaussianFactorGraph::shared_ptr LevenbergMarquardtOptimizer::linearize() const {
  return graph_.linearize(state_.values);
}

/* ************************************************************************* */
void LevenbergMarquardtOptimizer::increaseLambda() {
  if (params_.useFixedLambdaFactor) {
    state_.lambda *= params_.lambdaFactor;
  } else {
    state_.lambda *= params_.lambdaFactor;
    params_.lambdaFactor *= 2.0;
  }
  state_.reuseDiagonal = true;
}

/* ************************************************************************* */
void LevenbergMarquardtOptimizer::decreaseLambda(double stepQuality) {

  if (params_.useFixedLambdaFactor) {
    state_.lambda /= params_.lambdaFactor;
  } else {
    // CHECK_GT(step_quality, 0.0);
    state_.lambda *= std::max(1.0 / 3.0, 1.0 - pow(2.0 * stepQuality - 1.0, 3));
    params_.lambdaFactor = 2.0;
  }
  state_.lambda = std::max(params_.lambdaLowerBound, state_.lambda);
  state_.reuseDiagonal = false;

}

/* ************************************************************************* */
GaussianFactorGraph::shared_ptr LevenbergMarquardtOptimizer::buildDampedSystem(
    const GaussianFactorGraph& linear) {

  gttic(damp);
  if (params_.verbosityLM >= LevenbergMarquardtParams::DAMPED)
    cout << "building damped system with lambda " << state_.lambda << endl;

  // Only retrieve diagonal vector when reuse_diagonal = false
  if (params_.diagonalDamping && state_.reuseDiagonal == false) {
    state_.hessianDiagonal = linear.hessianDiagonal();
    BOOST_FOREACH(Vector& v, state_.hessianDiagonal | map_values) {
      for (int aa = 0; aa < v.size(); aa++) {
        v(aa) = std::min(std::max(v(aa), params_.minDiagonal),
            params_.maxDiagonal);
        v(aa) = sqrt(v(aa));
      }
    }
  } // reuse diagonal

  // for each of the variables, add a prior
  double sigma = 1.0 / std::sqrt(state_.lambda);
  GaussianFactorGraph::shared_ptr dampedPtr = linear.cloneToPtr();
  GaussianFactorGraph &damped = (*dampedPtr);
  damped.reserve(damped.size() + state_.values.size());
  if (params_.diagonalDamping) {
    BOOST_FOREACH(const VectorValues::KeyValuePair& key_vector, state_.hessianDiagonal) {
      // Fill in the diagonal of A with diag(hessian)
      try {
        Matrix A = Eigen::DiagonalMatrix<double, Eigen::Dynamic>(
            state_.hessianDiagonal.at(key_vector.first));
        size_t dim = key_vector.second.size();
        Vector b = Vector::Zero(dim);
        SharedDiagonal model = noiseModel::Isotropic::Sigma(dim, sigma);
        damped += boost::make_shared<JacobianFactor>(key_vector.first, A, b,
            model);
      } catch (const std::exception& e) {
        // Don't attempt any damping if no key found in diagonal
        continue;
      }
    }
  } else {
    // Straightforward damping:

    // initialize noise model cache to a reasonable default size
    NoiseCacheVector noises(6);
    BOOST_FOREACH(const Values::KeyValuePair& key_value, state_.values) {
      size_t dim = key_value.value.dim();

      if (dim > noises.size())
        noises.resize(dim);

      NoiseCacheItem& item = noises[dim-1];

      // Initialize noise model, A and b if we haven't done so already
      if(!item.model) {
        item.A = Matrix::Identity(dim, dim);
        item.b = Vector::Zero(dim);
        item.model = noiseModel::Isotropic::Sigma(dim, sigma);
      }
      damped += boost::make_shared<JacobianFactor>(key_value.key, item.A, item.b, item.model);
    }
  }
  gttoc(damp);
  return dampedPtr;
}

/* ************************************************************************* */
// Log current error/lambda to file
inline void LevenbergMarquardtOptimizer::writeLogFile(double currentError){
  if (!params_.logFile.empty()) {
    ofstream os(params_.logFile.c_str(), ios::app);
    boost::posix_time::ptime currentTime = boost::posix_time::microsec_clock::universal_time();
    os << /*inner iterations*/ state_.totalNumberInnerIterations << ","
        << 1e-6 * (currentTime - state_.startTime).total_microseconds() << ","
        << /*current error*/ currentError << "," << state_.lambda << ","
        << /*outer iterations*/ state_.iterations << endl;
  }
}

/* ************************************************************************* */
void LevenbergMarquardtOptimizer::iterate() {

  gttic(LM_iterate);

  // Pull out parameters we'll use
  const NonlinearOptimizerParams::Verbosity nloVerbosity = params_.verbosity;
  const LevenbergMarquardtParams::VerbosityLM lmVerbosity = params_.verbosityLM;

  // Linearize graph
  if (lmVerbosity >= LevenbergMarquardtParams::DAMPED)
    cout << "linearizing = " << endl;
  GaussianFactorGraph::shared_ptr linear = linearize();

  if(state_.totalNumberInnerIterations==0) { // write initial error
    writeLogFile(state_.error);

    if (lmVerbosity == LevenbergMarquardtParams::SUMMARY) {
      cout << "Initial error: " << state_.error << ", values: " << state_.values.size()
           << std::endl;
    }
  }

  // Keep increasing lambda until we make make progress
  while (true) {

#ifdef GTSAM_USING_NEW_BOOST_TIMERS
    boost::timer::cpu_timer lamda_iteration_timer;
    lamda_iteration_timer.start();
#else
    boost::timer lamda_iteration_timer;
    lamda_iteration_timer.restart();
#endif

    if (lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA)
      cout << "trying lambda = " << state_.lambda << endl;

    // Build damped system for this lambda (adds prior factors that make it like gradient descent)
    GaussianFactorGraph::shared_ptr dampedSystemPtr = buildDampedSystem(*linear);
    GaussianFactorGraph &dampedSystem = (*dampedSystemPtr);

    // Try solving
    double modelFidelity = 0.0;
    bool step_is_successful = false;
    bool stopSearchingLambda = false;
    double newError = numeric_limits<double>::infinity(), costChange;
    Values newValues;
    VectorValues delta;

    bool systemSolvedSuccessfully;
    try {
      // ============ Solve is where most computation happens !! =================
      delta = solve(dampedSystem, state_.values, params_);
      systemSolvedSuccessfully = true;
    } catch (const IndeterminantLinearSystemException& e) {
      systemSolvedSuccessfully = false;
    }

    if (systemSolvedSuccessfully) {
      state_.reuseDiagonal = true;

      if (lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA)
        cout << "linear delta norm = " << delta.norm() << endl;
      if (lmVerbosity >= LevenbergMarquardtParams::TRYDELTA)
        delta.print("delta");

      // cost change in the linearized system (old - new)
      double newlinearizedError = linear->error(delta);

      double linearizedCostChange = state_.error - newlinearizedError;
      if (lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA)
              cout << "newlinearizedError = " << newlinearizedError <<
              "  linearizedCostChange = " << linearizedCostChange << endl;

      if (linearizedCostChange >= 0) { // step is valid
        // update values
        gttic(retract);
        // ============ This is where the solution is updated ====================
        newValues = state_.values.retract(delta);
        // =======================================================================
        gttoc(retract);

        // compute new error
        gttic(compute_error);
        if (lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA)
          cout << "calculating error:" << endl;
        newError = graph_.error(newValues);
        gttoc(compute_error);

        if (lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA)
          cout << "old error (" << state_.error
              << ") new (tentative) error (" << newError << ")" << endl;

        // cost change in the original, nonlinear system (old - new)
        costChange = state_.error - newError;

        if (linearizedCostChange > 1e-20) { // the (linear) error has to decrease to satisfy this condition
          // fidelity of linearized model VS original system between
          modelFidelity = costChange / linearizedCostChange;
          // if we decrease the error in the nonlinear system and modelFidelity is above threshold
          step_is_successful = modelFidelity > params_.minModelFidelity;
          if (lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA)
            cout << "modelFidelity: " << modelFidelity << endl;
        } // else we consider the step non successful and we either increase lambda or stop if error change is small

        double minAbsoluteTolerance = params_.relativeErrorTol * state_.error;
        // if the change is small we terminate
        if (fabs(costChange) < minAbsoluteTolerance){
          if (lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA)
                        cout << "fabs(costChange)="<<fabs(costChange) << "  minAbsoluteTolerance="<< minAbsoluteTolerance
                        << " (relativeErrorTol=" << params_.relativeErrorTol << ")" << endl;
          stopSearchingLambda = true;
        }
      }
    }

    if (lmVerbosity == LevenbergMarquardtParams::SUMMARY) {
      // do timing
#ifdef GTSAM_USING_NEW_BOOST_TIMERS
      double iterationTime = 1e-9 * lamda_iteration_timer.elapsed().wall;
#else
      double iterationTime = lamda_iteration_timer.elapsed();
#endif
      if (state_.iterations == 0)
        cout << "iter      cost      cost_change    lambda  success iter_time" << endl;

      cout << boost::format("% 4d % 8e   % 3.2e   % 3.2e  % 4d   % 3.2e") %
                  state_.iterations % newError % costChange % state_.lambda %
                  systemSolvedSuccessfully % iterationTime << endl;
    }

    ++state_.totalNumberInnerIterations;

    if (step_is_successful) { // we have successfully decreased the cost and we have good modelFidelity
      state_.values.swap(newValues);
      state_.error = newError;
      decreaseLambda(modelFidelity);
      writeLogFile(state_.error);
      break;
    } else if (!stopSearchingLambda) { // we failed to solved the system or we had no decrease in cost
      if (lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA)
        cout << "increasing lambda" << endl;
      increaseLambda();
      writeLogFile(state_.error);

      // check if lambda is too big
      if (state_.lambda >= params_.lambdaUpperBound) {
        if (nloVerbosity >= NonlinearOptimizerParams::TERMINATION ||
            lmVerbosity == LevenbergMarquardtParams::SUMMARY)
          cout << "Warning:  Levenberg-Marquardt giving up because "
              "cannot decrease error with maximum lambda" << endl;
        break;
      }
    } else { // the change in the cost is very small and it is not worth trying bigger lambdas
      writeLogFile(state_.error);
      if (lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA)
              cout << "Levenberg-Marquardt: stopping as relative cost reduction is small" << endl;
      break;
    }
  } // end while

  // Increment the iteration counter
  ++state_.iterations;
}

/* ************************************************************************* */
LevenbergMarquardtParams LevenbergMarquardtOptimizer::ensureHasOrdering(
    LevenbergMarquardtParams params, const NonlinearFactorGraph& graph) const {
  if (!params.ordering)
    params.ordering = Ordering::Create(params.orderingType, graph);
  return params;
}

} /* namespace gtsam */

