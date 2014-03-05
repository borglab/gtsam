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
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/linear/GaussianEliminationTree.h>
#include <gtsam/inference/VariableIndex.h>

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
  NonlinearOptimizerParams::print(str);
  std::cout << "              lambdaInitial: " << lambdaInitial << "\n";
  std::cout << "               lambdaFactor: " << lambdaFactor << "\n";
  std::cout << "           lambdaUpperBound: " << lambdaUpperBound << "\n";
  std::cout << "                verbosityLM: " << verbosityLMTranslator(verbosityLM) << "\n";
  std::cout.flush();
}

/* ************************************************************************* */
GaussianFactorGraph::shared_ptr LevenbergMarquardtOptimizer::linearize() const {
  return graph_.linearize(state_.values);
}

/* ************************************************************************* */
void LevenbergMarquardtOptimizer::increaseLambda(){
  state_.lambda *= params_.lambdaFactor;
}

/* ************************************************************************* */
void LevenbergMarquardtOptimizer::decreaseLambda(){
  state_.lambda /= params_.lambdaFactor;
}

/* ************************************************************************* */
const Values& LevenbergMarquardtOptimizer::optimize() {
  // Set up graph for re-use, but be sure to delete it before returning
  try {
    // Only can reuse with direct multifrontal solving
    if(params_.linearSolverType == NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY
       || params_.linearSolverType == NonlinearOptimizerParams::MULTIFRONTAL_QR)
    {
      reusableLinearizedGraph_ = GaussianFactorGraph();
      reusableVectorValues_ = VectorValues();
    }
    defaultOptimize();
    reusableLinearizedGraph_.reset();
    reusableVariableIndex_.reset();
    reusableJunctionTree_.reset();
    reusableBayesTree_.reset();
    reusableVectorValues_.reset();
    return values();
  } catch(...) {
    reusableLinearizedGraph_.reset();
    reusableVariableIndex_.reset();
    reusableJunctionTree_.reset();
    reusableBayesTree_.reset();
    reusableVectorValues_.reset();
    throw;
  }
}

/* ************************************************************************* */
void LevenbergMarquardtOptimizer::iterate() {

  gttic (LM_iterate);

  // Pull out parameters we'll use
  const NonlinearOptimizerParams::Verbosity nloVerbosity = params_.verbosity;
  const LevenbergMarquardtParams::VerbosityLM lmVerbosity = params_.verbosityLM;

  // Linearize graph
  if(nloVerbosity >= NonlinearOptimizerParams::ERROR)
    cout << "linearizing = " << endl;

  // Set up damped system
  GaussianFactorGraph* dampedSystem;
  GaussianFactorGraph::shared_ptr _dampedSystemAllocatedHere;
  VectorValues::shared_ptr _vectorValuesAllocatedHere;
  VectorValues* delta;
  if(reusableLinearizedGraph_)
  {
    dampedSystem = &(*reusableLinearizedGraph_);
    delta = &(*reusableVectorValues_);

    if(dampedSystem->empty()) {
      // Create for the first time
      (*dampedSystem) = *linearize();

      // Create damping factors
      dampedSystem->reserve(dampedSystem->size() + state_.values.size());
      BOOST_FOREACH(const Values::KeyValuePair& key_value, state_.values) {
        size_t dim = key_value.value.dim();
        Matrix A = Matrix::Identity(dim, dim);
        Vector b = Vector::Zero(dim);
        SharedDiagonal model = noiseModel::Unit::Create(dim);
        (*dampedSystem) += boost::make_shared<JacobianFactor>(key_value.key, A, b, model);
      }
    } else {
      // Relinearize in place
      GaussianFactorGraph linearizedFactorPointers(dampedSystem->begin(), dampedSystem->begin() + graph_.size());
      graph_.linearizeInPlace(state_.values, linearizedFactorPointers);
    }
  }
  else
  {
    _dampedSystemAllocatedHere = linearize();
    dampedSystem = _dampedSystemAllocatedHere.get();
    _vectorValuesAllocatedHere = boost::make_shared<VectorValues>();
    delta = _vectorValuesAllocatedHere.get();

    // Create damping factors
    dampedSystem->reserve(dampedSystem->size() + state_.values.size());
    BOOST_FOREACH(const Values::KeyValuePair& key_value, state_.values) {
      size_t dim = key_value.value.dim();
      Matrix A = Matrix::Identity(dim, dim);
      Vector b = Vector::Zero(dim);
      SharedDiagonal model = noiseModel::Unit::Create(dim);
      (*dampedSystem) += boost::make_shared<JacobianFactor>(key_value.key, A, b, model);
    }
  }


  // Keep increasing lambda until we make make progress
  while (true) {
    ++state_.totalNumberInnerIterations;
    // Add prior-factors
    // TODO: replace this dampening with a backsubstitution approach
    gttic(damp);
    if (lmVerbosity >= LevenbergMarquardtParams::DAMPED) cout << "building damped system" << endl;
    {
      double sigma = 1.0 / std::sqrt(state_.lambda);
      for(size_t i = graph_.size(); i < dampedSystem->size(); ++i) {
        JacobianFactor& jacobian = dynamic_cast<JacobianFactor&>(*dampedSystem->at(i));
        jacobian.get_model() = noiseModel::Isotropic::Sigma(jacobian.rows(), sigma);
      }
    }
    gttoc(damp);

    // Try solving
    try {
      if (lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA)
        cout << "trying lambda = " << state_.lambda << endl;
      // Log current error/lambda to file
      if (!params_.logFile.empty()) {
        ofstream os(params_.logFile.c_str(), ios::app);

        boost::posix_time::ptime currentTime = boost::posix_time::microsec_clock::universal_time();

        os << state_.iterations << "," << 1e-6 * (currentTime - state_.startTime).total_microseconds() << ","
            << state_.error << "," << state_.lambda << endl;
      }

      if(reusableLinearizedGraph_)
      {
        // Build variable index if needed
        if(!reusableVariableIndex_)
          reusableVariableIndex_ = VariableIndex(*dampedSystem);

        // Build junction tree if needed
        if(!reusableJunctionTree_)
          reusableJunctionTree_ = GaussianJunctionTree(GaussianEliminationTree(*dampedSystem, *reusableVariableIndex_, *params_.ordering));

        // Eliminate or reeliminate
        if(reusableBayesTree_)
          reusableJunctionTree_->eliminateInPlace(*reusableBayesTree_, params_.getEliminationFunction());
        else
          reusableBayesTree_ = *reusableJunctionTree_->eliminate(params_.getEliminationFunction()).first;

        // Get delta
        reusableBayesTree_->optimizeInPlace(*delta);
      }
      else
      {
        *delta = solve(*dampedSystem, state_.values, params_);
      }

      if (lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA) cout << "linear delta norm = " << delta->norm() << endl;
      if (lmVerbosity >= LevenbergMarquardtParams::TRYDELTA) delta->print("delta");

      // update values
      gttic (retract);
      Values newValues = state_.values.retract(*delta);
      gttoc(retract);

      // create new optimization state with more adventurous lambda
      gttic (compute_error);
      if(nloVerbosity >= NonlinearOptimizerParams::ERROR) cout << "calculating error" << endl;
      double error = graph_.error(newValues);
      gttoc(compute_error);

      if (lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA) cout << "next error = " << error << endl;

      if (error <= state_.error) {
        state_.values.swap(newValues);
        state_.error = error;
        decreaseLambda();
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
          if (lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA)
            cout << "increasing lambda: old error (" << state_.error << ") new error (" << error << ")"  << endl;

          increaseLambda();
        }
      }
    } catch (IndeterminantLinearSystemException& e) {
      (void) e; // Prevent unused variable warning
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
        increaseLambda();
      }
    }
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

