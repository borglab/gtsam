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
#include <gtsam/linear/GaussianMultifrontalSolver.h>
#include <gtsam/linear/GaussianSequentialSolver.h>

using namespace std;

namespace gtsam {

NonlinearOptimizer::auto_ptr LevenbergMarquardtOptimizer::iterate() const {

  // Linearize graph
  GaussianFactorGraph::shared_ptr linear = graph_->linearize(values_, gnParams_->ordering);

  // Check whether to use QR
  const bool useQR;
  if(gnParams_->factorization == LevenbergMarquardtParams::LDL)
    useQR = false;
  else if(gnParams_->factorization == LevenbergMarquardtParams::QR)
    useQR = true;
  else
    throw runtime_error("Optimization parameter is invalid: LevenbergMarquardtParams::factorization");

  // Optimize
  VectorValues::shared_ptr delta;
  if(gnParams_->elimination == MULTIFRONTAL)
    delta = GaussianMultifrontalSolver(*linear, useQR).optimize();
  else if(gnParams_->elimination == SEQUENTIAL)
    delta = GaussianSequentialSolver(*linear, useQR).optimize();
  else
    throw runtime_error("Optimization parameter is invalid: LevenbergMarquardtParams::elimination");


  const NonlinearOptimizerParams::Verbosity verbosity = params_->verbosity;
  const double lambdaFactor = parameters_->lambdaFactor_ ;
  double lambda = params_->lambda;

  double next_error = error_;
  SharedValues next_values = values_;

  // Keep increasing lambda until we make make progress
  while(true) {
    if (verbosity >= Parameters::TRYLAMBDA) cout << "trying lambda = " << lambda << endl;

    // add prior-factors
    // TODO: replace this dampening with a backsubstitution approach
    typename L::shared_ptr dampedSystem(new L(linearSystem));
    {
      double sigma = 1.0 / sqrt(lambda);
      dampedSystem->reserve(dampedSystem->size() + dimensions_->size());
      // for each of the variables, add a prior
      for(Index j=0; j<dimensions_->size(); ++j) {
        size_t dim = (*dimensions_)[j];
        Matrix A = eye(dim);
        Vector b = zero(dim);
        SharedDiagonal model = noiseModel::Isotropic::Sigma(dim,sigma);
        typename L::sharedFactor prior(new JacobianFactor(j, A, b, model));
        dampedSystem->push_back(prior);
      }
    }
    if (verbosity >= Parameters::DAMPED) dampedSystem->print("damped");

    // Create a new solver using the damped linear system
    // FIXME: remove spcg specific code
    if (spcg_solver_) spcg_solver_->replaceFactors(dampedSystem);
    shared_solver solver = (spcg_solver_) ? spcg_solver_ : shared_solver(
        new S(dampedSystem, structure_, parameters_->useQR_));

    // Try solving
    try {
      VectorValues delta = *solver->optimize();
      if (verbosity >= Parameters::TRYLAMBDA) cout << "linear delta norm = " << delta.vector().norm() << endl;
      if (verbosity >= Parameters::TRYDELTA) delta.print("delta");

      // update values
      shared_values newValues(new Values(values_->retract(delta, *ordering_)));

      // create new optimization state with more adventurous lambda
      double error = graph_->error(*newValues);

      if (verbosity >= Parameters::TRYLAMBDA) cout << "next error = " << error << endl;

      if( error <= error_ ) {
        next_values = newValues;
        next_error = error;
        lambda /= lambdaFactor;
        break;
      }
      else {
        // Either we're not cautious, or the same lambda was worse than the current error.
        // The more adventurous lambda was worse too, so make lambda more conservative
        // and keep the same values.
        if(lambdaMode >= Parameters::BOUNDED && lambda >= 1.0e5) {
          if(verbosity >= Parameters::ERROR)
            cout << "Warning:  Levenberg-Marquardt giving up because cannot decrease error with maximum lambda" << endl;
          break;
        } else {
          lambda *= factor;
        }
      }
    } catch(const NegativeMatrixException& e) {
      if(verbosity >= Parameters::LAMBDA)
        cout << "Negative matrix, increasing lambda" << endl;
      // Either we're not cautious, or the same lambda was worse than the current error.
      // The more adventurous lambda was worse too, so make lambda more conservative
      // and keep the same values.
      if(lambdaMode >= Parameters::BOUNDED && lambda >= 1.0e5) {
        if(verbosity >= Parameters::ERROR)
          cout << "Warning:  Levenberg-Marquardt giving up because cannot decrease error with maximum lambda" << endl;
        break;
      } else {
        lambda *= factor;
      }
    } catch(...) {
      throw;
    }
  } // end while

  return newValuesErrorLambda_(next_values, next_error, lambda);


  // Maybe show output
  if(params_->verbosity >= NonlinearOptimizerParams::DELTA) delta->print("delta");

  // Update values
  SharedValues newValues(new Values(values_->retract(*delta, gnParams_->ordering)));
  double newError = graph_->error(newValues);

  // Maybe show output
  if (params_->verbosity >= NonlinearOptimizerParams::VALUES) newValues->print("newValues");
  if (params_->verbosity >= NonlinearOptimizerParams::ERROR) cout << "error: " << newError << endl;

  // Create new optimizer with new values and new error
  auto_ptr<LevenbergMarquardtOptimizer> newOptimizer(new LevenbergMarquardtOptimizer(
      graph_, newValues, gnParams_, newError, iterations_+1));

  return newOptimizer;
}

} /* namespace gtsam */
