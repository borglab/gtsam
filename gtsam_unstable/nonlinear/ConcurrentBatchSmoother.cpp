/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ConcurrentBatchSmoother.cpp
 * @brief   A Levenberg-Marquardt Batch Smoother that implements the
 *          Concurrent Filtering and Smoothing interface.
 * @author  Stephen Williams
 */

#include <gtsam_unstable/nonlinear/ConcurrentBatchSmoother.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/base/timing.h>
#include <gtsam/base/debug.h>

namespace gtsam {

/* ************************************************************************* */
void ConcurrentBatchSmoother::print(const std::string& s, const KeyFormatter& keyFormatter) const {
  std::cout << s;
  std::cout << "  Factors:" << std::endl;
  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, factors_) {
    PrintNonlinearFactor(factor, "    ", keyFormatter);
  }
  theta_.print("Values:\n");
}

/* ************************************************************************* */
bool ConcurrentBatchSmoother::equals(const ConcurrentSmoother& rhs, double tol) const {
  const ConcurrentBatchSmoother* smoother = dynamic_cast<const ConcurrentBatchSmoother*>(&rhs);
  return smoother
      && factors_.equals(smoother->factors_)
      && theta_.equals(smoother->theta_)
      && ordering_.equals(smoother->ordering_)
      && delta_.equals(smoother->delta_)
      && separatorValues_.equals(smoother->separatorValues_);
}

/* ************************************************************************* */
ConcurrentBatchSmoother::Result ConcurrentBatchSmoother::update(const NonlinearFactorGraph& newFactors, const Values& newTheta) {

  gttic(update);

  // Create the return result meta-data
  Result result;

  // Update all of the internal variables with the new information
  gttic(augment_system);
  {
    // Add the new variables to theta
    theta_.insert(newTheta);
    // Add new variables to the end of the ordering
    std::vector<size_t> dims;
    dims.reserve(newTheta.size());
    BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, newTheta) {
      ordering_.push_back(key_value.key);
      dims.push_back(key_value.value.dim());
    }
    // Augment Delta
    delta_.append(dims);
    for(size_t i = delta_.size() - dims.size(); i < delta_.size(); ++i) {
      delta_[i].setZero();
    }
    // Add the new factors to the graph, updating the variable index
    insertFactors(newFactors);
  }
  gttoc(augment_system);

  if(factors_.size() > 0) {
    // Reorder the system to ensure efficient optimization (and marginalization) performance
    gttic(reorder);
    reorder();
    gttoc(reorder);

    // Optimize the factors using a modified version of L-M
    gttic(optimize);
    result = optimize();
    gttoc(optimize);
  }

  // TODO: The following code does considerable work, much of which could be redundant given the previous optimization step
  // Refactor this code to reduce computational burden

  // Calculate the marginal on the separator from the smoother factors
  if(separatorValues_.size() > 0) {
    gttic(presync);
    updateSmootherSummarization();
    gttoc(presync);
  }

  gttoc(update);

  return result;
}

/* ************************************************************************* */
void ConcurrentBatchSmoother::presync() {

  gttic(presync);

  gttoc(presync);
}

/* ************************************************************************* */
void ConcurrentBatchSmoother::getSummarizedFactors(NonlinearFactorGraph& summarizedFactors, Values& separatorValues) {

  gttic(get_summarized_factors);

  // Copy the previous calculated smoother summarization factors into the output
  summarizedFactors.push_back(smootherSummarization_);

  // Copy the separator values into the output
  separatorValues.insert(separatorValues_);

  gttoc(get_summarized_factors);
}

/* ************************************************************************* */
void ConcurrentBatchSmoother::synchronize(const NonlinearFactorGraph& smootherFactors, const Values& smootherValues,
    const NonlinearFactorGraph& summarizedFactors, const Values& separatorValues) {

  gttic(synchronize);

  // Remove the previous filter summarization from the graph
  removeFactors(filterSummarizationSlots_);

  // Insert new linpoints into the values, augment the ordering, and store new dims to augment delta
  std::vector<size_t> dims;
  dims.reserve(smootherValues.size() + separatorValues.size());
  BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, smootherValues) {
    Values::iterator iter = theta_.find(key_value.key);
    if(iter == theta_.end()) {
      theta_.insert(key_value.key, key_value.value);
      ordering_.push_back(key_value.key);
      dims.push_back(key_value.value.dim());
    } else {
      iter->value = key_value.value;
    }
  }
  BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, separatorValues) {
    Values::iterator iter = theta_.find(key_value.key);
    if(iter == theta_.end()) {
      theta_.insert(key_value.key, key_value.value);
      ordering_.push_back(key_value.key);
      dims.push_back(key_value.value.dim());
    } else {
      iter->value = key_value.value;
    }
  }

  // Augment Delta
  delta_.append(dims);
  for(size_t i = delta_.size() - dims.size(); i < delta_.size(); ++i) {
    delta_[i].setZero();
  }

  // Insert the new smoother factors
  insertFactors(smootherFactors);

  // Insert the new filter summarized factors
  filterSummarizationSlots_ = insertFactors(summarizedFactors);

  // Update the list of root keys
  separatorValues_ = separatorValues;

  gttoc(synchronize);
}

/* ************************************************************************* */
void ConcurrentBatchSmoother::postsync() {

  gttic(postsync);

  gttoc(postsync);
}

/* ************************************************************************* */
std::vector<size_t> ConcurrentBatchSmoother::insertFactors(const NonlinearFactorGraph& factors) {

  gttic(insert_factors);

  // create the output vector
  std::vector<size_t> slots;
  slots.reserve(factors.size());

  // Insert the factor into an existing hole in the factor graph, if possible
  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, factors) {
    size_t slot;
    if(availableSlots_.size() > 0) {
      slot = availableSlots_.front();
      availableSlots_.pop();
      factors_.replace(slot, factor);
    } else {
      slot = factors_.size();
      factors_.push_back(factor);
    }
    slots.push_back(slot);
  }

  gttoc(insert_factors);

  return slots;
}

/* ************************************************************************* */
void ConcurrentBatchSmoother::removeFactors(const std::vector<size_t>& slots) {

  gttic(remove_factors);

  // For each factor slot to delete...
  SymbolicFactorGraph factors;
  BOOST_FOREACH(size_t slot, slots) {
    // Create a symbolic version for the variable index
    factors.push_back(factors_.at(slot)->symbolic(ordering_));

    // Remove the factor from the graph
    factors_.remove(slot);

    // Mark the factor slot as available
    availableSlots_.push(slot);
  }

  gttoc(remove_factors);
}

/* ************************************************************************* */
void ConcurrentBatchSmoother::reorder() {

  // Recalculate the variable index
  variableIndex_ = VariableIndex(*factors_.symbolic(ordering_));

  // Initialize all variables to group0
  std::vector<int> cmember(variableIndex_.size(), 0);

  // Set all of the separator keys to Group1
  if(separatorValues_.size() > 0) {
    BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, separatorValues_) {
      cmember[ordering_.at(key_value.key)] = 1;
    }
  }

  // Generate the permutation
  Permutation forwardPermutation = *inference::PermutationCOLAMD_(variableIndex_, cmember);

  // Permute the ordering, variable index, and deltas
  ordering_.permuteInPlace(forwardPermutation);
  variableIndex_.permuteInPlace(forwardPermutation);
  delta_.permuteInPlace(forwardPermutation);
}

/* ************************************************************************* */
ConcurrentBatchSmoother::Result ConcurrentBatchSmoother::optimize() {

  // Create output result structure
  Result result;
  result.nonlinearVariables = theta_.size() - separatorValues_.size();
  result.linearVariables = separatorValues_.size();

  // Pull out parameters we'll use
  const NonlinearOptimizerParams::Verbosity nloVerbosity = parameters_.verbosity;
  const LevenbergMarquardtParams::VerbosityLM lmVerbosity = parameters_.verbosityLM;
  double lambda = parameters_.lambdaInitial;

  // Create a Values that holds the current evaluation point
  Values evalpoint = theta_.retract(delta_, ordering_);
  result.error = factors_.error(evalpoint);
  if(result.error < parameters_.errorTol) {
    return result;
  }

  // Use a custom optimization loop so the linearization points can be controlled
  double previousError;
  VectorValues newDelta;
  do {
    previousError = result.error;

    // Do next iteration
    gttic(optimizer_iteration);
    {
      // Linearize graph around the linearization point
      GaussianFactorGraph linearFactorGraph = *factors_.linearize(theta_, ordering_);

      // Keep increasing lambda until we make make progress
      while(true) {
        if (lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA)
          std::cout << "trying lambda = " << lambda << std::endl;
        
        // Add prior factors at the current solution
        gttic(damp);
        GaussianFactorGraph dampedFactorGraph(linearFactorGraph);
        dampedFactorGraph.reserve(linearFactorGraph.size() + delta_.size());
        {
          // for each of the variables, add a prior at the current solution
          for(size_t j=0; j<delta_.size(); ++j) {
            Matrix A = eye(delta_[j].size());
            Vector b = delta_[j];
            SharedDiagonal model = noiseModel::Isotropic::Sigma(delta_[j].size(), 1.0 / std::sqrt(lambda));
            GaussianFactor::shared_ptr prior(new JacobianFactor(j, A, b, model));
            dampedFactorGraph.push_back(prior);
          }
        }
        gttoc(damp);
        if (lmVerbosity >= LevenbergMarquardtParams::DAMPED) 
        	dampedFactorGraph.print("damped");
        result.lambdas++;
                
        gttic(solve);
        // Solve Damped Gaussian Factor Graph
        newDelta = GaussianJunctionTree(dampedFactorGraph).optimize(parameters_.getEliminationFunction());
        // update the evalpoint with the new delta
        evalpoint = theta_.retract(newDelta, ordering_);
        gttoc(solve);

        if (lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA) 
          std::cout << "linear delta norm = " << newDelta.norm() << std::endl;
        if (lmVerbosity >= LevenbergMarquardtParams::TRYDELTA) 
        	newDelta.print("delta");

        // Evaluate the new error
        gttic(compute_error);
        double error = factors_.error(evalpoint);
        gttoc(compute_error);

        if (lmVerbosity >= LevenbergMarquardtParams::TRYLAMBDA) 
        	std::cout << "next error = " << error << std::endl;
        
        if(error < result.error) {
          // Keep this change
          // Update the error value
          result.error = error;
          // Update the linearization point
          theta_ = evalpoint;
          // Reset the deltas to zeros
          delta_.setZero();
          // Put the linearization points and deltas back for specific variables
          if(separatorValues_.size() > 0) {
            theta_.update(separatorValues_);
            BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, separatorValues_) {
              Index index = ordering_.at(key_value.key);
              delta_.at(index) = newDelta.at(index);
            }
          }
          // Decrease lambda for next time
          lambda /= parameters_.lambdaFactor;
          // End this lambda search iteration
          break;
        } else {
          // Reject this change
          if(lambda >= parameters_.lambdaUpperBound) {
            // The maximum lambda has been used. Print a warning and end the search.
            std::cout << "Warning:  Levenberg-Marquardt giving up because cannot decrease error with maximum lambda" << std::endl;
            break;
          } else {
            // Increase lambda and continue searching
            lambda *= parameters_.lambdaFactor;
          }
        }
      } // end while
    }
    gttoc(optimizer_iteration);

    if (lmVerbosity >= LevenbergMarquardtParams::LAMBDA)
      std::cout << "using lambda = " << lambda << std::endl;

    result.iterations++;
  } while(result.iterations < parameters_.maxIterations &&
      !checkConvergence(parameters_.relativeErrorTol, parameters_.absoluteErrorTol, parameters_.errorTol, previousError, result.error, NonlinearOptimizerParams::SILENT));

  return result;
}

/* ************************************************************************* */
void ConcurrentBatchSmoother::updateSmootherSummarization() {

  // The smoother summarization factors are the resulting marginal factors on the separator
  // variables that result from marginalizing out all of the other variables
  // These marginal factors will be cached for later transmission to the filter using
  // linear container factors

  // Create a nonlinear factor graph without the filter summarization factors
  NonlinearFactorGraph graph(factors_);
  BOOST_FOREACH(size_t slot, filterSummarizationSlots_) {
    graph.remove(slot);
  }

  // Get the set of separator keys
  gtsam::FastSet<Key> separatorKeys;
  BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, separatorValues_) {
    separatorKeys.insert(key_value.key);
  }

  // Calculate the marginal factors on the separator
  smootherSummarization_ = internal::calculateMarginalFactors(graph, theta_, separatorKeys, parameters_.getEliminationFunction());
}

/* ************************************************************************* */
void ConcurrentBatchSmoother::PrintNonlinearFactor(const NonlinearFactor::shared_ptr& factor, const std::string& indent, const KeyFormatter& keyFormatter) {
  std::cout << indent;
  if(factor) {
    if(boost::dynamic_pointer_cast<LinearContainerFactor>(factor)) {
      std::cout << "l( ";
    } else {
      std::cout << "f( ";
    }
    BOOST_FOREACH(Key key, *factor) {
      std::cout << keyFormatter(key) << " ";
    }
    std::cout << ")" << std::endl;
  } else {
    std::cout << "{ NULL }" << std::endl;
  }
}

/* ************************************************************************* */
void ConcurrentBatchSmoother::PrintLinearFactor(const GaussianFactor::shared_ptr& factor, const Ordering& ordering, const std::string& indent, const KeyFormatter& keyFormatter) {
  std::cout << indent;
  if(factor) {
    std::cout << "g( ";
    BOOST_FOREACH(Index index, *factor) {
      std::cout << keyFormatter(ordering.key(index)) << " ";
    }
    std::cout << ")" << std::endl;
  } else {
    std::cout << "{ NULL }" << std::endl;
  }
}

/* ************************************************************************* */
}/// namespace gtsam
