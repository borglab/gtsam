/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BatchFixedLagSmoother.cpp
 * @brief   An LM-based fixed-lag smoother.
 *
 * @author  Michael Kaess, Stephen Williams
 * @date    Oct 14, 2012
 */

#include <gtsam/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianFactor.h>

#include <stdexcept>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
void BatchFixedLagSmoother::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  FixedLagSmoother::print(s, keyFormatter);
  // TODO: What else to print?
}

/* ************************************************************************* */
bool BatchFixedLagSmoother::equals(const FixedLagSmoother& rhs,
    double tol) const {
  const BatchFixedLagSmoother* e =
      dynamic_cast<const BatchFixedLagSmoother*>(&rhs);
  return e != nullptr && FixedLagSmoother::equals(*e, tol)
      && factors_.equals(e->factors_, tol) && theta_.equals(e->theta_, tol);
}

/* ************************************************************************* */
Matrix BatchFixedLagSmoother::marginalCovariance(Key key) const {
  throw runtime_error(
      "BatchFixedLagSmoother::marginalCovariance not implemented");
}

/* ************************************************************************* */
FixedLagSmoother::Result BatchFixedLagSmoother::update(
    const NonlinearFactorGraph& newFactors, const Values& newTheta,
    const KeyTimestampMap& timestamps, const FactorIndices& factorsToRemove) {

  // Capture keys touched by explicit removals before adding new factors. They
  // are removed below only when no replacement factor still references them.
  set<size_t> factorSlotsToRemove;
  KeySet keysWithRemovedFactors;
  for (const size_t factorIndex : factorsToRemove) {
    if (factorIndex >= factors_.size()) {
      throw out_of_range(
          "BatchFixedLagSmoother::update: factor index " +
          to_string(factorIndex) + " is outside the factor graph.");
    }
    // An empty slot may be reused by insertFactors below; it is not a removal.
    if (factors_[factorIndex] &&
        factorSlotsToRemove.insert(factorIndex).second) {
      keysWithRemovedFactors.insert(factors_[factorIndex]->begin(),
                                    factors_[factorIndex]->end());
    }
  }

  // Update all of the internal variables with the new information
  gttic(augment_system);
  // Add the new variables to theta
  theta_.insert(newTheta);
  // Add new variables to the end of the ordering
  for (const auto key : newTheta.keys()) {
    ordering_.push_back(key);
  }
  // Augment Delta
  delta_.insert(newTheta.zeroVectors());

  // Add the new factors to the graph, updating the variable index
  insertFactors(newFactors);
  gttoc(augment_system);

  // Update timestamps before removing unused keys so a removed key cannot
  // reappear as a timestamp-only entry after its state is erased.
  updateKeyTimestampMap(timestamps);

  // Preserve this update's cutoff even if the newest state is removed below.
  const double current_timestamp = getCurrentTimestamp();

  // Remove factors and keep the factor index and reusable slots synchronized.
  removeFactors(factorSlotsToRemove);

  // Removing the last factor touching a key removes that state immediately. A
  // replacement factor in this update keeps the key in the active graph.
  const KeySet activeFactorKeys = factors_.keys();
  KeyVector unusedKeys;
  for (const Key key : keysWithRemovedFactors) {
    if (!activeFactorKeys.exists(key)) unusedKeys.push_back(key);
  }
  eraseKeys(unusedKeys);

  // Find the set of variables to be marginalized out
  KeyVector marginalizableKeys = findKeysBefore(
      current_timestamp - smootherLag_);

  // Values may arrive before the factors that reference them. Once such a
  // value expires, erase it before constructing the constrained ordering,
  // which only contains keys appearing in live factors.
  const KeySet activeFactorKeysAfterRemoval = factors_.keys();
  KeyVector expiredPendingKeys;
  marginalizableKeys.erase(
      remove_if(marginalizableKeys.begin(), marginalizableKeys.end(),
                [&](const Key key) {
                  if (activeFactorKeysAfterRemoval.exists(key)) return false;
                  expiredPendingKeys.push_back(key);
                  return true;
                }),
      marginalizableKeys.end());
  eraseKeys(expiredPendingKeys);

  // Reorder
  gttic(reorder);
  reorder(marginalizableKeys);
  gttoc(reorder);

  // Optimize
  gttic(optimize);
  Result result;
  if (factors_.size() > 0) {
    result = optimize();
  }
  result.expiredPendingKeys = KeySet(expiredPendingKeys);
  gttoc(optimize);

  // Marginalize out old variables.
  gttic(marginalize);
  if (marginalizableKeys.size() > 0) {
    marginalize(marginalizableKeys);
  }
  gttoc(marginalize);

  return result;
}

/* ************************************************************************* */
void BatchFixedLagSmoother::insertFactors(
    const NonlinearFactorGraph& newFactors) {
  for(const auto& factor: newFactors) {
    Key index;
    // Insert the factor into an existing hole in the factor graph, if possible
    if (availableSlots_.size() > 0) {
      index = availableSlots_.front();
      availableSlots_.pop();
      factors_.replace(index, factor);
    } else {
      index = factors_.size();
      factors_.push_back(factor);
    }
    // Update the FactorIndex
    for(Key key: *factor) {
      factorIndex_[key].insert(index);
    }
  }
}

/* ************************************************************************* */
void BatchFixedLagSmoother::removeFactors(
    const set<size_t>& deleteFactors) {
  for(size_t slot: deleteFactors) {
    if (factors_.at(slot)) {
      // Remove references to this factor from the FactorIndex
      for(Key key: *(factors_.at(slot))) {
        factorIndex_[key].erase(slot);
      }
      // Remove the factor from the factor graph
      factors_.remove(slot);
      // Add the factor's old slot to the list of available slots
      availableSlots_.push(slot);
    } else {
      // TODO: Throw an error??
      cout << "Attempting to remove a factor from slot " << slot
          << ", but it is already nullptr." << endl;
    }
  }
}

/* ************************************************************************* */
void BatchFixedLagSmoother::eraseKeys(const KeyVector& keys) {

  for(Key key: keys) {
    // Erase the key from the values
    if (theta_.exists(key)) theta_.erase(key);

    // Erase the key from the factor index
    factorIndex_.erase(key);

    // Erase the key from the set of linearized keys
    if (linearValues_.exists(key)) {
      linearValues_.erase(key);
    }
  }

  eraseKeyTimestampMap(keys);

  // Remove marginalized keys from the ordering and delta
  for(Key key: keys) {
    const auto orderingEntry = find(ordering_.begin(), ordering_.end(), key);
    if (orderingEntry != ordering_.end()) ordering_.erase(orderingEntry);
    if (delta_.exists(key)) delta_.erase(key);
  }
}

/* ************************************************************************* */
void BatchFixedLagSmoother::reorder(const KeyVector& marginalizeKeys) {
  // COLAMD groups will be used to place marginalize keys in Group 0, and everything else in Group 1
  ordering_ = Ordering::ColamdConstrainedFirst(factors_, marginalizeKeys);
}

/* ************************************************************************* */
FixedLagSmoother::Result BatchFixedLagSmoother::optimize() {

  // Create output result structure
  Result result;
  result.nonlinearVariables = theta_.size() - linearValues_.size();
  result.linearVariables = linearValues_.size();

  // Set optimization parameters
  double lambda = parameters_.lambdaInitial;
  double lambdaFactor = parameters_.lambdaFactor;
  double lambdaUpperBound = parameters_.lambdaUpperBound;
  double lambdaLowerBound = 1.0e-10;
  size_t maxIterations = parameters_.maxIterations;
  double relativeErrorTol = parameters_.relativeErrorTol;
  double absoluteErrorTol = parameters_.absoluteErrorTol;
  double errorTol = parameters_.errorTol;

  // Create a Values that holds the current evaluation point
  Values evalpoint = theta_.retract(delta_);
  result.error = factors_.error(evalpoint);

  // check if we're already close enough
  if (result.error <= errorTol) {
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
      GaussianFactorGraph linearFactorGraph = *factors_.linearize(theta_);

      // Keep increasing lambda until we make make progress
      while (true) {

        // Add prior factors at the current solution
        gttic(damp);
        GaussianFactorGraph dampedFactorGraph(linearFactorGraph);
        dampedFactorGraph.reserve(linearFactorGraph.size() + delta_.size());
        {
          // for each of the variables, add a prior at the current solution
          double sigma = 1.0 / sqrt(lambda);
          for(const auto& key_value: delta_) {
            size_t dim = key_value.second.size();
            Matrix A = Matrix::Identity(dim, dim);
            Vector b = key_value.second;
            SharedDiagonal model = noiseModel::Isotropic::Sigma(dim, sigma);
            GaussianFactor::shared_ptr prior(
                new JacobianFactor(key_value.first, A, b, model));
            dampedFactorGraph.push_back(prior);
          }
        }
        gttoc(damp);
        result.intermediateSteps++;

        gttic(solve);
        // Solve Damped Gaussian Factor Graph
        newDelta = dampedFactorGraph.optimize(ordering_,
            parameters_.getEliminationFunction());
        // update the evalpoint with the new delta
        evalpoint = theta_.retract(newDelta);
        gttoc(solve);

        // Evaluate the new error
        gttic(compute_error);
        double error = factors_.error(evalpoint);
        gttoc(compute_error);

        if (error < result.error) {
          // Keep this change
          // Update the error value
          result.error = error;
          // Update the linearization point
          theta_ = evalpoint;
          // Reset the deltas to zeros
          delta_.setZero();
          // Put the linearization points and deltas back for specific variables
          if (enforceConsistency_ && (linearValues_.size() > 0)) {
            theta_.update(linearValues_);
            for(const auto key: linearValues_.keys()) {
              delta_.at(key) = newDelta.at(key);
            }
          }
          // Decrease lambda for next time
          lambda /= lambdaFactor;
          if (lambda < lambdaLowerBound) {
            lambda = lambdaLowerBound;
          }
          // End this lambda search iteration
          break;
        } else {
          // Reject this change
          if (lambda >= lambdaUpperBound) {
            // The maximum lambda has been used. Print a warning and end the search.
            if(parameters_.verbosity >= NonlinearOptimizerParams::TERMINATION
               || parameters_.verbosityLM == LevenbergMarquardtParams::SUMMARY) {
              cout
                << "Warning:  Levenberg-Marquardt giving up because cannot decrease error with maximum lambda"
                << endl;
            }
            break;
          } else {
            // Increase lambda and continue searching
            lambda *= lambdaFactor;
          }
        }
      } // end while
    }
    gttoc(optimizer_iteration);

    result.iterations++;
  } while (result.iterations < maxIterations
      && !checkConvergence(relativeErrorTol, absoluteErrorTol, errorTol,
          previousError, result.error, NonlinearOptimizerParams::SILENT));

  return result;
}

/* ************************************************************************* */
void BatchFixedLagSmoother::marginalize(const KeyVector& marginalizeKeys) {
  // In order to marginalize out the selected variables, the factors involved in those variables
  // must be identified and removed. Also, the effect of those removed factors on the
  // remaining variables needs to be accounted for. This will be done with linear container factors
  // from the result of a partial elimination. This function removes the marginalized factors and
  // adds the linearized factors back in.

  // Identify all of the factors involving any marginalized variable. These must be removed.
  set<size_t> removedFactorSlots;
  const VariableIndex variableIndex(factors_);
  for(Key key: marginalizeKeys) {
    const auto& slots = variableIndex[key];
    removedFactorSlots.insert(slots.begin(), slots.end());
  }

  // Add the removed factors to a factor graph
  NonlinearFactorGraph removedFactors;
  for(size_t slot: removedFactorSlots) {
    if (factors_.at(slot)) {
      removedFactors.push_back(factors_.at(slot));
    }
  }

  // Calculate marginal factors on the remaining keys
  NonlinearFactorGraph marginalFactors = CalculateMarginalFactors(
      removedFactors, theta_, marginalizeKeys, parameters_.getEliminationFunction());

  // Remove marginalized factors from the factor graph
  removeFactors(removedFactorSlots);

  // Remove marginalized keys from the system
  eraseKeys(marginalizeKeys);

  // Insert the new marginal factors
  insertFactors(marginalFactors);

  // Record the linearization points of variables involved in new marginal
  // factors. When enforceConsistency_ is enabled, these variables must keep
  // their current linearization point so the LinearContainerFactors remain
  // valid across future optimization iterations.
  if (enforceConsistency_) {
    for (const auto& factor : marginalFactors) {
      for (Key key : factor->keys()) {
        if (!linearValues_.exists(key)) {
          linearValues_.insert(key, theta_.at(key));
        }
      }
    }
  }
}

/* ************************************************************************* */
void BatchFixedLagSmoother::PrintKeySet(const KeySet& keys,
    const string& label) {
  cout << label;
  for(Key key: keys) {
    cout << " " << DefaultKeyFormatter(key);
  }
  cout << endl;
}

/* ************************************************************************* */
void BatchFixedLagSmoother::PrintSymbolicFactor(
    const NonlinearFactor::shared_ptr& factor) {
  cout << "f(";
  if (factor) {
    for(Key key: factor->keys()) {
      cout << " " << DefaultKeyFormatter(key);
    }
  } else {
    cout << " nullptr";
  }
  cout << " )" << endl;
}

/* ************************************************************************* */
void BatchFixedLagSmoother::PrintSymbolicFactor(
    const GaussianFactor::shared_ptr& factor) {
  cout << "f(";
  for(Key key: factor->keys()) {
    cout << " " << DefaultKeyFormatter(key);
  }
  cout << " )" << endl;
}

/* ************************************************************************* */
void BatchFixedLagSmoother::PrintSymbolicGraph(
    const NonlinearFactorGraph& graph, const string& label) {
  cout << label << endl;
  for(const auto& factor: graph) {
    PrintSymbolicFactor(factor);
  }
}

/* ************************************************************************* */
void BatchFixedLagSmoother::PrintSymbolicGraph(const GaussianFactorGraph& graph,
    const string& label) {
  cout << label << endl;
  for(const auto& factor: graph) {
    PrintSymbolicFactor(factor);
  }
}

/* ************************************************************************* */
GaussianFactorGraph BatchFixedLagSmoother::CalculateMarginalFactors(
    const GaussianFactorGraph& graph, const KeyVector& keys,
    const GaussianFactorGraph::Eliminate& eliminateFunction) {
  if (keys.size() == 0) {
    // There are no keys to marginalize. Simply return the input factors
    return graph;
  } else {
    // .first is the eliminated Bayes tree, while .second is the remaining factor graph
    return *graph.eliminatePartialMultifrontal(keys, eliminateFunction).second;
  }
}

/* ************************************************************************* */
NonlinearFactorGraph BatchFixedLagSmoother::CalculateMarginalFactors(
    const NonlinearFactorGraph& graph, const Values& theta, const KeyVector& keys,
    const GaussianFactorGraph::Eliminate& eliminateFunction) {
  if (keys.size() == 0) {
    // There are no keys to marginalize. Simply return the input factors
    return graph;
  } else {
    // Create the linear factor graph
    const auto linearFactorGraph = graph.linearize(theta);

    const auto marginalLinearFactors =
        CalculateMarginalFactors(*linearFactorGraph, keys, eliminateFunction);

    // Wrap in nonlinear container factors
    return LinearContainerFactor::ConvertLinearGraph(marginalLinearFactors, theta);
  }
}

/* ************************************************************************* */
} /// namespace gtsam
