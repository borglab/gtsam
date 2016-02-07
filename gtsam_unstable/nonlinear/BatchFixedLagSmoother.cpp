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

#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianFactor.h>
//#include <gtsam/inference/inference.h>
#include <gtsam/base/debug.h>

namespace gtsam {

/* ************************************************************************* */
void BatchFixedLagSmoother::print(const std::string& s,
    const KeyFormatter& keyFormatter) const {
  FixedLagSmoother::print(s, keyFormatter);
  // TODO: What else to print?
}

/* ************************************************************************* */
bool BatchFixedLagSmoother::equals(const FixedLagSmoother& rhs,
    double tol) const {
  const BatchFixedLagSmoother* e =
      dynamic_cast<const BatchFixedLagSmoother*>(&rhs);
  return e != NULL && FixedLagSmoother::equals(*e, tol)
      && factors_.equals(e->factors_, tol) && theta_.equals(e->theta_, tol);
}

/* ************************************************************************* */
Matrix BatchFixedLagSmoother::marginalCovariance(Key key) const {
  throw std::runtime_error(
      "BatchFixedLagSmoother::marginalCovariance not implemented");
}

/* ************************************************************************* */
FixedLagSmoother::Result BatchFixedLagSmoother::update(
    const NonlinearFactorGraph& newFactors, const Values& newTheta,
    const KeyTimestampMap& timestamps) {

  const bool debug = ISDEBUG("BatchFixedLagSmoother update");
  if (debug) {
    std::cout << "BatchFixedLagSmoother::update() START" << std::endl;
  }

  // Update all of the internal variables with the new information
  gttic(augment_system);
  // Add the new variables to theta
  theta_.insert(newTheta);
  // Add new variables to the end of the ordering
  BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, newTheta) {
    ordering_.push_back(key_value.key);
  }
  // Augment Delta
  delta_.insert(newTheta.zeroVectors());

  // Add the new factors to the graph, updating the variable index
  insertFactors(newFactors);
  gttoc(augment_system);

  // Update the Timestamps associated with the factor keys
  updateKeyTimestampMap(timestamps);

  // Get current timestamp
  double current_timestamp = getCurrentTimestamp();
  if (debug)
    std::cout << "Current Timestamp: " << current_timestamp << std::endl;

  // Find the set of variables to be marginalized out
  std::set<Key> marginalizableKeys = findKeysBefore(
      current_timestamp - smootherLag_);
  if (debug) {
    std::cout << "Marginalizable Keys: ";
    BOOST_FOREACH(Key key, marginalizableKeys) {
      std::cout << DefaultKeyFormatter(key) << " ";
    }
    std::cout << std::endl;
  }

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
  gttoc(optimize);

  // Marginalize out old variables.
  gttic(marginalize);
  if (marginalizableKeys.size() > 0) {
    marginalize(marginalizableKeys);
  }
  gttoc(marginalize);

  if (debug) {
    std::cout << "BatchFixedLagSmoother::update() FINISH" << std::endl;
  }

  return result;
}

/* ************************************************************************* */
void BatchFixedLagSmoother::insertFactors(
    const NonlinearFactorGraph& newFactors) {
  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, newFactors) {
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
    BOOST_FOREACH(Key key, *factor) {
      factorIndex_[key].insert(index);
    }
  }
}

/* ************************************************************************* */
void BatchFixedLagSmoother::removeFactors(
    const std::set<size_t>& deleteFactors) {
  BOOST_FOREACH(size_t slot, deleteFactors) {
    if (factors_.at(slot)) {
      // Remove references to this factor from the FactorIndex
      BOOST_FOREACH(Key key, *(factors_.at(slot))) {
        factorIndex_[key].erase(slot);
      }
      // Remove the factor from the factor graph
      factors_.remove(slot);
      // Add the factor's old slot to the list of available slots
      availableSlots_.push(slot);
    } else {
      // TODO: Throw an error??
      std::cout << "Attempting to remove a factor from slot " << slot
          << ", but it is already NULL." << std::endl;
    }
  }
}

/* ************************************************************************* */
void BatchFixedLagSmoother::eraseKeys(const std::set<Key>& keys) {

  BOOST_FOREACH(Key key, keys) {
    // Erase the key from the values
    theta_.erase(key);

    // Erase the key from the factor index
    factorIndex_.erase(key);

    // Erase the key from the set of linearized keys
    if (linearKeys_.exists(key)) {
      linearKeys_.erase(key);
    }
  }

  eraseKeyTimestampMap(keys);

  // Remove marginalized keys from the ordering and delta
  BOOST_FOREACH(Key key, keys) {
    ordering_.erase(std::find(ordering_.begin(), ordering_.end(), key));
    delta_.erase(key);
  }
}

/* ************************************************************************* */
void BatchFixedLagSmoother::reorder(const std::set<Key>& marginalizeKeys) {

  const bool debug = ISDEBUG("BatchFixedLagSmoother reorder");

  if (debug) {
    std::cout << "BatchFixedLagSmoother::reorder() START" << std::endl;
  }

  if (debug) {
    std::cout << "Marginalizable Keys: ";
    BOOST_FOREACH(Key key, marginalizeKeys) {
      std::cout << DefaultKeyFormatter(key) << " ";
    }
    std::cout << std::endl;
  }

  // COLAMD groups will be used to place marginalize keys in Group 0, and everything else in Group 1
  ordering_ = Ordering::ColamdConstrainedFirst(factors_,
      std::vector<Key>(marginalizeKeys.begin(), marginalizeKeys.end()));

  if (debug) {
    ordering_.print("New Ordering: ");
  }

  if (debug) {
    std::cout << "BatchFixedLagSmoother::reorder() FINISH" << std::endl;
  }
}

/* ************************************************************************* */
FixedLagSmoother::Result BatchFixedLagSmoother::optimize() {

  const bool debug = ISDEBUG("BatchFixedLagSmoother optimize");

  if (debug) {
    std::cout << "BatchFixedLagSmoother::optimize() START" << std::endl;
  }

  // Create output result structure
  Result result;
  result.nonlinearVariables = theta_.size() - linearKeys_.size();
  result.linearVariables = linearKeys_.size();

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
    if (debug) {
      std::cout << "BatchFixedLagSmoother::optimize  Exiting, as error = "
          << result.error << " < " << errorTol << std::endl;
    }
    return result;
  }

  if (debug) {
    std::cout << "BatchFixedLagSmoother::optimize  linearValues: "
        << linearKeys_.size() << std::endl;
    std::cout << "BatchFixedLagSmoother::optimize  Initial error: "
        << result.error << std::endl;
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

        if (debug) {
          std::cout << "BatchFixedLagSmoother::optimize  trying lambda = "
              << lambda << std::endl;
        }

        // Add prior factors at the current solution
        gttic(damp);
        GaussianFactorGraph dampedFactorGraph(linearFactorGraph);
        dampedFactorGraph.reserve(linearFactorGraph.size() + delta_.size());
        {
          // for each of the variables, add a prior at the current solution
          double sigma = 1.0 / std::sqrt(lambda);
          BOOST_FOREACH(const VectorValues::KeyValuePair& key_value, delta_) {
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

        if (debug) {
          std::cout << "BatchFixedLagSmoother::optimize  linear delta norm = "
              << newDelta.norm() << std::endl;
          std::cout << "BatchFixedLagSmoother::optimize  next error = " << error
              << std::endl;
        }

        if (error < result.error) {
          // Keep this change
          // Update the error value
          result.error = error;
          // Update the linearization point
          theta_ = evalpoint;
          // Reset the deltas to zeros
          delta_.setZero();
          // Put the linearization points and deltas back for specific variables
          if (enforceConsistency_ && (linearKeys_.size() > 0)) {
            theta_.update(linearKeys_);
            BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, linearKeys_) {
              delta_.at(key_value.key) = newDelta.at(key_value.key);
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
            std::cout
                << "Warning:  Levenberg-Marquardt giving up because cannot decrease error with maximum lambda"
                << std::endl;
            break;
          } else {
            // Increase lambda and continue searching
            lambda *= lambdaFactor;
          }
        }
      } // end while
    }
    gttoc(optimizer_iteration);

    if (debug) {
      std::cout << "BatchFixedLagSmoother::optimize  using lambda = " << lambda
          << std::endl;
    }

    result.iterations++;
  } while (result.iterations < maxIterations
      && !checkConvergence(relativeErrorTol, absoluteErrorTol, errorTol,
          previousError, result.error, NonlinearOptimizerParams::SILENT));

  if (debug) {
    std::cout << "BatchFixedLagSmoother::optimize  newError: " << result.error
        << std::endl;
  }

  if (debug) {
    std::cout << "BatchFixedLagSmoother::optimize() FINISH" << std::endl;
  }

  return result;
}

/* ************************************************************************* */
void BatchFixedLagSmoother::marginalize(const std::set<Key>& marginalizeKeys) {
  // In order to marginalize out the selected variables, the factors involved in those variables
  // must be identified and removed. Also, the effect of those removed factors on the
  // remaining variables needs to be accounted for. This will be done with linear container factors
  // from the result of a partial elimination. This function removes the marginalized factors and
  // adds the linearized factors back in.

  const bool debug = ISDEBUG("BatchFixedLagSmoother marginalize");

  if (debug)
    std::cout << "BatchFixedLagSmoother::marginalize  Begin" << std::endl;

  if (debug) {
    std::cout << "BatchFixedLagSmoother::marginalize  Marginalize Keys: ";
    BOOST_FOREACH(Key key, marginalizeKeys) {
      std::cout << DefaultKeyFormatter(key) << " ";
    }
    std::cout << std::endl;
  }

  // Identify all of the factors involving any marginalized variable. These must be removed.
  std::set<size_t> removedFactorSlots;
  VariableIndex variableIndex(factors_);
  BOOST_FOREACH(Key key, marginalizeKeys) {
    const FastVector<size_t>& slots = variableIndex[key];
    removedFactorSlots.insert(slots.begin(), slots.end());
  }

  if (debug) {
    std::cout << "BatchFixedLagSmoother::marginalize  Removed Factor Slots: ";
    BOOST_FOREACH(size_t slot, removedFactorSlots) {
      std::cout << slot << " ";
    }
    std::cout << std::endl;
  }

  // Add the removed factors to a factor graph
  NonlinearFactorGraph removedFactors;
  BOOST_FOREACH(size_t slot, removedFactorSlots) {
    if (factors_.at(slot)) {
      removedFactors.push_back(factors_.at(slot));
    }
  }

  if (debug) {
    PrintSymbolicGraph(removedFactors,
        "BatchFixedLagSmoother::marginalize  Removed Factors: ");
  }

  // Calculate marginal factors on the remaining keys
  NonlinearFactorGraph marginalFactors = calculateMarginalFactors(
      removedFactors, theta_, marginalizeKeys,
      parameters_.getEliminationFunction());

  if (debug) {
    PrintSymbolicGraph(removedFactors,
        "BatchFixedLagSmoother::marginalize  Marginal Factors: ");
  }

  // Remove marginalized factors from the factor graph
  removeFactors(removedFactorSlots);

  // Remove marginalized keys from the system
  eraseKeys(marginalizeKeys);

  // Insert the new marginal factors
  insertFactors(marginalFactors);
}

/* ************************************************************************* */
void BatchFixedLagSmoother::PrintKeySet(const std::set<Key>& keys,
    const std::string& label) {
  std::cout << label;
  BOOST_FOREACH(gtsam::Key key, keys) {
    std::cout << " " << gtsam::DefaultKeyFormatter(key);
  }
  std::cout << std::endl;
}

/* ************************************************************************* */
void BatchFixedLagSmoother::PrintKeySet(const gtsam::KeySet& keys,
    const std::string& label) {
  std::cout << label;
  BOOST_FOREACH(gtsam::Key key, keys) {
    std::cout << " " << gtsam::DefaultKeyFormatter(key);
  }
  std::cout << std::endl;
}

/* ************************************************************************* */
void BatchFixedLagSmoother::PrintSymbolicFactor(
    const NonlinearFactor::shared_ptr& factor) {
  std::cout << "f(";
  if (factor) {
    BOOST_FOREACH(Key key, factor->keys()) {
      std::cout << " " << gtsam::DefaultKeyFormatter(key);
    }
  } else {
    std::cout << " NULL";
  }
  std::cout << " )" << std::endl;
}

/* ************************************************************************* */
void BatchFixedLagSmoother::PrintSymbolicFactor(
    const GaussianFactor::shared_ptr& factor) {
  std::cout << "f(";
  BOOST_FOREACH(Key key, factor->keys()) {
    std::cout << " " << gtsam::DefaultKeyFormatter(key);
  }
  std::cout << " )" << std::endl;
}

/* ************************************************************************* */
void BatchFixedLagSmoother::PrintSymbolicGraph(
    const NonlinearFactorGraph& graph, const std::string& label) {
  std::cout << label << std::endl;
  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, graph) {
    PrintSymbolicFactor(factor);
  }
}

/* ************************************************************************* */
void BatchFixedLagSmoother::PrintSymbolicGraph(const GaussianFactorGraph& graph,
    const std::string& label) {
  std::cout << label << std::endl;
  BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, graph) {
    PrintSymbolicFactor(factor);
  }
}

/* ************************************************************************* */
NonlinearFactorGraph BatchFixedLagSmoother::calculateMarginalFactors(
    const NonlinearFactorGraph& graph, const Values& theta,
    const std::set<Key>& marginalizeKeys,
    const GaussianFactorGraph::Eliminate& eliminateFunction) {

  const bool debug = ISDEBUG("BatchFixedLagSmoother calculateMarginalFactors");

  if (debug)
    std::cout << "BatchFixedLagSmoother::calculateMarginalFactors START"
        << std::endl;

  if (debug)
    PrintKeySet(marginalizeKeys,
        "BatchFixedLagSmoother::calculateMarginalFactors  Marginalize Keys: ");

  // Get the set of all keys involved in the factor graph
  KeySet allKeys(graph.keys());
  if (debug)
    PrintKeySet(allKeys,
        "BatchFixedLagSmoother::calculateMarginalFactors  All Keys: ");

  // Calculate the set of RemainingKeys = AllKeys \Intersect marginalizeKeys
  KeySet remainingKeys;
  std::set_difference(allKeys.begin(), allKeys.end(), marginalizeKeys.begin(),
      marginalizeKeys.end(), std::inserter(remainingKeys, remainingKeys.end()));
  if (debug)
    PrintKeySet(remainingKeys,
        "BatchFixedLagSmoother::calculateMarginalFactors  Remaining Keys: ");

  if (marginalizeKeys.size() == 0) {
    // There are no keys to marginalize. Simply return the input factors
    if (debug)
      std::cout << "BatchFixedLagSmoother::calculateMarginalFactors FINISH"
          << std::endl;
    return graph;
  } else {

    // Create the linear factor graph
    GaussianFactorGraph linearFactorGraph = *graph.linearize(theta);
    // .first is the eliminated Bayes tree, while .second is the remaining factor graph
    GaussianFactorGraph marginalLinearFactors =
        *linearFactorGraph.eliminatePartialMultifrontal(
            std::vector<Key>(marginalizeKeys.begin(), marginalizeKeys.end())).second;

    // Wrap in nonlinear container factors
    NonlinearFactorGraph marginalFactors;
    marginalFactors.reserve(marginalLinearFactors.size());
    BOOST_FOREACH(const GaussianFactor::shared_ptr& gaussianFactor, marginalLinearFactors) {
      marginalFactors += boost::make_shared<LinearContainerFactor>(
          gaussianFactor, theta);
      if (debug) {
        std::cout
            << "BatchFixedLagSmoother::calculateMarginalFactors  Marginal Factor: ";
        PrintSymbolicFactor(marginalFactors.back());
      }
    }

    if (debug)
      PrintSymbolicGraph(marginalFactors,
          "BatchFixedLagSmoother::calculateMarginalFactors  All Marginal Factors: ");

    if (debug)
      std::cout << "BatchFixedLagSmoother::calculateMarginalFactors FINISH"
          << std::endl;

    return marginalFactors;
  }
}

/* ************************************************************************* */
} /// namespace gtsam
