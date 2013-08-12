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
void BatchFixedLagSmoother::print(const std::string& s, const KeyFormatter& keyFormatter) const {
  FixedLagSmoother::print(s, keyFormatter);
  // TODO: What else to print?
}

/* ************************************************************************* */
bool BatchFixedLagSmoother::equals(const FixedLagSmoother& rhs, double tol) const {
  const BatchFixedLagSmoother* e =  dynamic_cast<const BatchFixedLagSmoother*> (&rhs);
  return e != NULL
      && FixedLagSmoother::equals(*e, tol)
      && factors_.equals(e->factors_, tol)
      && theta_.equals(e->theta_, tol);
}

/* ************************************************************************* */
FixedLagSmoother::Result BatchFixedLagSmoother::update(const NonlinearFactorGraph& newFactors, const Values& newTheta, const KeyTimestampMap& timestamps) {

  const bool debug = ISDEBUG("BatchFixedLagSmoother update");
  if(debug) {
    std::cout << "BatchFixedLagSmoother::update() START" << std::endl;
  }

  // Update all of the internal variables with the new information
  gttic(augment_system);
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
  gttoc(augment_system);

  // Update the Timestamps associated with the factor keys
  updateKeyTimestampMap(timestamps);

  // Get current timestamp
  double current_timestamp = getCurrentTimestamp();
  if(debug) std::cout << "Current Timestamp: " << current_timestamp << std::endl;

  // Find the set of variables to be marginalized out
  std::set<Key> marginalizableKeys = findKeysBefore(current_timestamp - smootherLag_);
  if(debug) {
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
  if(factors_.size() > 0) {
    result = optimize();
  }
  gttoc(optimize);

  // Marginalize out old variables.
  gttic(marginalize);
  if(marginalizableKeys.size() > 0) {
    marginalize(marginalizableKeys);
  }
  gttoc(marginalize);

  if(debug) {
    std::cout << "BatchFixedLagSmoother::update() FINISH" << std::endl;
  }

  return result;
}

/* ************************************************************************* */
void BatchFixedLagSmoother::insertFactors(const NonlinearFactorGraph& newFactors) {
  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, newFactors) {
    Index index;
    // Insert the factor into an existing hole in the factor graph, if possible
    if(availableSlots_.size() > 0) {
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
void BatchFixedLagSmoother::removeFactors(const std::set<size_t>& deleteFactors) {
  BOOST_FOREACH(size_t slot, deleteFactors) {
    if(factors_.at(slot)) {
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
      std::cout << "Attempting to remove a factor from slot " << slot << ", but it is already NULL." << std::endl;
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
    if(linearKeys_.exists(key)) {
      linearKeys_.erase(key);
    }
  }

  eraseKeyTimestampMap(keys);

  // Permute the ordering such that the removed keys are at the end.
  // This is a prerequisite for removing them from several structures
  std::vector<Index> toBack;
  BOOST_FOREACH(Key key, keys) {
    toBack.push_back(ordering_.at(key));
  }
  Permutation forwardPermutation = Permutation::PushToBack(toBack, ordering_.size());
  ordering_.permuteInPlace(forwardPermutation);
  delta_.permuteInPlace(forwardPermutation);

  // Remove marginalized keys from the ordering and delta
  for(size_t i = 0; i < keys.size(); ++i) {
    ordering_.pop_back();
    delta_.pop_back();
  }

}

/* ************************************************************************* */
void BatchFixedLagSmoother::reorder(const std::set<Key>& marginalizeKeys) {

  const bool debug = ISDEBUG("BatchFixedLagSmoother reorder");

  if(debug) {
    std::cout << "BatchFixedLagSmoother::reorder() START" << std::endl;
  }

  if(debug) {
    std::cout << "Marginalizable Keys: ";
    BOOST_FOREACH(Key key, marginalizeKeys) {
      std::cout << DefaultKeyFormatter(key) << " ";
    }
    std::cout << std::endl;
  }

  // Calculate a variable index
  VariableIndex variableIndex(*factors_.symbolic(ordering_), ordering_.size());

  // COLAMD groups will be used to place marginalize keys in Group 0, and everything else in Group 1
  int group0 = 0;
  int group1 = marginalizeKeys.size() > 0 ? 1 : 0;

  // Initialize all variables to group1
  std::vector<int> cmember(variableIndex.size(), group1);

  // Set all of the marginalizeKeys to Group0
  if(marginalizeKeys.size() > 0) {
    BOOST_FOREACH(Key key, marginalizeKeys) {
      cmember[ordering_.at(key)] = group0;
    }
  }

  // Generate the permutation
  Permutation forwardPermutation = *inference::PermutationCOLAMD_(variableIndex, cmember);

  // Permute the ordering, variable index, and deltas
  ordering_.permuteInPlace(forwardPermutation);
  delta_.permuteInPlace(forwardPermutation);

  if(debug) {
    ordering_.print("New Ordering: ");
  }

  if(debug) {
    std::cout << "BatchFixedLagSmoother::reorder() FINISH" << std::endl;
  }
}

/* ************************************************************************* */
FixedLagSmoother::Result BatchFixedLagSmoother::optimize() {

  const bool debug = ISDEBUG("BatchFixedLagSmoother optimize");

  if(debug) {
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
  Values evalpoint = theta_.retract(delta_, ordering_);
  result.error = factors_.error(evalpoint);

  // check if we're already close enough
  if(result.error <= errorTol) {
    if(debug) { std::cout << "BatchFixedLagSmoother::optimize  Exiting, as error = " << result.error << " < " << errorTol << std::endl; }
    return result;
  }

  if(debug) {
    std::cout << "BatchFixedLagSmoother::optimize  linearValues: " << linearKeys_.size() << std::endl;
    std::cout << "BatchFixedLagSmoother::optimize  Initial error: " << result.error << std::endl;
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

        if(debug) { std::cout << "BatchFixedLagSmoother::optimize  trying lambda = " << lambda << std::endl; }

        // Add prior factors at the current solution
        gttic(damp);
        GaussianFactorGraph dampedFactorGraph(linearFactorGraph);
        dampedFactorGraph.reserve(linearFactorGraph.size() + delta_.size());
        {
          // for each of the variables, add a prior at the current solution
          double sigma = 1.0 / std::sqrt(lambda);
          for(size_t j=0; j<delta_.size(); ++j) {
            size_t dim = delta_[j].size();
            Matrix A = eye(dim);
            Vector b = delta_[j];
            SharedDiagonal model = noiseModel::Isotropic::Sigma(dim, sigma);
            GaussianFactor::shared_ptr prior(new JacobianFactor(j, A, b, model));
            dampedFactorGraph.push_back(prior);
          }
        }
        gttoc(damp);
        result.intermediateSteps++;

        gttic(solve);
        // Solve Damped Gaussian Factor Graph
        newDelta = GaussianJunctionTree(dampedFactorGraph).optimize(parameters_.getEliminationFunction());
        // update the evalpoint with the new delta
        evalpoint = theta_.retract(newDelta, ordering_);
        gttoc(solve);

        // Evaluate the new error
        gttic(compute_error);
        double error = factors_.error(evalpoint);
        gttoc(compute_error);

        if(debug) {
          std::cout << "BatchFixedLagSmoother::optimize  linear delta norm = " << newDelta.norm() << std::endl;
          std::cout << "BatchFixedLagSmoother::optimize  next error = " << error << std::endl;
        }

        if(error < result.error) {
          // Keep this change
          // Update the error value
          result.error = error;
          // Update the linearization point
          theta_ = evalpoint;
          // Reset the deltas to zeros
          delta_.setZero();
          // Put the linearization points and deltas back for specific variables
          if(enforceConsistency_ && (linearKeys_.size() > 0)) {
            theta_.update(linearKeys_);
            BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, linearKeys_) {
              Index index = ordering_.at(key_value.key);
              delta_.at(index) = newDelta.at(index);
            }
          }
          // Decrease lambda for next time
          lambda /= lambdaFactor;
          if(lambda < lambdaLowerBound) {
            lambda = lambdaLowerBound;
          }
          // End this lambda search iteration
          break;
        } else {
          // Reject this change
          if(lambda >= lambdaUpperBound) {
            // The maximum lambda has been used. Print a warning and end the search.
            std::cout << "Warning:  Levenberg-Marquardt giving up because cannot decrease error with maximum lambda" << std::endl;
            break;
          } else {
            // Increase lambda and continue searching
            lambda *= lambdaFactor;
          }
        }
      } // end while
    }
    gttoc(optimizer_iteration);

    if(debug) { std::cout << "BatchFixedLagSmoother::optimize  using lambda = " << lambda << std::endl; }

    result.iterations++;
  } while(result.iterations < maxIterations &&
      !checkConvergence(relativeErrorTol, absoluteErrorTol, errorTol, previousError, result.error, NonlinearOptimizerParams::SILENT));

  if(debug) { std::cout << "BatchFixedLagSmoother::optimize  newError: " << result.error << std::endl; }

  if(debug) {
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

  if(debug) std::cout << "BatchFixedLagSmoother::marginalize  Begin" << std::endl;

  if(debug) {
    std::cout << "BatchFixedLagSmoother::marginalize  Marginalize Keys: ";
    BOOST_FOREACH(Key key, marginalizeKeys) {
      std::cout << DefaultKeyFormatter(key) << " ";
    }
    std::cout << std::endl;
  }

  // Identify all of the factors involving any marginalized variable. These must be removed.
  std::set<size_t> removedFactorSlots;
  VariableIndex variableIndex(*factors_.symbolic(ordering_), theta_.size());
  BOOST_FOREACH(Key key, marginalizeKeys) {
    const FastList<size_t>& slots = variableIndex[ordering_.at(key)];
    BOOST_FOREACH(size_t slot, slots) {
      removedFactorSlots.insert(slot);
    }
  }

  if(debug) {
    std::cout << "BatchFixedLagSmoother::marginalize  Removed Factor Slots: ";
    BOOST_FOREACH(size_t slot, removedFactorSlots) {
      std::cout << slot << " ";
    }
    std::cout << std::endl;
  }

  // Add the removed factors to a factor graph
  NonlinearFactorGraph removedFactors;
  BOOST_FOREACH(size_t slot, removedFactorSlots) {
    if(factors_.at(slot)) {
      removedFactors.push_back(factors_.at(slot));
    }
  }

  if(debug) {
    PrintSymbolicGraph(removedFactors, "BatchFixedLagSmoother::marginalize  Removed Factors: ");
  }

  // Calculate marginal factors on the remaining keys
  NonlinearFactorGraph marginalFactors = calculateMarginalFactors(removedFactors, theta_, marginalizeKeys, parameters_.getEliminationFunction());

  if(debug) {
    PrintSymbolicGraph(removedFactors, "BatchFixedLagSmoother::marginalize  Marginal Factors: ");
  }

  // Remove marginalized factors from the factor graph
  removeFactors(removedFactorSlots);

  // Remove marginalized keys from the system
  eraseKeys(marginalizeKeys);

  // Insert the new marginal factors
  insertFactors(marginalFactors);
}

/* ************************************************************************* */
void BatchFixedLagSmoother::PrintKeySet(const std::set<Key>& keys, const std::string& label) {
  std::cout << label;
  BOOST_FOREACH(gtsam::Key key, keys) {
    std::cout << " " << gtsam::DefaultKeyFormatter(key);
  }
  std::cout << std::endl;
}

/* ************************************************************************* */
void BatchFixedLagSmoother::PrintKeySet(const gtsam::FastSet<Key>& keys, const std::string& label) {
  std::cout << label;
  BOOST_FOREACH(gtsam::Key key, keys) {
    std::cout << " " << gtsam::DefaultKeyFormatter(key);
  }
  std::cout << std::endl;
}

/* ************************************************************************* */
void BatchFixedLagSmoother::PrintSymbolicFactor(const NonlinearFactor::shared_ptr& factor) {
  std::cout << "f(";
  if(factor) {
    BOOST_FOREACH(Key key, factor->keys()) {
      std::cout << " " << gtsam::DefaultKeyFormatter(key);
    }
  } else {
    std::cout << " NULL";
  }
  std::cout << " )" << std::endl;
}

/* ************************************************************************* */
void BatchFixedLagSmoother::PrintSymbolicFactor(const GaussianFactor::shared_ptr& factor, const Ordering& ordering) {
  std::cout << "f(";
  BOOST_FOREACH(Index index, factor->keys()) {
    std::cout << " " << index << "[" << gtsam::DefaultKeyFormatter(ordering.key(index)) << "]";
  }
  std::cout << " )" << std::endl;
}

/* ************************************************************************* */
void BatchFixedLagSmoother::PrintSymbolicGraph(const NonlinearFactorGraph& graph, const std::string& label) {
  std::cout << label << std::endl;
  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, graph) {
    PrintSymbolicFactor(factor);
  }
}

/* ************************************************************************* */
void BatchFixedLagSmoother::PrintSymbolicGraph(const GaussianFactorGraph& graph, const Ordering& ordering, const std::string& label) {
  std::cout << label << std::endl;
  BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, graph) {
    PrintSymbolicFactor(factor, ordering);
  }
}

/* ************************************************************************* */
std::vector<Index> BatchFixedLagSmoother::EliminationForest::ComputeParents(const VariableIndex& structure) {
  // Number of factors and variables
  const size_t m = structure.nFactors();
  const size_t n = structure.size();

  static const Index none = std::numeric_limits<Index>::max();

  // Allocate result parent vector and vector of last factor columns
  std::vector<Index> parents(n, none);
  std::vector<Index> prevCol(m, none);

  // for column j \in 1 to n do
  for (Index j = 0; j < n; j++) {
    // for row i \in Struct[A*j] do
    BOOST_FOREACH(const size_t i, structure[j]) {
      if (prevCol[i] != none) {
        Index k = prevCol[i];
        // find root r of the current tree that contains k
        Index r = k;
        while (parents[r] != none)
          r = parents[r];
        if (r != j) parents[r] = j;
      }
      prevCol[i] = j;
    }
  }

  return parents;
}

/* ************************************************************************* */
std::vector<BatchFixedLagSmoother::EliminationForest::shared_ptr> BatchFixedLagSmoother::EliminationForest::Create(const GaussianFactorGraph& factorGraph, const VariableIndex& structure) {
  // Compute the tree structure
  std::vector<Index> parents(ComputeParents(structure));

  // Number of variables
  const size_t n = structure.size();

  static const Index none = std::numeric_limits<Index>::max();

  // Create tree structure
  std::vector<shared_ptr> trees(n);
  for (Index k = 1; k <= n; k++) {
    Index j = n - k;  // Start at the last variable and loop down to 0
    trees[j].reset(new EliminationForest(j));  // Create a new node on this variable
    if (parents[j] != none)  // If this node has a parent, add it to the parent's children
      trees[parents[j]]->add(trees[j]);
  }

  // Hang factors in right places
  BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, factorGraph) {
    if(factor && factor->size() > 0) {
      Index j = *std::min_element(factor->begin(), factor->end());
      if(j < structure.size())
        trees[j]->add(factor);
    }
  }

  return trees;
}

/* ************************************************************************* */
GaussianFactor::shared_ptr BatchFixedLagSmoother::EliminationForest::eliminateRecursive(GaussianFactorGraph::Eliminate function) {

  // Create the list of factors to be eliminated, initially empty, and reserve space
  GaussianFactorGraph factors;
  factors.reserve(this->factors_.size() + this->subTrees_.size());

  // Add all factors associated with the current node
  factors.push_back(this->factors_.begin(), this->factors_.end());

  // for all subtrees, eliminate into Bayes net and a separator factor, added to [factors]
  BOOST_FOREACH(const shared_ptr& child, subTrees_)
    factors.push_back(child->eliminateRecursive(function));

  // Combine all factors (from this node and from subtrees) into a joint factor
  GaussianFactorGraph::EliminationResult eliminated(function(factors, 1));

  return eliminated.second;
}

/* ************************************************************************* */
void BatchFixedLagSmoother::EliminationForest::removeChildrenIndices(std::set<Index>& indices, const BatchFixedLagSmoother::EliminationForest::shared_ptr& tree) {
  BOOST_FOREACH(const EliminationForest::shared_ptr& child, tree->children()) {
    indices.erase(child->key());
    removeChildrenIndices(indices, child);
  }
}

/* ************************************************************************* */
NonlinearFactorGraph BatchFixedLagSmoother::calculateMarginalFactors(const NonlinearFactorGraph& graph, const Values& theta,
    const std::set<Key>& marginalizeKeys, const GaussianFactorGraph::Eliminate& eliminateFunction) {

  const bool debug = ISDEBUG("BatchFixedLagSmoother calculateMarginalFactors");

  if(debug) std::cout << "BatchFixedLagSmoother::calculateMarginalFactors START" << std::endl;

  if(debug) PrintKeySet(marginalizeKeys, "BatchFixedLagSmoother::calculateMarginalFactors  Marginalize Keys: ");

  // Get the set of all keys involved in the factor graph
  FastSet<Key> allKeys(graph.keys());
  if(debug) PrintKeySet(allKeys, "BatchFixedLagSmoother::calculateMarginalFactors  All Keys: ");

  // Calculate the set of RemainingKeys = AllKeys \Intersect marginalizeKeys
  FastSet<Key> remainingKeys;
  std::set_difference(allKeys.begin(), allKeys.end(), marginalizeKeys.begin(), marginalizeKeys.end(), std::inserter(remainingKeys, remainingKeys.end()));
  if(debug) PrintKeySet(remainingKeys, "BatchFixedLagSmoother::calculateMarginalFactors  Remaining Keys: ");

  if(marginalizeKeys.size() == 0) {
    // There are no keys to marginalize. Simply return the input factors
    if(debug) std::cout << "BatchFixedLagSmoother::calculateMarginalFactors FINISH" << std::endl;
    return graph;
  } else {
    // Create a subset of theta that only contains the required keys
    Values values;
    BOOST_FOREACH(Key key, allKeys) {
      values.insert(key, theta.at(key));
    }

    // Calculate the ordering: [Others Root]
    std::map<Key, int> constraints;
    BOOST_FOREACH(Key key, marginalizeKeys) {
      constraints[key] = 0;
    }
    BOOST_FOREACH(Key key, remainingKeys) {
      constraints[key] = 1;
    }
    Ordering ordering = *graph.orderingCOLAMDConstrained(values, constraints);

    // Create the linear factor graph
    GaussianFactorGraph linearFactorGraph = *graph.linearize(values, ordering);

    // Construct a variable index
    VariableIndex variableIndex(linearFactorGraph, ordering.size());

    // Construct an elimination tree to perform sparse elimination
    std::vector<EliminationForest::shared_ptr> forest( BatchFixedLagSmoother::EliminationForest::Create(linearFactorGraph, variableIndex) );

    // This is a forest. Only the top-most node/index of each tree needs to be eliminated; all of the children will be eliminated automatically
    // Find the subset of nodes/keys that must be eliminated
    std::set<Index> indicesToEliminate;
    BOOST_FOREACH(Key key, marginalizeKeys) {
      indicesToEliminate.insert(ordering.at(key));
    }
    BOOST_FOREACH(Key key, marginalizeKeys) {
      EliminationForest::removeChildrenIndices(indicesToEliminate, forest.at(ordering.at(key)));
    }

    if(debug) PrintKeySet(indicesToEliminate, "BatchFixedLagSmoother::calculateMarginalFactors  Indices To Eliminate: ");

    // Eliminate each top-most key, returning a Gaussian Factor on some of the remaining variables
    // Convert the marginal factors into Linear Container Factors
    NonlinearFactorGraph marginalFactors;
    BOOST_FOREACH(Index index, indicesToEliminate) {
      GaussianFactor::shared_ptr gaussianFactor = forest.at(index)->eliminateRecursive(eliminateFunction);
      if(gaussianFactor->size() > 0) {
        LinearContainerFactor::shared_ptr marginalFactor(new LinearContainerFactor(gaussianFactor, ordering, values));
        marginalFactors.push_back(marginalFactor);
        if(debug) {
          std::cout << "BatchFixedLagSmoother::calculateMarginalFactors  Marginal Factor: ";
          PrintSymbolicFactor(marginalFactor);
        }
      }
    }

    // Also add any remaining factors that were unaffected by marginalizing out the selected variables.
    // These are part of the marginal on the remaining variables as well.
    BOOST_FOREACH(Key key, remainingKeys) {
      BOOST_FOREACH(const GaussianFactor::shared_ptr& gaussianFactor, forest.at(ordering.at(key))->factors()) {
        LinearContainerFactor::shared_ptr marginalFactor(new LinearContainerFactor(gaussianFactor, ordering, values));
        marginalFactors.push_back(marginalFactor);
        if(debug) {
          std::cout << "BatchFixedLagSmoother::calculateMarginalFactors  Remaining Factor: ";
          PrintSymbolicFactor(marginalFactor);
        }
      }
    }

    if(debug) PrintSymbolicGraph(marginalFactors, "BatchFixedLagSmoother::calculateMarginalFactors  All Marginal Factors: ");

    if(debug) std::cout << "BatchFixedLagSmoother::calculateMarginalFactors FINISH" << std::endl;

    return marginalFactors;
  }
}

/* ************************************************************************* */
} /// namespace gtsam
