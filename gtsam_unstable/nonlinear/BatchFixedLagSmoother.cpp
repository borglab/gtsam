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
#include <gtsam/inference/inference.h>
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

  // Add the new factors
  insertFactors(newFactors);

  // Add the new variables
  theta_.insert(newTheta);

  // Add new variables to the end of the ordering
  BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, newTheta) {
    ordering_.push_back(key_value.key);
  }

  // Augment Delta
  std::vector<size_t> dims;
  dims.reserve(newTheta.size());
  BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, newTheta) {
    dims.push_back(key_value.value.dim());
  }
  delta_.append(dims);
  for(size_t i = delta_.size() - dims.size(); i < delta_.size(); ++i) {
    delta_[i].setZero();
  }

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
  reorder(marginalizableKeys);

  // Optimize
  Result result;
  if(theta_.size() > 0) {
    result = optimize();
  }

  // Marginalize out old variables.
  if(marginalizableKeys.size() > 0) {
    marginalize(marginalizableKeys);
  }

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

  // Calculate a variable index
  VariableIndexOrdered variableIndex(*factors_.symbolic(ordering_), ordering_.size());

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
}

/* ************************************************************************* */
FixedLagSmoother::Result BatchFixedLagSmoother::optimize() {
  // Create output result structure
  Result result;
  result.nonlinearVariables = theta_.size() - linearKeys_.size();
  result.linearVariables = linearKeys_.size();

  // Set optimization parameters
  double lambda = parameters_.lambdaInitial;
  double lambdaFactor = parameters_.lambdaFactor;
  double lambdaUpperBound = parameters_.lambdaUpperBound;
  double lambdaLowerBound = 0.5 / parameters_.lambdaUpperBound;
  size_t maxIterations = parameters_.maxIterations;
  double relativeErrorTol = parameters_.relativeErrorTol;
  double absoluteErrorTol = parameters_.absoluteErrorTol;
  double errorTol = parameters_.errorTol;

  // Create a Values that holds the current evaluation point
  Values evalpoint = theta_.retract(delta_, ordering_);
  result.error = factors_.error(evalpoint);

  // Use a custom optimization loop so the linearization points can be controlled
  double previousError;
  VectorValuesOrdered newDelta;
  do {
    previousError = result.error;

    // Do next iteration
    gttic(optimizer_iteration);
    {
      // Linearize graph around the linearization point
      GaussianFactorGraphOrdered linearFactorGraph = *factors_.linearize(theta_, ordering_);

      // Keep increasing lambda until we make make progress
      while(true) {
        // Add prior factors at the current solution
        gttic(damp);
        GaussianFactorGraphOrdered dampedFactorGraph(linearFactorGraph);
        dampedFactorGraph.reserve(linearFactorGraph.size() + delta_.size());
        {
          // for each of the variables, add a prior at the current solution
          double sigma = 1.0 / std::sqrt(lambda);
          for(size_t j=0; j<delta_.size(); ++j) {
            size_t dim = delta_[j].size();
            Matrix A = eye(dim);
            Vector b = delta_[j];
            SharedDiagonal model = noiseModel::Isotropic::Sigma(dim, sigma);
            GaussianFactorOrdered::shared_ptr prior(new JacobianFactorOrdered(j, A, b, model));
            dampedFactorGraph.push_back(prior);
          }
        }
        gttoc(damp);
        result.intermediateSteps++;

        gttic(solve);
        // Solve Damped Gaussian Factor Graph
        newDelta = GaussianJunctionTreeOrdered(dampedFactorGraph).optimize(parameters_.getEliminationFunction());
        // update the evalpoint with the new delta
        evalpoint = theta_.retract(newDelta, ordering_);
        gttoc(solve);

        // Evaluate the new error
        gttic(compute_error);
        double error = factors_.error(evalpoint);
        gttoc(compute_error);

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
          // Increase lambda and continue searching
          lambda *= lambdaFactor;
          if(lambda > lambdaUpperBound) {
            // The maximum lambda has been used. Print a warning and end the search.
            std::cout << "Warning:  Levenberg-Marquardt giving up because cannot decrease error with maximum lambda" << std::endl;
            break;
          }
        }
      } // end while
    }
    gttoc(optimizer_iteration);

    result.iterations++;
  } while(result.iterations < maxIterations &&
      !checkConvergence(relativeErrorTol, absoluteErrorTol, errorTol, previousError, result.error, NonlinearOptimizerParams::SILENT));

  return result;
}

/* ************************************************************************* */
void BatchFixedLagSmoother::marginalize(const std::set<Key>& marginalizeKeys) {
  // In order to marginalize out the selected variables, the factors involved in those variables
  // must be identified and removed. Also, the effect of those removed factors on the
  // remaining variables needs to be accounted for. This will be done with linear container factors
  // from the result of a partial elimination. This function removes the marginalized factors and
  // adds the linearized factors back in.

  // Calculate marginal factors on the remaining variables (after marginalizing 'marginalizeKeys')
  // Note: It is assumed the ordering already has these keys first
  // Create the linear factor graph
  GaussianFactorGraphOrdered linearFactorGraph = *factors_.linearize(theta_, ordering_);

  // Create a variable index
  VariableIndexOrdered variableIndex(linearFactorGraph, ordering_.size());

  // Use the variable Index to mark the factors that will be marginalized
  std::set<size_t> removedFactorSlots;
  BOOST_FOREACH(Key key, marginalizeKeys) {
    const FastList<size_t>& slots = variableIndex[ordering_.at(key)];
    removedFactorSlots.insert(slots.begin(), slots.end());
  }

  // Construct an elimination tree to perform sparse elimination
  std::vector<EliminationForest::shared_ptr> forest( EliminationForest::Create(linearFactorGraph, variableIndex) );

  // This is a tree. Only the top-most nodes/indices need to be eliminated; all of the children will be eliminated automatically
  // Find the subset of nodes/keys that must be eliminated
  std::set<Index> indicesToEliminate;
  BOOST_FOREACH(Key key, marginalizeKeys) {
    indicesToEliminate.insert(ordering_.at(key));
  }
  BOOST_FOREACH(Key key, marginalizeKeys) {
    EliminationForest::removeChildrenIndices(indicesToEliminate, forest.at(ordering_.at(key)));
  }

  // Eliminate each top-most key, returning a Gaussian Factor on some of the remaining variables
  // Convert the marginal factors into Linear Container Factors
  // Add the marginal factor variables to the separator
  NonlinearFactorGraph marginalFactors;
  BOOST_FOREACH(Index index, indicesToEliminate) {
    GaussianFactorOrdered::shared_ptr gaussianFactor = forest.at(index)->eliminateRecursive(parameters_.getEliminationFunction());
    if(gaussianFactor->size() > 0) {
      LinearContainerFactor::shared_ptr marginalFactor(new LinearContainerFactor(gaussianFactor, ordering_, theta_));
      marginalFactors.push_back(marginalFactor);
      // Add the keys associated with the marginal factor to the separator values
      BOOST_FOREACH(Key key, *marginalFactor) {
        if(!linearKeys_.exists(key)) {
          linearKeys_.insert(key, theta_.at(key));
        }
      }
    }
  }
  insertFactors(marginalFactors);

  // Remove the marginalized variables and factors from the filter
  // Remove marginalized factors from the factor graph
  removeFactors(removedFactorSlots);

  // Remove marginalized keys from the system
  eraseKeys(marginalizeKeys);
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
void BatchFixedLagSmoother::PrintSymbolicFactor(const GaussianFactorOrdered::shared_ptr& factor, const OrderingOrdered& ordering) {
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
void BatchFixedLagSmoother::PrintSymbolicGraph(const GaussianFactorGraphOrdered& graph, const OrderingOrdered& ordering, const std::string& label) {
  std::cout << label << std::endl;
  BOOST_FOREACH(const GaussianFactorOrdered::shared_ptr& factor, graph) {
    PrintSymbolicFactor(factor, ordering);
  }
}

/* ************************************************************************* */
std::vector<Index> BatchFixedLagSmoother::EliminationForest::ComputeParents(const VariableIndexOrdered& structure) {
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
std::vector<BatchFixedLagSmoother::EliminationForest::shared_ptr> BatchFixedLagSmoother::EliminationForest::Create(const GaussianFactorGraphOrdered& factorGraph, const VariableIndexOrdered& structure) {
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
  BOOST_FOREACH(const GaussianFactorOrdered::shared_ptr& factor, factorGraph) {
    if(factor && factor->size() > 0) {
      Index j = *std::min_element(factor->begin(), factor->end());
      if(j < structure.size())
        trees[j]->add(factor);
    }
  }

  return trees;
}

/* ************************************************************************* */
GaussianFactorOrdered::shared_ptr BatchFixedLagSmoother::EliminationForest::eliminateRecursive(GaussianFactorGraphOrdered::Eliminate function) {

  // Create the list of factors to be eliminated, initially empty, and reserve space
  GaussianFactorGraphOrdered factors;
  factors.reserve(this->factors_.size() + this->subTrees_.size());

  // Add all factors associated with the current node
  factors.push_back(this->factors_.begin(), this->factors_.end());

  // for all subtrees, eliminate into Bayes net and a separator factor, added to [factors]
  BOOST_FOREACH(const shared_ptr& child, subTrees_)
    factors.push_back(child->eliminateRecursive(function));

  // Combine all factors (from this node and from subtrees) into a joint factor
  GaussianFactorGraphOrdered::EliminationResult eliminated(function(factors, 1));

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
} /// namespace gtsam
