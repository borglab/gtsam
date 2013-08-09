/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ConcurrentBatchFilter.cpp
 * @brief   A Levenberg-Marquardt Batch Filter that implements the
 *          Concurrent Filtering and Smoothing interface.
 * @author  Stephen Williams
 */

#include <gtsam_unstable/nonlinear/ConcurrentBatchFilter.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/base/timing.h>
#include <gtsam/base/debug.h>

namespace gtsam {

/* ************************************************************************* */
void ConcurrentBatchFilter::print(const std::string& s, const KeyFormatter& keyFormatter) const {
  std::cout << s;
  std::cout << "  Factors:" << std::endl;
  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, factors_) {
    PrintNonlinearFactor(factor, "    ", keyFormatter);
  }
  theta_.print("Values:\n");
}

/* ************************************************************************* */
bool ConcurrentBatchFilter::equals(const ConcurrentFilter& rhs, double tol) const {
  const ConcurrentBatchFilter* filter = dynamic_cast<const ConcurrentBatchFilter*>(&rhs);
  return filter
      && factors_.equals(filter->factors_)
      && theta_.equals(filter->theta_)
      && ordering_.equals(filter->ordering_)
      && delta_.equals(filter->delta_)
      && separatorValues_.equals(filter->separatorValues_);
}

/* ************************************************************************* */
ConcurrentBatchFilter::Result ConcurrentBatchFilter::update(const NonlinearFactorGraph& newFactors, const Values& newTheta, const boost::optional<FastList<Key> >& keysToMove) {

  gttic(update);

  // Create the return result meta-data
  Result result;

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

  // Reorder the system to ensure efficient optimization (and marginalization) performance
  gttic(reorder);
  reorder(keysToMove);
  gttoc(reorder);

  // Optimize the factors using a modified version of L-M
  gttic(optimize);
  if(factors_.size() > 0) {
    result = optimize(factors_, theta_, ordering_, delta_, separatorValues_, parameters_);
  }
  gttoc(optimize);

  gttic(marginalize);
  if(keysToMove && keysToMove->size() > 0){
    marginalize(*keysToMove);
  }
  gttoc(marginalize);

  gttoc(update);

  return result;
}

/* ************************************************************************* */
void ConcurrentBatchFilter::presync() {

  gttic(presync);

  gttoc(presync);
}

/* ************************************************************************* */
void ConcurrentBatchFilter::synchronize(const NonlinearFactorGraph& summarizedFactors, const Values& separatorValues) {

  gttic(synchronize);

  // Remove the previous smoother summarization
  removeFactors(smootherSummarizationSlots_);

  // Create a factor graph containing the new smoother summarization, the factors to be sent to the smoother,
  // and all of the filter factors. This is the set of factors on the filter side since the smoother started
  // its previous update cycle.
  NonlinearFactorGraph graph;
  graph.push_back(factors_);
  graph.push_back(smootherFactors_);
  graph.push_back(summarizedFactors);
  Values values;
  values.insert(theta_);
  values.insert(smootherValues_);
  values.update(separatorValues); // ensure the smoother summarized factors are linearized around the values in the smoother

  if(factors_.size() > 0) {
    // Perform an optional optimization on the to-be-sent-to-the-smoother factors
    if(relin_) {
      // Create ordering and delta
      Ordering ordering = graph.orderingCOLAMD();
      VectorValues delta = values.zeroVectors();
      // Optimize this graph using a modified version of L-M
      optimize(graph, values, ordering, delta, separatorValues, parameters_);
      // Update filter theta and delta
      BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, theta_) {
        theta_.update(key_value.key, values.at(key_value.key));
        delta_.at(ordering_.at(key_value.key)) = delta.at(ordering.at(key_value.key));
      }
      // Update the fixed linearization points (since they just changed)
      BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, separatorValues_) {
        separatorValues_.update(key_value.key, values.at(key_value.key));
      }
    }

    // Create separate ordering constraints that force either the filter keys or the smoother keys to the front
    typedef std::map<Key, int> OrderingConstraints;
    OrderingConstraints filterConstraints;
    OrderingConstraints smootherConstraints;
    BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, theta_) { /// the filter keys
      filterConstraints[key_value.key] = 0;
      smootherConstraints[key_value.key] = 1;
    }
    BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, smootherValues_) { /// the smoother keys
      filterConstraints[key_value.key] = 1;
      smootherConstraints[key_value.key] = 0;
    }
    BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, separatorValues_) { /// the *new* separator keys
      filterConstraints[key_value.key] = 2;
      smootherConstraints[key_value.key] = 2;
    }

    // Generate separate orderings that place the filter keys or the smoother keys first
    // TODO: This is convenient, but it recalculates the variable index each time
    Ordering filterOrdering = graph.orderingCOLAMDConstrained(filterConstraints);
    Ordering smootherOrdering = graph.orderingCOLAMDConstrained(smootherConstraints);

    // Extract the set of filter keys and smoother keys
    std::set<Key> filterKeys;
    std::set<Key> separatorKeys;
    std::set<Key> smootherKeys;
    BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, theta_) {
      filterKeys.insert(key_value.key);
    }
    BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, separatorValues_) {
      separatorKeys.insert(key_value.key);
      filterKeys.erase(key_value.key);
    }
    BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, smootherValues_) {
      smootherKeys.insert(key_value.key);
    }

    // Calculate the marginal on the new separator from the filter factors. This is performed by marginalizing out
    // all of the filter variables that are not part of the new separator. This filter summarization will then be
    // sent to the smoother.
    filterSummarization_ = marginalize(graph, values, filterOrdering, filterKeys, parameters_.getEliminationFunction());
    // The filter summarization should also include any nonlinear factors that involve only the separator variables.
    // Otherwise the smoother will be missing this information
    BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, factors_) {
      if(factor) {
        NonlinearFactor::const_iterator key = factor->begin();
        while((key != factor->end()) && (std::binary_search(separatorKeys.begin(), separatorKeys.end(), *key))) {
          ++key;
        }
        if(key == factor->end()) {
          filterSummarization_.push_back(factor);
        }
      }
    }

    // Calculate the marginal on the new separator from the smoother factors. This is performed by marginalizing out
    // all of the smoother variables that are not part of the new separator. This smoother summarization will be
    // stored locally for use in the filter
    smootherSummarizationSlots_ = insertFactors( marginalize(graph, values, smootherOrdering, smootherKeys, parameters_.getEliminationFunction()) );
  }
  gttoc(synchronize);
}

/* ************************************************************************* */
void ConcurrentBatchFilter::getSummarizedFactors(NonlinearFactorGraph& summarizedFactors, Values& separatorValues) {

  gttic(get_summarized_factors);

  // Copy the previous calculated smoother summarization factors into the output
  summarizedFactors.push_back(filterSummarization_);
  separatorValues.insert(separatorValues_);

  gttoc(get_summarized_factors);
}

/* ************************************************************************* */
void ConcurrentBatchFilter::getSmootherFactors(NonlinearFactorGraph& smootherFactors, Values& smootherValues) {

  gttic(get_smoother_factors);

  // Copy the previous calculated smoother summarization factors into the output
  smootherFactors.push_back(smootherFactors_);
  smootherValues.insert(smootherValues_);

  gttoc(get_smoother_factors);
}

/* ************************************************************************* */
void ConcurrentBatchFilter::postsync() {

  gttic(postsync);

  // Clear out the factors and values that were just sent to the smoother
  smootherFactors_.resize(0);
  smootherValues_.clear();

  gttoc(postsync);
}

/* ************************************************************************* */
std::vector<size_t> ConcurrentBatchFilter::insertFactors(const NonlinearFactorGraph& factors) {

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
void ConcurrentBatchFilter::removeFactors(const std::vector<size_t>& slots) {

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
void ConcurrentBatchFilter::reorder(const boost::optional<FastList<Key> >& keysToMove) {

  // Calculate the variable index
  VariableIndex variableIndex(*factors_.symbolic(ordering_), ordering_.size());

  // COLAMD groups will be used to place marginalize keys in Group 0, and everything else in Group 1
  int group0 = 0;
  int group1 = (keysToMove && (keysToMove->size() > 0) ) ? 1 : 0;

  // Initialize all variables to group1
  std::vector<int> cmember(variableIndex.size(), group1);

  // Set all of the keysToMove to Group0
  if(keysToMove && keysToMove->size() > 0) {
    BOOST_FOREACH(Key key, *keysToMove) {
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
ConcurrentBatchFilter::Result ConcurrentBatchFilter::optimize(const NonlinearFactorGraph& factors, Values& theta, const Ordering& ordering,
     VectorValues& delta, const Values& linearValues, const LevenbergMarquardtParams& parameters) {

  // Create output result structure
  Result result;
  result.nonlinearVariables = theta.size() - linearValues.size();
  result.linearVariables = linearValues.size();

  // Set optimization parameters
  double lambda = parameters.lambdaInitial;
  double lambdaFactor = parameters.lambdaFactor;
  double lambdaUpperBound = parameters.lambdaUpperBound;
  double lambdaLowerBound = 0.5 / parameters.lambdaUpperBound;
  size_t maxIterations = parameters.maxIterations;
  double relativeErrorTol = parameters.relativeErrorTol;
  double absoluteErrorTol = parameters.absoluteErrorTol;
  double errorTol = parameters.errorTol;

  // Create a Values that holds the current evaluation point
  Values evalpoint = theta.retract(delta);
  result.error = factors.error(evalpoint);

  // Use a custom optimization loop so the linearization points can be controlled
  double previousError;
  VectorValues newDelta;
  do {
    previousError = result.error;

    // Do next iteration
    gttic(optimizer_iteration);
    {
      // Linearize graph around the linearization point
      GaussianFactorGraph linearFactorGraph = *factors.linearize(theta);

      // Keep increasing lambda until we make make progress
      while(true) {
        // Add prior factors at the current solution
        gttic(damp);
        GaussianFactorGraph dampedFactorGraph(linearFactorGraph);
        dampedFactorGraph.reserve(linearFactorGraph.size() + delta.size());
        {
          // for each of the variables, add a prior at the current solution
          for(size_t j=0; j<delta.size(); ++j) {
            Matrix A = lambda * eye(delta[j].size());
            Vector b = lambda * delta[j];
            SharedDiagonal model = noiseModel::Unit::Create(delta[j].size());
            GaussianFactor::shared_ptr prior(new JacobianFactor(j, A, b, model));
            dampedFactorGraph.push_back(prior);
          }
        }
        gttoc(damp);
        result.lambdas++;

        gttic(solve);
        // Solve Damped Gaussian Factor Graph
        newDelta = GaussianJunctionTree(dampedFactorGraph).optimize(parameters.getEliminationFunction());
        // update the evalpoint with the new delta
        evalpoint = theta.retract(newDelta);
        gttoc(solve);

        // Evaluate the new nonlinear error
        gttic(compute_error);
        double error = factors.error(evalpoint);
        gttoc(compute_error);

        if(error < result.error) {
          // Keep this change
          // Update the error value
          result.error = error;
          // Update the linearization point
          theta = evalpoint;
          // Reset the deltas to zeros
          delta.setZero();
          // Put the linearization points and deltas back for specific variables
          if(linearValues.size() > 0) {
            theta.update(linearValues);
            BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, linearValues) {
              Index index = ordering.at(key_value.key);
              delta.at(index) = newDelta.at(index);
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
void ConcurrentBatchFilter::marginalize(const FastList<Key>& keysToMove) {
  // In order to marginalize out the selected variables, the factors involved in those variables
  // must be identified and removed. Also, the effect of those removed factors on the
  // remaining variables needs to be accounted for. This will be done with linear container factors
  // from the result of a partial elimination. This function removes the marginalized factors and
  // adds the linearized factors back in.


  // Calculate marginal factors on the remaining variables (after marginalizing 'keyToMove')
  // Note: It is assumed the ordering already has these keys first

  // Create the linear factor graph
  GaussianFactorGraph linearFactorGraph = *factors_.linearize(theta_);

  // Calculate the variable index
  VariableIndex variableIndex(linearFactorGraph, ordering_.size());

  // Use the variable Index to mark the factors that will be marginalized
  std::set<size_t> removedFactorSlots;
  BOOST_FOREACH(Key key, keysToMove) {
    const FastList<size_t>& slots = variableIndex[ordering_.at(key)];
    removedFactorSlots.insert(slots.begin(), slots.end());
  }

  // Construct an elimination tree to perform sparse elimination
  std::vector<EliminationForest::shared_ptr> forest( EliminationForest::Create(linearFactorGraph, variableIndex) );

  // This is a tree. Only the top-most nodes/indices need to be eliminated; all of the children will be eliminated automatically
  // Find the subset of nodes/keys that must be eliminated
  std::set<Index> indicesToEliminate;
  BOOST_FOREACH(Key key, keysToMove) {
    indicesToEliminate.insert(ordering_.at(key));
  }
  BOOST_FOREACH(Key key, keysToMove) {
    EliminationForest::removeChildrenIndices(indicesToEliminate, forest.at(ordering_.at(key)));
  }

  // Eliminate each top-most key, returning a Gaussian Factor on some of the remaining variables
  // Convert the marginal factors into Linear Container Factors
  // Add the marginal factor variables to the separator
  NonlinearFactorGraph marginalFactors;
  BOOST_FOREACH(Index index, indicesToEliminate) {
    GaussianFactor::shared_ptr gaussianFactor = forest.at(index)->eliminateRecursive(parameters_.getEliminationFunction());
    if(gaussianFactor->size() > 0) {
      LinearContainerFactor::shared_ptr marginalFactor(new LinearContainerFactor(gaussianFactor, theta_));
      marginalFactors.push_back(marginalFactor);
      // Add the keys associated with the marginal factor to the separator values
      BOOST_FOREACH(Key key, *marginalFactor) {
        if(!separatorValues_.exists(key)) {
          separatorValues_.insert(key, theta_.at(key));
        }
      }
    }
  }
  std::vector<size_t> marginalSlots = insertFactors(marginalFactors);


  // Cache marginalized variables and factors for later transmission to the smoother
  {
    // Add the new marginal factors to the list of smootherSeparatorFactors. In essence, we have just moved the separator
    smootherSummarizationSlots_.insert(smootherSummarizationSlots_.end(), marginalSlots.begin(), marginalSlots.end());

    // Move the marginalized factors from the filter to the smoother (holding area)
    // Note: Be careful to only move nonlinear factors and not any marginals that may also need to be removed
    BOOST_FOREACH(size_t slot, removedFactorSlots) {
      std::vector<size_t>::iterator iter = std::find(smootherSummarizationSlots_.begin(), smootherSummarizationSlots_.end(), slot);
      if(iter == smootherSummarizationSlots_.end()) {
        // This is a real nonlinear factor. Add it to the smoother factor cache.
        smootherFactors_.push_back(factors_.at(slot));
      } else {
        // This is a marginal factor that was removed and replaced by a new marginal factor. Remove this slot from the separator factor list.
        smootherSummarizationSlots_.erase(iter);
      }
    }

    // Add the linearization point of the moved variables to the smoother cache
    BOOST_FOREACH(Key key, keysToMove) {
      smootherValues_.insert(key, theta_.at(key));
    }
  }

  // Remove the marginalized variables and factors from the filter
  {
    // Remove marginalized factors from the factor graph
    std::vector<size_t> slots(removedFactorSlots.begin(), removedFactorSlots.end());
    removeFactors(slots);

    // Remove marginalized keys from values (and separator)
    BOOST_FOREACH(Key key, keysToMove) {
      theta_.erase(key);
      if(separatorValues_.exists(key)) {
        separatorValues_.erase(key);
      }
    }

    // Permute the ordering such that the removed keys are at the end.
    // This is a prerequisite for removing them from several structures
    std::vector<Index> toBack;
    BOOST_FOREACH(Key key, keysToMove) {
      toBack.push_back(ordering_.at(key));
    }
    Permutation forwardPermutation = Permutation::PushToBack(toBack, ordering_.size());
    ordering_.permuteInPlace(forwardPermutation);
    delta_.permuteInPlace(forwardPermutation);

    // Remove marginalized keys from the ordering and delta
    for(size_t i = 0; i < keysToMove.size(); ++i) {
      ordering_.pop_back();
      delta_.pop_back();
    }
  }
}

/* ************************************************************************* */
NonlinearFactorGraph ConcurrentBatchFilter::marginalize(const NonlinearFactorGraph& graph, const Values& values,
    const Ordering& ordering, const std::set<Key>& marginalizeKeys, const GaussianFactorGraph::Eliminate& function) {

  // Calculate marginal factors on the remaining variables (after marginalizing 'marginalizeKeys')
  // Note: It is assumed the ordering already has these keys first

  // Create the linear factor graph
  GaussianFactorGraph linearFactorGraph = *graph.linearize(values);

  // Construct a variable index
  VariableIndex variableIndex(linearFactorGraph, ordering.size());

  // Construct an elimination tree to perform sparse elimination
  std::vector<EliminationForest::shared_ptr> forest( EliminationForest::Create(linearFactorGraph, variableIndex) );

  // This is a forest. Only the top-most node/index of each tree needs to be eliminated; all of the children will be eliminated automatically
  // Find the subset of nodes/keys that must be eliminated
  std::set<Index> indicesToEliminate;
  BOOST_FOREACH(Key key, marginalizeKeys) {
    indicesToEliminate.insert(ordering.at(key));
  }
  BOOST_FOREACH(Key key, marginalizeKeys) {
    EliminationForest::removeChildrenIndices(indicesToEliminate, forest.at(ordering.at(key)));
  }

  // Eliminate each top-most key, returning a Gaussian Factor on some of the remaining variables
  // Convert the marginal factors into Linear Container Factors
  // Add the marginal factor variables to the separator
  NonlinearFactorGraph marginalFactors;
  BOOST_FOREACH(Index index, indicesToEliminate) {
    GaussianFactor::shared_ptr gaussianFactor = forest.at(index)->eliminateRecursive(function);
    if(gaussianFactor->size() > 0) {
      LinearContainerFactor::shared_ptr marginalFactor(new LinearContainerFactor(gaussianFactor, values));
      marginalFactors.push_back(marginalFactor);
    }
  }

  return marginalFactors;
}

/* ************************************************************************* */
void ConcurrentBatchFilter::PrintNonlinearFactor(const NonlinearFactor::shared_ptr& factor,
    const std::string& indent, const KeyFormatter& keyFormatter) {
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
void ConcurrentBatchFilter::PrintLinearFactor(const GaussianFactor::shared_ptr& factor,
    const std::string& indent, const KeyFormatter& keyFormatter) {
  std::cout << indent;
  if(factor) {
    std::cout << "g( ";
    BOOST_FOREACH(Index key, *factor) {
      std::cout << keyFormatter(key) << " ";
    }
    std::cout << ")" << std::endl;
  } else {
    std::cout << "{ NULL }" << std::endl;
  }
}

/* ************************************************************************* */
std::vector<Index> ConcurrentBatchFilter::EliminationForest::ComputeParents(const VariableIndex& structure) {
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
std::vector<ConcurrentBatchFilter::EliminationForest::shared_ptr> ConcurrentBatchFilter::EliminationForest::Create(const GaussianFactorGraph& factorGraph, const VariableIndex& structure) {
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
GaussianFactor::shared_ptr ConcurrentBatchFilter::EliminationForest::eliminateRecursive(GaussianFactorGraph::Eliminate function) {

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
void ConcurrentBatchFilter::EliminationForest::removeChildrenIndices(std::set<Index>& indices, const ConcurrentBatchFilter::EliminationForest::shared_ptr& tree) {
  BOOST_FOREACH(const EliminationForest::shared_ptr& child, tree->children()) {
    indices.erase(child->key());
    removeChildrenIndices(indices, child);
  }
}

/* ************************************************************************* */

}/// namespace gtsam
