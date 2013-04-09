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
      && variableIndex_.equals(smoother->variableIndex_)
      && separatorValues_.equals(smoother->separatorValues_);
}

/* ************************************************************************* */
ConcurrentBatchSmoother::Result ConcurrentBatchSmoother::update(const NonlinearFactorGraph& newFactors, const Values& newTheta) {

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
  reorder();
  gttoc(reorder);

  // Optimize the factors using a modified version of L-M
  gttic(optimize);
  if(factors_.size() > 0) {
    result = optimize();
  }
  gttoc(optimize);


  gttic(presync);
  gttoc(presync);

  gttoc(update);











//  gttic(update);
//
//  // Create result structure
//  Result result;
//
//  gttic(augment_system);
//  // Add the new factors to the graph
//  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, newFactors) {
//    insertFactor(factor);
//  }
//  // Add the new variables to theta
//  theta_.insert(newTheta);
//  gttoc(augment_system);
//
//  // Optimize the graph, updating theta
//  gttic(optimize);
//  if(graph_.size() > 0){
//    optimize();
//
//    // TODO: fill in the results structure properly
//    result.iterations = 0;
//    result.nonlinearVariables = theta_.size() - separatorValues_.size();
//    result.linearVariables = separatorValues_.size();
//    result.error = 0;
//  }
//  gttoc(optimize);
//
//  // Move all of the Pre-Sync code to the end of the update. This allows the smoother to perform these
//  // calculations while the filter is still running
//  gttic(presync);
//  // Calculate and store the information passed up to the separator. This requires:
//  //   1) Calculate an ordering that forces the separator variables to be in the root
//  //   2) Eliminate all of the variables except the root. This produces one or more
//  //      linear, marginal factors on the separator variables.
//  //   3) Convert the marginal factors into nonlinear ones using the 'LinearContainerFactor'
//
//  if(separatorValues_.size() > 0) {
//    // Force variables associated with root keys to keep the same linearization point
//    gttic(enforce_consistency);
//    Values linpoint;
//    linpoint.insert(theta_);
//    linpoint.insert(separatorValues_);
//
////linpoint.print("ConcurrentBatchSmoother::presync()  LinPoint:\n");
//
//    gttoc(enforce_consistency);
//
//    // Calculate a root-constrained ordering
//    gttic(compute_ordering);
//    std::map<Key, int> constraints;
//    BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, separatorValues_) {
//      constraints[key_value.key] = 1;
//    }
//    Ordering ordering = *graph_.orderingCOLAMDConstrained(linpoint, constraints);
//    gttoc(compute_ordering);
//
//    // Create a Bayes Tree using iSAM2 cliques
//    gttic(create_bayes_tree);
//    JunctionTree<GaussianFactorGraph, ISAM2Clique> jt(*graph_.linearize(linpoint, ordering));
//    ISAM2Clique::shared_ptr root = jt.eliminate(parameters_.getEliminationFunction());
//    BayesTree<GaussianConditional, ISAM2Clique> bayesTree;
//    bayesTree.insert(root);
//    gttoc(create_bayes_tree);
//
////ordering.print("ConcurrentBatchSmoother::presync()  Ordering:\n");
//std::cout << "ConcurrentBatchSmoother::presync()  Root Keys: "; BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, separatorValues_) { std::cout << DefaultKeyFormatter(key_value.key) << " "; } std::cout << std::endl;
//std::cout << "ConcurrentBatchSmoother::presync()  Bayes Tree:" << std::endl;
////SymbolicPrintTree(root, ordering, "  ");
//
//    // Extract the marginal factors from the smoother
//    // For any non-filter factor that involves a root variable,
//    // calculate its marginal on the root variables using the
//    // current linearization point.
//
//    // Find all of the smoother branches as the children of root cliques that are not also root cliques
//    gttic(find_smoother_branches);
//    std::set<ISAM2Clique::shared_ptr> rootCliques;
//    std::set<ISAM2Clique::shared_ptr> smootherBranches;
//    BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, separatorValues_) {
//      const ISAM2Clique::shared_ptr& clique = bayesTree.nodes().at(ordering.at(key_value.key));
//      if(clique) {
//        rootCliques.insert(clique);
//        smootherBranches.insert(clique->children().begin(), clique->children().end());
//      }
//    }
//    BOOST_FOREACH(const ISAM2Clique::shared_ptr& rootClique, rootCliques) {
//      smootherBranches.erase(rootClique);
//    }
//    gttoc(find_smoother_branches);
//
//    // Extract the cached factors on the root cliques from the smoother branches
//    gttic(extract_cached_factors);
//    GaussianFactorGraph cachedFactors;
//    BOOST_FOREACH(const ISAM2Clique::shared_ptr& clique, smootherBranches) {
//      cachedFactors.push_back(clique->cachedFactor());
//    }
//    gttoc(extract_cached_factors);
//
//std::cout << "ConcurrentBatchSmoother::presync()  Cached Factors Before:" << std::endl;
//BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, cachedFactors) {
//  std::cout << "  g( ";
//  BOOST_FOREACH(Index index, factor->keys()) {
//    std::cout << DefaultKeyFormatter(ordering.key(index)) << " ";
//  }
//  std::cout << ")" << std::endl;
//}
//
//    // Marginalize out any additional (non-root) variables
//    gttic(marginalize_extra_variables);
//    // The rootKeys have been ordered last, so their linear indices will be { linpoint.size()-rootKeys.size() :: linpoint.size()-1 }
//    Index minRootIndex = linpoint.size() - separatorValues_.size();
//    // Calculate the set of keys to be marginalized
//    FastSet<Index> cachedIndices = cachedFactors.keys();
//    std::vector<Index> marginalizeIndices;
//    std::remove_copy_if(cachedIndices.begin(), cachedIndices.end(), std::back_inserter(marginalizeIndices), boost::lambda::_1 >= minRootIndex);
//
//std::cout << "ConcurrentBatchSmoother::presync()  Marginalize Keys: ";
//BOOST_FOREACH(Index index, marginalizeIndices) { std::cout << DefaultKeyFormatter(ordering.key(index)) << " "; }
//std::cout << std::endl;
//
//    // If non-root-keys are present, marginalize them out
//    if(marginalizeIndices.size() > 0) {
//      // Eliminate the extra variables, stored the remaining factors back into the 'cachedFactors' graph
//      GaussianConditional::shared_ptr conditional;
//      boost::tie(conditional, cachedFactors) = cachedFactors.eliminate(marginalizeIndices, parameters_.getEliminationFunction());
//    }
//    gttoc(marginalize_extra_variables);
//
//std::cout << "ConcurrentBatchSmoother::presync()  Cached Factors After:" << std::endl;
//BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, cachedFactors) {
//  std::cout << "  g( ";
//  BOOST_FOREACH(Index index, factor->keys()) {
//    std::cout << DefaultKeyFormatter(ordering.key(index)) << " ";
//  }
//  std::cout << ")" << std::endl;
//}
//
//    // Convert factors into 'Linearized' nonlinear factors
//    gttic(store_cached_factors);
//    smootherSummarization_.resize(0);
//    BOOST_FOREACH(const GaussianFactor::shared_ptr& gaussianFactor, cachedFactors) {
//      LinearContainerFactor::shared_ptr factor(new LinearContainerFactor(gaussianFactor, ordering, linpoint));
//      smootherSummarization_.push_back(factor);
//    }
//    gttoc(store_cached_factors);
//
//std::cout << "ConcurrentBatchSmoother::presync()  Smoother Summarization:" << std::endl;
//BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, smootherSummarization_) {
//  std::cout << "  f( ";
//  BOOST_FOREACH(Key key, factor->keys()) {
//    std::cout << DefaultKeyFormatter(key) << " ";
//  }
//  std::cout << ")" << std::endl;
//}
//
//  }
//  gttoc(presync);
//
//  gttoc(update);

  return result;
}

/* ************************************************************************* */
void ConcurrentBatchSmoother::presync() {

  gttic(presync);

  gttoc(presync);
}

/* ************************************************************************* */
void ConcurrentBatchSmoother::getSummarizedFactors(NonlinearFactorGraph& summarizedFactors) {

  gttic(get_summarized_factors);

  // Copy the previous calculated smoother summarization factors into the output
  summarizedFactors.push_back(smootherSummarization_);

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

  // Augment the Variable Index
  variableIndex_.augment(*factors.symbolic(ordering_));

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

  // Remove references to this factor from the VariableIndex
  variableIndex_.remove(slots, factors);

  gttoc(remove_factors);
}

/* ************************************************************************* */
void ConcurrentBatchSmoother::reorder() {

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
        // Add prior factors at the current solution
        gttic(damp);
        GaussianFactorGraph dampedFactorGraph(linearFactorGraph);
        dampedFactorGraph.reserve(linearFactorGraph.size() + delta_.size());
        {
          // for each of the variables, add a prior at the current solution
          for(size_t j=0; j<delta_.size(); ++j) {
            Matrix A = lambda * eye(delta_[j].size());
            Vector b = lambda * delta_[j];
            SharedDiagonal model = noiseModel::Unit::Create(delta_[j].size());
            GaussianFactor::shared_ptr prior(new JacobianFactor(j, A, b, model));
            dampedFactorGraph.push_back(prior);
          }
        }
        gttoc(damp);
        result.lambdas++;

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




///* ************************************************************************* */
//std::set<size_t> ConcurrentBatchSmoother::findFactorsWithAny(const std::set<Key>& keys) const {
//  // Find the set of factor slots for each specified key
//  std::set<size_t> factorSlots;
//  BOOST_FOREACH(Key key, keys) {
//    FactorIndex::const_iterator iter = factorIndex_.find(key);
//    if(iter != factorIndex_.end()) {
//      factorSlots.insert(iter->second.begin(), iter->second.end());
//    }
//  }
//
//  return factorSlots;
//}
//
///* ************************************************************************* */
//std::set<size_t> ConcurrentBatchSmoother::findFactorsWithOnly(const std::set<Key>& keys) const {
//  // Find the set of factor slots with any of the provided keys
//  std::set<size_t> factorSlots = findFactorsWithAny(keys);
//  // Test each factor for non-specified keys
//  std::set<size_t>::iterator slot = factorSlots.begin();
//  while(slot != factorSlots.end()) {
//    const NonlinearFactor::shared_ptr& factor = graph_.at(*slot);
//    std::set<Key> factorKeys(factor->begin(), factor->end()); // ensure the keys are sorted
//    if(!std::includes(keys.begin(), keys.end(), factorKeys.begin(), factorKeys.end())) {
//      factorSlots.erase(slot++);
//    } else {
//      ++slot;
//    }
//  }
//
//  return factorSlots;
//}
//
///* ************************************************************************* */
//NonlinearFactor::shared_ptr ConcurrentBatchSmoother::marginalizeKeysFromFactor(const NonlinearFactor::shared_ptr& factor, const std::set<Key>& keysToKeep, const Values& theta) const {
//
//factor->print("Factor Before:\n");
//
//  // Sort the keys for this factor
//  std::set<Key> factorKeys;
//  BOOST_FOREACH(Key key, *factor) {
//    factorKeys.insert(key);
//  }
//
//  // Calculate the set of keys to marginalize
//  std::set<Key> marginalizeKeys;
//  std::set_difference(factorKeys.begin(), factorKeys.end(), keysToKeep.begin(), keysToKeep.end(), std::inserter(marginalizeKeys, marginalizeKeys.end()));
//  std::set<Key> remainingKeys;
//  std::set_intersection(factorKeys.begin(), factorKeys.end(), keysToKeep.begin(), keysToKeep.end(), std::inserter(remainingKeys, remainingKeys.end()));
//
//  //
//  if(marginalizeKeys.size() == 0) {
//    // No keys need to be marginalized out. Simply return the original factor.
//    return factor;
//  } else if(marginalizeKeys.size() == factor->size()) {
//    // All keys need to be marginalized out. Return an empty factor
//    return NonlinearFactor::shared_ptr();
//  } else {
//    // (0) Create an ordering with the remaining keys last
//    Ordering ordering;
//    BOOST_FOREACH(Key key, marginalizeKeys) {
//      ordering.push_back(key);
//    }
//    BOOST_FOREACH(Key key, remainingKeys) {
//      ordering.push_back(key);
//    }
//ordering.print("Ordering:\n");
//
//    //  (1) construct a linear factor graph
//    GaussianFactorGraph graph;
//    graph.push_back( factor->linearize(theta, ordering) );
//graph.at(0)->print("Linear Factor Before:\n");
//
//    //  (2) solve for the marginal factor
//    // Perform partial elimination, resulting in a conditional probability ( P(MarginalizedVariable | RemainingVariables)
//    // and factors on the remaining variables ( f(RemainingVariables) ). These are the factors we need to add to iSAM2
//    std::vector<Index> variables;
//    BOOST_FOREACH(Key key, marginalizeKeys) {
//      variables.push_back(ordering.at(key));
//    }
////    std::pair<GaussianFactorGraph::sharedConditional, GaussianFactorGraph> result = graph.eliminate(variables);
//    GaussianFactorGraph::EliminationResult result = EliminateQR(graph, marginalizeKeys.size());
//result.first->print("Resulting Conditional:\n");
//result.second->print("Resulting Linear Factor:\n");
////    graph = result.second;
//    graph.replace(0, result.second);
//
//    //  (3) convert the marginal factors into Linearized Factors
//    NonlinearFactor::shared_ptr marginalFactor;
//    assert(graph.size() <= 1);
//    if(graph.size() > 0) {
//graph.at(0)->print("Linear Factor After:\n");
//      marginalFactor.reset(new LinearContainerFactor(graph.at(0), ordering, theta));
//    }
//marginalFactor->print("Factor After:\n");
//    return marginalFactor;
//  }
//}



///* ************************************************************************* */
//void ConcurrentBatchSmoother::PrintSingleClique(const ISAM2Clique::shared_ptr& clique, const Ordering& ordering, const std::string& indent, const KeyFormatter& keyFormatter) {
//  std::cout << indent << "P( ";
//  BOOST_FOREACH(Index index, clique->conditional()->frontals()){
//    std::cout << keyFormatter(ordering.key(index)) << " ";
//  }
//  if(clique->conditional()->nrParents() > 0){
//    std::cout << "| ";
//    BOOST_FOREACH(Index index, clique->conditional()->parents()){
//      std::cout << keyFormatter(ordering.key(index)) << " ";
//    }
//  }
//  std::cout << ")" << std::endl;
//}
//
///* ************************************************************************* */
//void ConcurrentBatchSmoother::PrintRecursiveClique(const ISAM2Clique::shared_ptr& clique, const Ordering& ordering, const std::string& indent, const KeyFormatter& keyFormatter) {
//
//  // Print this node
//  PrintSingleClique(clique, ordering, indent, keyFormatter);
//
//  // Print Children
//  BOOST_FOREACH(const ISAM2Clique::shared_ptr& child, clique->children()) {
//    PrintRecursiveClique(child, ordering, indent+"  ", keyFormatter);
//  }
//}
//
///* ************************************************************************* */
//void ConcurrentBatchSmoother::PrintBayesTree(const ISAM2& bayesTree, const Ordering& ordering, const std::string& indent, const KeyFormatter& keyFormatter) {
//
//  std::cout << indent << "Bayes Tree:" << std::endl;
//  if (bayesTree.root().use_count() == 0) {
//    std::cout << indent << "  {EMPTY}" << std::endl;
//  } else {
//    std::cout << indent << "  clique size == " << bayesTree.size() << ", node size == " << bayesTree.nodes().size() <<  std::endl;
//    PrintRecursiveClique(bayesTree.root(), ordering, indent+"  ", keyFormatter);
//  }
//}

/* ************************************************************************* */

}/// namespace gtsam
