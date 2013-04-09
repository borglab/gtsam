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
      && variableIndex_.equals(filter->variableIndex_)
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
    result = optimize();
  }
  gttoc(optimize);

  gttic(marginalize);
  if(keysToMove && keysToMove->size() > 0){
//    // Generate a permutation that will put the factor graph into the proper order
//    std::set<Key> activeKeys;
//    std::set<Key> marginalizableKeys;
//    calculateKeySets(activeKeys, marginalizableKeys, registeredSensors_, ordering_, keyTimestampMap_, currentTimestamp, parameters_.smootherLag);
//    permuteOrdering(ordering_, factors_, marginalizableKeys, activeKeys);
//
//    // Marginalize out inactive states (and remove from ordering/values)
//    marginalize(factors_, theta_, ordering_, keyTimestampMap_, linearValues_, marginalizableKeys);
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
void ConcurrentBatchFilter::synchronize(const NonlinearFactorGraph& summarizedFactors) {

  gttic(synchronize);

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

  // Augment the Variable Index
  variableIndex_.augment(*factors.symbolic(ordering_));

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

  // Remove references to this factor from the VariableIndex
  variableIndex_.remove(slots, factors);

  gttoc(remove_factors);
}

/* ************************************************************************* */
void ConcurrentBatchFilter::reorder(const boost::optional<FastList<Key> >& keysToMove) {

  // COLAMD groups will be used to place marginalize keys in Group 0, active keys in Group 1, and separator keys in Group 2
  int group0 = 0;
  int group1 = group0 + (keysToMove ? 1 : 0);
  int group2 = group1 + 1;

  // Initialize all variables to group1
  std::vector<int> cmember(variableIndex_.size(), group1);

  // Set all of the keysToMove to Group0
  if(keysToMove && keysToMove->size() > 0) {
    BOOST_FOREACH(Key key, *keysToMove) {
      cmember[ordering_.at(key)] = group0;
    }
  }

  // Set all of the separator keys to Group2
  if(separatorValues_.size() > 0) {
    BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, separatorValues_) {
      cmember[ordering_.at(key_value.key)] = group2;
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
ConcurrentBatchFilter::Result ConcurrentBatchFilter::optimize() {
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
void ConcurrentBatchFilter::PrintLinearFactor(const GaussianFactor::shared_ptr& factor, const Ordering& ordering,
    const std::string& indent, const KeyFormatter& keyFormatter) {
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
//void ConcurrentBatchFilter::synchronize(const NonlinearFactorGraph& summarizedFactors) {
//
//  gttic(synchronize);
//
//  std::cout << "ConcurrentBatchFilter::synchronize(*) Begin" << std::endl;
//
//std::cout << "ConcurrentBatchFilter::synchronize(*)  Input Smoother Summarization:" << std::endl;
//BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, summarizedFactors) {
//  std::cout << "  f( ";
//  BOOST_FOREACH(Key key, factor->keys()) {
//    std::cout << DefaultKeyFormatter(key) << " ";
//  }
//  std::cout << ")" << std::endl;
//}
//
//// TODO: Temporarily remove smoother effects
//  // Remove the previous smoother summarization from the graph
//  BOOST_FOREACH(size_t slot, smootherSummarizationSlots_) {
//    removeFactor(slot);
//  }
//  smootherSummarizationSlots_.clear();
//
//std::cout << "Removed Previous Smoother Summarization:" << std::endl;
//for(size_t i = 0; i < graph_.size(); ++i) {
//  if(graph_[i]) {
//    std::cout << "  f" << i << "( ";
//    BOOST_FOREACH(Key key, graph_[i]->keys()) {
//      std::cout << DefaultKeyFormatter(key) << " ";
//    }
//    std::cout << ")" << std::endl;
//  }
//}
//
//// TODO: Temporarily remove smoother effects
//  // Insert the updated smoother summarized factors
//  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, summarizedFactors) {
//    smootherSummarizationSlots_.push_back(insertFactor(factor));
//  }
//
//std::cout << "Added New Smoother Summarization:" << std::endl;
//for(size_t i = 0; i < graph_.size(); ++i) {
//  if(graph_[i]) {
//    std::cout << "  f" << i << "( ";
//    BOOST_FOREACH(Key key, graph_[i]->keys()) {
//      std::cout << DefaultKeyFormatter(key) << " ";
//    }
//    std::cout << ")" << std::endl;
//  }
//}
//
//// TODO: Temporarily remove smoother effects
//  // Now that the smoother summarization has been updated, re-optimize the filter
//  update();
//
//
//
//
//
//
//  // Force variables associated with root keys to keep the same linearization point
//  // TODO: This may be too many variables. It may only require that the smoother separator keys be kept constant.
//  gttic(enforce_consistency);
//  Values linpoint = theta_;
//  if(separatorValues_.size() > 0) {
//    linpoint.update(separatorValues_);
//  }
//  gttoc(enforce_consistency);
//
//std::cout << "ConcurrentBatchFilter::synchronize(*)  Old Smoother Separator Keys:  ";
//BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, separatorValues_) {
//  std::cout << DefaultKeyFormatter(key_value.key) << " ";
//}
//std::cout << std::endl;
//
//  // Calculate an ordering that puts variables newer than LAG at the top
//  gttic(compute_ordering);
//  Ordering ordering = computeOrdering(graph_);
//  gttoc(compute_ordering);
//
//ordering.print("Hmf5Batch::synchronize(*)  Ordering:\n");
//
//  // Create a Bayes Tree using iSAM2 cliques
//  gttic(create_bayes_tree);
//  JunctionTree<GaussianFactorGraph, ISAM2Clique> jt(*graph_.linearize(linpoint, ordering));
//  ISAM2Clique::shared_ptr root = jt.eliminate(parameters_.getEliminationFunction());
//  BayesTree<GaussianConditional, ISAM2Clique> bayesTree;
//  bayesTree.insert(root);
//  gttoc(create_bayes_tree);
//
//std::cout << "ConcurrentBatchFilter::synchronize(*)  Create Bayes Tree:" << std::endl;
//SymbolicPrintTree(root, ordering, "  ");
//
//
//
//
//
//  // Remove smoother summarization from the graph. It will be added back at the end.
//  BOOST_FOREACH(size_t slot, smootherSummarizationSlots_) {
//    removeFactor(slot);
//  }
//  smootherSummarizationSlots_.clear();
//
//
//
//
//
//
//  // Find the Separator Cliques
//  // The root is defined as any clique with a separator key (+ the path to the root)
//  gttic(identify_separator_cliques);
//  std::set<Key> separatorKeys = findSeparatorKeys(graph_);
//  std::set<ISAM2Clique::shared_ptr> separatorCliques;
//  Values oldSeparatorValues = separatorValues_;
//  separatorValues_.clear();
//  BOOST_FOREACH(Key key, separatorKeys) {
//    Index index = ordering.at(key);
//    const ISAM2Clique::shared_ptr& clique = bayesTree.nodes().at(index);
//    if(clique) {
//      separatorCliques.insert(clique);
//    }
//  }
//  // + the path to the root
//  // TODO: Given the ordering, is this actually necessary?
//  BOOST_FOREACH(const ISAM2Clique::shared_ptr& clique, separatorCliques) {
//    ISAM2Clique::shared_ptr separatorClique = clique;
//    while(!separatorClique->isRoot()) {
//      separatorClique = separatorClique->parent();
//      separatorCliques.insert(separatorClique);
//    }
//  }
//  BOOST_FOREACH(const ISAM2Clique::shared_ptr& clique, separatorCliques) {
//    BOOST_FOREACH(Index index, clique->conditional()->frontals()) {
//      separatorKeys.insert(ordering.key(index));
//    }
//  }
//  BOOST_FOREACH(Key key, separatorKeys) {
//    separatorValues_.insert(key, linpoint.at(key));
//  }
//  gttoc(identify_separator_cliques);
//
//std::cout << "ConcurrentBatchFilter::synchronize(*)  New Separator Cliques:" << std::endl;
//BOOST_FOREACH(const ISAM2Clique::shared_ptr& clique, separatorCliques) {
//  std::cout << "  P( ";
//  BOOST_FOREACH(Index index, clique->conditional()->frontals()) {
//    std::cout << DefaultKeyFormatter(ordering.key(index)) << " ";
//  }
//  if(clique->conditional()->nrParents() > 0) { std::cout << "| "; }
//  BOOST_FOREACH(Index index, clique->conditional()->parents()) {
//    std::cout << DefaultKeyFormatter(ordering.key(index)) << " ";
//  }
//  std::cout << ")" << std::endl;
//}
//
//std::cout << "ConcurrentBatchFilter::synchronize(*)  New Separator Keys:  ";
//BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, separatorValues_) {
//std::cout << DefaultKeyFormatter(key_value.key) << " ";
//}
//std::cout << std::endl;
//
//  // Identify the filter and smoother branches
//  // First find all of the children of the separator cliques (that are not separator cliques themselves),
//  // Then check if the child clique contains a filter key. If so, it is a filter branch, if not, it is a smoother branch
//  gttic(identify_smoother_and_filter_branches);
//  std::set<ISAM2Clique::shared_ptr> branches;
//  // Add all of the children to the children set
//  BOOST_FOREACH(const ISAM2Clique::shared_ptr& clique, separatorCliques) {
//    branches.insert(clique->children().begin(), clique->children().end());
//  }
//  // Remove any child that is also a separator
//  BOOST_FOREACH(const ISAM2Clique::shared_ptr& clique, separatorCliques) {
//    branches.erase(clique);
//  }
//  // Find the set of keys inside the lag
//  std::set<Key> insideKeys = findKeysBefore(getCurrentTimestamp() - lag_);
//  // Classify each child branch as a filter or a smoother branch
//  std::set<ISAM2Clique::shared_ptr> filterBranches;
//  std::set<ISAM2Clique::shared_ptr> smootherBranches;
//  BOOST_FOREACH(const ISAM2Clique::shared_ptr& branch, branches) {
//    std::set<Key> frontalKeys;
//    BOOST_FOREACH(Index index, branch->conditional()->frontals()) {
//      frontalKeys.insert(ordering.key(index));
//    }
//    if(std::find_first_of(frontalKeys.begin(), frontalKeys.end(), insideKeys.begin(), insideKeys.end()) == frontalKeys.end()) {
//      // No InsideKey exists in this branch. It must be a smoother branch.
//      smootherBranches.insert(branch);
//    } else {
//      // An InsideKey was found. This is a filter branch.
//      filterBranches.insert(branch);
//    }
//  }
//  gttoc(identify_smoother_and_filter_branches);
//
//std::cout << "ConcurrentBatchFilter::synchronize(*)  Smoother Branches:" << std::endl;
//BOOST_FOREACH(const ISAM2Clique::shared_ptr& clique, smootherBranches) {
//std::cout << "  P( ";
//BOOST_FOREACH(Index index, clique->conditional()->frontals()) {
//  std::cout << DefaultKeyFormatter(ordering.key(index)) << " ";
//}
//if(clique->conditional()->nrParents() > 0) { std::cout << "| "; }
//BOOST_FOREACH(Index index, clique->conditional()->parents()) {
//  std::cout << DefaultKeyFormatter(ordering.key(index)) << " ";
//}
//std::cout << ")" << std::endl;
//}
//
//std::cout << "ConcurrentBatchFilter::synchronize(*)  Filter Branches:" << std::endl;
//BOOST_FOREACH(const ISAM2Clique::shared_ptr& clique, filterBranches) {
//std::cout << "  P( ";
//BOOST_FOREACH(Index index, clique->conditional()->frontals()) {
//  std::cout << DefaultKeyFormatter(ordering.key(index)) << " ";
//}
//if(clique->conditional()->nrParents() > 0) { std::cout << "| "; }
//BOOST_FOREACH(Index index, clique->conditional()->parents()) {
//  std::cout << DefaultKeyFormatter(ordering.key(index)) << " ";
//}
//std::cout << ")" << std::endl;
//}
//
//  // Extract cached factors from the filter branches and store for future transmission to the smoother
//  gttic(extract_filter_summarization);
//  filterSummarization_.resize(0);
//  BOOST_FOREACH(const ISAM2Clique::shared_ptr& clique, filterBranches) {
//    LinearizedGaussianFactor::shared_ptr factor;
//    if(const JacobianFactor::shared_ptr rhs = boost::dynamic_pointer_cast<JacobianFactor>(clique->cachedFactor()))
//      factor = LinearizedJacobianFactor::shared_ptr(new LinearizedJacobianFactor(rhs, ordering, linpoint));
//    else if(const HessianFactor::shared_ptr rhs = boost::dynamic_pointer_cast<HessianFactor>(clique->cachedFactor()))
//      factor = LinearizedHessianFactor::shared_ptr(new LinearizedHessianFactor(rhs, ordering, linpoint));
//    else
//      throw std::invalid_argument("In ConcurrentBatchFilter::process(...), cached factor is neither a JacobianFactor nor a HessianFactor");
//    filterSummarization_.push_back(factor);
//  }
//  gttoc(extract_filter_summarization);
//
//  // Find all factors that contain only root clique variables
//  // This information is not contained in either the smoother
//  // summarization or the filter summarization. This must be sent
//  // to the smoother as well. Including it as part of the
//  // root/filter summarization
//  gttic(extract_root_summarization);
//  std::set<size_t> separatorFactorSlots = findFactorsWithOnly(separatorKeys);
//  BOOST_FOREACH(size_t slot, separatorFactorSlots) {
//    filterSummarization_.push_back(graph_.at(slot));
//  }
//  gttoc(extract_root_summarization);
//
//std::cout << "ConcurrentBatchFilter::synchronize(*)  Filter Summarization:" << std::endl;
//BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, filterSummarization_) {
//  if(boost::dynamic_pointer_cast<LinearizedGaussianFactor>(factor))
//    std::cout << "  L( ";
//  else
//    std::cout << "  f( ";
//  BOOST_FOREACH(Key key, factor->keys()) {
//    std::cout << DefaultKeyFormatter(key) << " ";
//  }
//  std::cout << ")" << std::endl;
//}
//
//  // Find all of the frontal keys in the smoother branches and grab their current linearization point
//  gttic(find_smoother_values);
//  std::set<Key> smootherKeys;
//  std::queue<ISAM2Clique::shared_ptr> smootherCliques;
//  BOOST_FOREACH(const ISAM2Clique::shared_ptr& clique, smootherBranches) {
//    smootherCliques.push(clique);
//  }
//  while(smootherCliques.size() > 0) {
//    // Extract the frontal keys from the next clique
//    BOOST_FOREACH(Index index, smootherCliques.front()->conditional()->frontals()) {
//      smootherKeys.insert(ordering.key(index));
//    }
//    // Add any children to the queue
//    BOOST_FOREACH(const ISAM2Clique::shared_ptr& child, smootherCliques.front()->children()) {
//      smootherCliques.push(child);
//    }
//    // Remove this clique from the queue
//    smootherCliques.pop();
//  }
//  // Store the current linearization point for future transmission to the smoother (skip any variables in the previous root)
//  smootherValues_.clear();
//  BOOST_FOREACH(Key key, smootherKeys) {
//    smootherValues_.insert(key, linpoint.at(key));
//  }
//  gttoc(find_smoother_values);
//
//std::cout << "Hmf5Batch::synchronize(*)  Smoother Keys:  ";
//BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, smootherValues_) {
//std::cout << DefaultKeyFormatter(key_value.key) << " ";
//}
//std::cout << std::endl;
//
//  // Find all of the factors that contain at least one smoother key
//  // These nonlinear factors will be sent to the smoother
//  // TODO: Are the summarized factors part of the graph currently?
//  gttic(find_smoother_factors);
//  std::set<size_t> smootherFactorSlots = findFactorsWithAny(smootherKeys);
//  // Convert the set of slots into a set of factors
//  smootherFactors_.resize(0);
//  BOOST_FOREACH(size_t slot, smootherFactorSlots) {
//    smootherFactors_.push_back(graph_.at(slot));
//  }
//  gttoc(find_smoother_factors);
//
//std::cout << "Hmf5Batch::synchronize(*)  Smoother Factors:" << std::endl;
//BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, smootherFactors_) {
//std::cout << "  f( ";
//BOOST_FOREACH(Key key, factor->keys()) {
//  std::cout << DefaultKeyFormatter(key) << " ";
//}
//std::cout << ")" << std::endl;
//}
//
//  // Calculate the new smoother summarization.
//  // This is potentially a combination of the previous summarization and cached factors from new smoother branches
//  gttic(extract_smoother_summarization);
//  NonlinearFactorGraph smootherSummarization;
//  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, summarizedFactors) {
//    if(smootherBranches.size() == 0) {
//      factor->print("Summarized Factor Before Marginalization:\n");
//      NonlinearFactor::shared_ptr marginalFactor = marginalizeKeysFromFactor(factor, separatorKeys);
//      if(marginalFactor) {
//        smootherSummarization.push_back(marginalFactor);
//      }
//      if(marginalFactor) {
//        marginalFactor->print("Summarized Factor After Marginalization:\n");
//      } else {
//        std::cout << "Summarized Factor After Marginalization:\n{NULL}" << std::endl;
//      }
//    }
//  }
//  // Insert cached factors for any smoother branches
//  BOOST_FOREACH(const ISAM2Clique::shared_ptr& clique, smootherBranches) {
//    clique->cachedFactor()->print("Summarized Cached Factor:\n");
//    LinearizedGaussianFactor::shared_ptr factor;
//    if(const JacobianFactor::shared_ptr rhs = boost::dynamic_pointer_cast<JacobianFactor>(clique->cachedFactor()))
//      factor = LinearizedJacobianFactor::shared_ptr(new LinearizedJacobianFactor(rhs, ordering, linpoint));
//    else if(const HessianFactor::shared_ptr rhs = boost::dynamic_pointer_cast<HessianFactor>(clique->cachedFactor()))
//      factor = LinearizedHessianFactor::shared_ptr(new LinearizedHessianFactor(rhs, ordering, linpoint));
//    else
//      throw std::invalid_argument("In ConcurrentBatchFilter::process(...), cached factor is neither a JacobianFactor nor a HessianFactor");
//    factor->print("Summarized Linearized Factor:\n");
//    smootherSummarization.push_back(factor);
//  }
//  gttoc(extract_smoother_summarization);
//
//std::cout << "ConcurrentBatchFilter::synchronize(*)  Smoother Summarization:" << std::endl;
//BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, smootherSummarization) {
//if(boost::dynamic_pointer_cast<LinearizedGaussianFactor>(factor))
//  std::cout << "  L( ";
//else
//  std::cout << "  f( ";
//BOOST_FOREACH(Key key, factor->keys()) {
//  std::cout << DefaultKeyFormatter(key) << " ";
//}
//std::cout << ")" << std::endl;
//}
//
//  gttic(purge_smoother_information);
//  // Remove nonlinear factors being sent to the smoother
//  BOOST_FOREACH(size_t slot, smootherFactorSlots) {
//    removeFactor(slot);
//  }
//
//  // Remove the nonlinear keys being sent to the smoother
//  BOOST_FOREACH(Key key, smootherKeys) {
//    removeKey(key);
//  }
//
//  // Add all of the smoother summarization factors to the filter, keeping track of their locations for later removal
//  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, smootherSummarization) {
//    smootherSummarizationSlots_.push_back(insertFactor(factor));
//  }
//  gttoc(purge_smoother_information);
//
//  std::cout << "ConcurrentBatchFilter::synchronize(*) End" << std::endl;
//  gttoc(synchronize);
//}
//
///* ************************************************************************* */
//void ConcurrentBatchFilter::synchronize(const NonlinearFactorGraph& summarizedFactors) {
//  gttic(synchronize);
//
//std::cout << "ConcurrentBatchFilter::synchronize(*) Begin" << std::endl;
//
////// Check that solution before marginalization matches solution after marginalization
////Values solutionBefore;
////solutionBefore.insert(theta_);
////solutionBefore.update(smootherSeparatorValues_);
////Ordering orderingBefore = *graph_.orderingCOLAMD(solutionBefore);
////GaussianFactorGraph lingraphBefore = *graph_.linearize(solutionBefore, orderingBefore);
////GaussianMultifrontalSolver gmsBefore(lingraphBefore, true);
////VectorValues deltaBefore = *gmsBefore.optimize();
////solutionBefore = solutionBefore.retract(deltaBefore, orderingBefore);
//
//
//std::cout << "ConcurrentBatchFilter::synchronize(*)  Input Smoother Summarization:" << std::endl;
//BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, summarizedFactors) {
//  std::cout << "  f( ";
//  BOOST_FOREACH(Key key, factor->keys()) {
//    std::cout << DefaultKeyFormatter(key) << " ";
//  }
//  std::cout << ")" << std::endl;
//}
//
//  // At this step, we need to update the summarized factors representing the smoother,
//  // then update the solution by re-eliminating the nonlinear graph. In the process, we
//  // will create a Bayes Tree and form a new root clique. We will let CCOLAMD define
//  // the ordering, constraining the most recent variables (variables > lag) to the
//  // top of the tree.
//  //
//  // Once formed, we then need to determine which branches will be sent to the smoother,
//  // which branches constitute the filter, and which clique(s) form the root.
//  //
//  // Ideally, all of the old variables will be in the smoother. But, depending on the
//  // factors and the ordering, there may be some cliques with recent and old variables.
//  // So, we define the smoother branches as those remaining after removing any clique
//  // with any recent variables.
//  //
//  // The root should be the smallest set of variables that separate the smoother from
//  // the rest of the graph. Thus, the root will be the parents of all the smoother
//  // branches (plus the path to the root to ensure a connected, full-rank system).
//  //
//  // The filter branches are then defined as the children of the root that are not
//  // smoother branches.
//
//
////std::cout << "Previous Graph:" << std::endl;
////for(size_t i = 0; i < graph_.size(); ++i) {
////  if(graph_[i]) {
////    std::cout << "  f" << i << "( ";
////    BOOST_FOREACH(Key key, graph_[i]->keys()) {
////      std::cout << DefaultKeyFormatter(key) << " ";
////    }
////    std::cout << ")" << std::endl;
////  }
////}
//
//// TODO: Temporarily remove smoother effects
//  // Remove the previous smoother summarization from the graph
//  BOOST_FOREACH(size_t slot, smootherSummarizationSlots_) {
//    removeFactor(slot);
//  }
//  smootherSummarizationSlots_.clear();
//
//std::cout << "Removed Previous Smoother Summarization:" << std::endl;
//for(size_t i = 0; i < graph_.size(); ++i) {
//  if(graph_[i]) {
//    std::cout << "  f" << i << "( ";
//    BOOST_FOREACH(Key key, graph_[i]->keys()) {
//      std::cout << DefaultKeyFormatter(key) << " ";
//    }
//    std::cout << ")" << std::endl;
//  }
//}
//
//// TODO: Temporarily remove smoother effects
//  // Insert the updated smoother summarized factors
//  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, summarizedFactors) {
//    smootherSummarizationSlots_.push_back(insertFactor(factor));
//  }
//
//std::cout << "Added New Smoother Summarization:" << std::endl;
//for(size_t i = 0; i < graph_.size(); ++i) {
//  if(graph_[i]) {
//    std::cout << "  f" << i << "( ";
//    BOOST_FOREACH(Key key, graph_[i]->keys()) {
//      std::cout << DefaultKeyFormatter(key) << " ";
//    }
//    std::cout << ")" << std::endl;
//  }
//}
//
//// TODO: Temporarily remove smoother effects
//  // Now that the smoother summarization has been updated, re-optimize the filter
//  update();
//
//
//  // The filter should now be optimal (up to the linearization point of the separator variables)
//  // Calculate a new separator, a new filter summarization, a new smoother summarization, and the
//  // set of factors to send to the smoother
//
//
//
//  // Force variables associated with root keys to keep the same linearization point
//  // TODO: This may be too many variables. It may only require that the smoother separator keys be kept constant.
//  gttic(enforce_consistency);
//  Values linpoint = theta_;
//  if(separatorValues_.size() > 0) {
//    linpoint.update(separatorValues_);
//  }
//  gttoc(enforce_consistency);
//
//std::cout << "ConcurrentBatchFilter::synchronize(*)  Old Smoother Separator Keys:  ";
//BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, separatorValues_) {
//  std::cout << DefaultKeyFormatter(key_value.key) << " ";
//}
//std::cout << std::endl;
//
////std::cout << "ConcurrentBatchFilter::synchronize(*)  Old Root Keys:  ";
////BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, rootValues_) {
////  std::cout << DefaultKeyFormatter(key_value.key) << " ";
////}
////std::cout << std::endl;
//
////std::cout << "ConcurrentBatchFilter::synchronize(*)  All Keys:  ";
////BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, linpoint) {
////  std::cout << DefaultKeyFormatter(key_value.key) << " ";
////}
////std::cout << std::endl;
//
//
//  // Calculate an ordering that puts variables newer than LAG at the top
//  gttic(compute_ordering);
//  std::set<Key> recentKeys = findKeysAfter(getCurrentTimestamp() - lag_);
//  std::map<Key, int> constraints;
//  BOOST_FOREACH(Key key, recentKeys) {
//    constraints[key] = 1;
//  }
//  Ordering ordering = *graph_.orderingCOLAMDConstrained(linpoint, constraints);
//  gttoc(compute_ordering);
//
////typedef std::map<Key, int> Constraints;
////std::cout << "ConcurrentBatchFilter::synchronize(*)  Constraints:  ";
////BOOST_FOREACH(const Constraints::value_type key_group, constraints) {
////  std::cout << DefaultKeyFormatter(key_group.first) << " ";
////}
////std::cout << std::endl;
//
////ordering.print("ConcurrentBatchFilter::synchronize(*)  Ordering:\n");
//
//
//  // Create a Bayes Tree using iSAM2 cliques
//  gttic(create_bayes_tree);
//  JunctionTree<GaussianFactorGraph, ISAM2Clique> jt(*graph_.linearize(linpoint, ordering));
//  ISAM2Clique::shared_ptr root = jt.eliminate(parameters_.getEliminationFunction());
//  BayesTree<GaussianConditional, ISAM2Clique> bayesTree;
//  bayesTree.insert(root);
//  gttoc(create_bayes_tree);
//
////std::cout << "ConcurrentBatchFilter::synchronize(*)  Create Bayes Tree:" << std::endl;
////SymbolicPrintTree(root, ordering, "  ");
//
//  // Find all of the filter cliques, and the smoother branches
//  // The smoother branches are defined as the children filter cliques that are not also filter cliques
//  gttic(identify_smoother_branches);
//  std::set<ISAM2Clique::shared_ptr> filterCliques;
//  std::set<ISAM2Clique::shared_ptr> smootherBranches;
//  BOOST_FOREACH(Key key, recentKeys) {
//    Index index = ordering.at(key);
//    const ISAM2Clique::shared_ptr& clique = bayesTree.nodes().at(index);
//    if(clique) {
//      // Add all of the child-cliques to the smoother branches
//      filterCliques.insert(clique);
//      smootherBranches.insert(clique->children().begin(), clique->children().end());
//    }
//  }
//  BOOST_FOREACH(const ISAM2Clique::shared_ptr& filterClique, filterCliques) {
//    smootherBranches.erase(filterClique);
//  }
//  gttoc(identify_smoother_branches);
//
//std::cout << "ConcurrentBatchFilter::synchronize(*)  Smoother Branches:" << std::endl;
//BOOST_FOREACH(const ISAM2Clique::shared_ptr& clique, smootherBranches) {
//  std::cout << "  P( ";
//  BOOST_FOREACH(Index index, clique->conditional()->frontals()) {
//    std::cout << DefaultKeyFormatter(ordering.key(index)) << " ";
//  }
//  if(clique->conditional()->nrParents() > 0) { std::cout << "| "; }
//  BOOST_FOREACH(Index index, clique->conditional()->parents()) {
//    std::cout << DefaultKeyFormatter(ordering.key(index)) << " ";
//  }
//  std::cout << ")" << std::endl;
//}
//
//  // Find all of the frontal keys in the smoother branches
//  gttic(find_smoother_keys);
//  std::set<Key> smootherKeys;
//  std::queue<ISAM2Clique::shared_ptr> smootherCliques;
//  BOOST_FOREACH(const ISAM2Clique::shared_ptr& clique, smootherBranches) {
//    smootherCliques.push(clique);
//  }
//  while(smootherCliques.size() > 0) {
//    // Extract the frontal keys from the next clique
//    BOOST_FOREACH(Index index, smootherCliques.front()->conditional()->frontals()) {
//      smootherKeys.insert(ordering.key(index));
//    }
//    // Add any children to the queue
//    BOOST_FOREACH(const ISAM2Clique::shared_ptr& child, smootherCliques.front()->children()) {
//      smootherCliques.push(child);
//    }
//    // Remove this clique from the queue
//    smootherCliques.pop();
//  }
//  // Store the current linearization point for future transmission to the smoother (skip any variables in the previous root)
//  smootherValues_.clear();
//  BOOST_FOREACH(Key key, smootherKeys) {
//    smootherValues_.insert(key, linpoint.at(key));
//  }
//  gttoc(find_smoother_keys);
//
//std::cout << "ConcurrentBatchFilter::synchronize(*)  Smoother Keys:  ";
//BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, smootherValues_) {
//  std::cout << DefaultKeyFormatter(key_value.key) << " ";
//}
//std::cout << std::endl;
//
//  // Find the set of smoother separator keys
//  // The separator keys are defined as the set of parents to current branches
//  //                                 + the parents of previous branches that are not new smoother keys
//  gttic(update_smoother_separator);
//  BOOST_FOREACH(Key key, smootherKeys) {
//    if(separatorValues_.exists(key)) {
//      separatorValues_.erase(key);
//    }
//  }
//  BOOST_FOREACH(const ISAM2Clique::shared_ptr& clique, smootherBranches) {
//    BOOST_FOREACH(Index index, clique->conditional()->parents()) {
//      Key key = ordering.key(index);
//      if(!separatorValues_.exists(key))
//        separatorValues_.insert(key, linpoint.at(key));
//    }
//  }
//  gttoc(update_smoother_separator);
//
//std::cout << "ConcurrentBatchFilter::synchronize(*)  New Smoother Separator Keys:  ";
//BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, separatorValues_) {
//  std::cout << DefaultKeyFormatter(key_value.key) << " ";
//}
//std::cout << std::endl;
//
//  // Calculate the new smoother summarization.
//  // This is potentially a combination of the previous summarization and cached factors from new smoother branches
//  gttic(extract_smoother_summarization);
//  // Add in previous summarization, marginalizing out non-smoother-separator keys
//  std::set<Key> smootherSeparatorKeys;
//  BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, separatorValues_) {
//    smootherSeparatorKeys.insert(key_value.key);
//  }
//  NonlinearFactorGraph smootherSummarization;
//  BOOST_FOREACH(size_t slot, smootherSummarizationSlots_) {
//if(smootherBranches.size() == 0) {
//graph_.at(slot)->print("Summarized Factor Before Marginalization:\n");
//    NonlinearFactor::shared_ptr marginalFactor = marginalizeKeysFromFactor(graph_.at(slot), smootherSeparatorKeys);
//    if(marginalFactor) {
//      smootherSummarization.push_back(marginalFactor);
//    }
//if(marginalFactor) {
//  marginalFactor->print("Summarized Factor After Marginalization:\n");
//} else {
//  std::cout << "Summarized Factor After Marginalization:\n{NULL}" << std::endl;
//}
//}
//    removeFactor(slot);
//  }
//  smootherSummarizationSlots_.clear();
//  // Insert cached factors for any smoother branches
//  BOOST_FOREACH(const ISAM2Clique::shared_ptr& clique, smootherBranches) {
//clique->cachedFactor()->print("Summarized Cached Factor:\n");
//    LinearizedGaussianFactor::shared_ptr factor;
//    if(const JacobianFactor::shared_ptr rhs = boost::dynamic_pointer_cast<JacobianFactor>(clique->cachedFactor()))
//      factor = LinearizedJacobianFactor::shared_ptr(new LinearizedJacobianFactor(rhs, ordering, linpoint));
//    else if(const HessianFactor::shared_ptr rhs = boost::dynamic_pointer_cast<HessianFactor>(clique->cachedFactor()))
//      factor = LinearizedHessianFactor::shared_ptr(new LinearizedHessianFactor(rhs, ordering, linpoint));
//    else
//      throw std::invalid_argument("In ConcurrentBatchFilter::process(...), cached factor is neither a JacobianFactor nor a HessianFactor");
//factor->print("Summarized Linearized Factor:\n");
//    smootherSummarization.push_back(factor);
//  }
//  gttoc(extract_smoother_summarization);
//
//std::cout << "ConcurrentBatchFilter::synchronize(*)  Smoother Summarization:" << std::endl;
//BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, smootherSummarization) {
//  if(boost::dynamic_pointer_cast<LinearizedGaussianFactor>(factor))
//    std::cout << "  L( ";
//  else
//    std::cout << "  f( ";
//  BOOST_FOREACH(Key key, factor->keys()) {
//    std::cout << DefaultKeyFormatter(key) << " ";
//  }
//  std::cout << ")" << std::endl;
//}
//
//  // Find all of the factors that contain at least one smoother key
//  // These nonlinear factors will be sent to the smoother
//  gttic(find_smoother_factors);
//  std::set<size_t> smootherFactorSlots = findFactorsWithAny(smootherKeys);
//  // Convert the set of slots into a set of factors
//  smootherFactors_.resize(0);
//  BOOST_FOREACH(size_t slot, smootherFactorSlots) {
//    smootherFactors_.push_back(graph_.at(slot));
//  }
//  gttoc(find_smoother_factors);
//
//std::cout << "ConcurrentBatchFilter::synchronize(*)  Smoother Factors:" << std::endl;
//BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, smootherFactors_) {
//  if(boost::dynamic_pointer_cast<LinearizedGaussianFactor>(factor))
//    std::cout << "  L( ";
//  else
//    std::cout << "  f( ";
//  BOOST_FOREACH(Key key, factor->keys()) {
//    std::cout << DefaultKeyFormatter(key) << " ";
//  }
//  std::cout << ")" << std::endl;
//}
//
//  // Find the Root Cliques
//  // The root is defined as the parents of the smoother separator + the path to the root
//  gttic(identify_root_cliques);
//  std::set<ISAM2Clique::shared_ptr> rootCliques;
//  BOOST_FOREACH(Key key, smootherSeparatorKeys) {
//    Index index = ordering.at(key);
//    const ISAM2Clique::shared_ptr& clique = bayesTree.nodes().at(index);
//    if(clique) {
//      rootCliques.insert(clique);
//    }
//  }
//  BOOST_FOREACH(const ISAM2Clique::shared_ptr& clique, rootCliques) {
//    ISAM2Clique::shared_ptr rootClique = clique;
//    while(!rootClique->isRoot()) {
//      rootClique = rootClique->parent();
//      rootCliques.insert(rootClique);
//    }
//  }
//  std::set<Key> rootKeys;
//  BOOST_FOREACH(const ISAM2Clique::shared_ptr& clique, rootCliques) {
//    BOOST_FOREACH(Index index, clique->conditional()->frontals()) {
//      rootKeys.insert(ordering.key(index));
//    }
//  }
//  gttoc(identify_root_cliques);
//
////std::cout << "ConcurrentBatchFilter::synchronize(*)  Root Cliques:" << std::endl;
////BOOST_FOREACH(const ISAM2Clique::shared_ptr& clique, rootCliques) {
////std::cout << "  P( ";
////BOOST_FOREACH(Index index, clique->conditional()->frontals()) {
////  std::cout << DefaultKeyFormatter(ordering.key(index)) << " ";
////}
////if(clique->conditional()->nrParents() > 0) { std::cout << "| "; }
////BOOST_FOREACH(Index index, clique->conditional()->parents()) {
////  std::cout << DefaultKeyFormatter(ordering.key(index)) << " ";
////}
////std::cout << ")" << std::endl;
////}
//
//  // Update the keys and linearization points of the new root
//  gttic(update_root_values);
//  rootValues_.clear();
//  BOOST_FOREACH(Key key, rootKeys) {
//    rootValues_.insert(key, linpoint.at(key));
//  }
//  gttoc(update_root_values);
//
////std::cout << "ConcurrentBatchFilter::synchronize(*)  New Root Keys:  ";
////BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, rootValues_) {
////  std::cout << DefaultKeyFormatter(key_value.key) << " ";
////}
////std::cout << std::endl;
//
//  // Identify the filter branches
//  // The filter branches are the children of the root that are not smoother branches.
//  gttic(identify_filter_branches);
//  std::set<ISAM2Clique::shared_ptr> filterBranches;
//  // Find the children of the root
//  BOOST_FOREACH(const ISAM2Clique::shared_ptr& clique, rootCliques) {
//    filterBranches.insert(clique->children().begin(), clique->children().end());
//  }
//  // Remove any root cliques that made it into the filter branches
//  BOOST_FOREACH(const ISAM2Clique::shared_ptr& clique, rootCliques) {
//    filterBranches.erase(clique);
//  }
//  // Remove the smoother branches
//  BOOST_FOREACH(const ISAM2Clique::shared_ptr& clique, smootherBranches) {
//    filterBranches.erase(clique);
//  }
//  gttoc(identify_filter_branches);
//
////std::cout << "ConcurrentBatchFilter::synchronize(*)  Filter Branches:" << std::endl;
////BOOST_FOREACH(const ISAM2Clique::shared_ptr& clique, filterBranches) {
////  std::cout << "  P( ";
////  BOOST_FOREACH(Index index, clique->conditional()->frontals()) {
////    std::cout << DefaultKeyFormatter(ordering.key(index)) << " ";
////  }
////  if(clique->conditional()->nrParents() > 0) { std::cout << "| "; }
////  BOOST_FOREACH(Index index, clique->conditional()->parents()) {
////    std::cout << DefaultKeyFormatter(ordering.key(index)) << " ";
////  }
////  std::cout << ")" << std::endl;
////}
//
//  // Extract cached factors from the filter branches and store for future transmission to the smoother
//  gttic(extract_filter_summarization);
//  filterSummarization_.resize(0);
//  BOOST_FOREACH(const ISAM2Clique::shared_ptr& clique, filterBranches) {
//    LinearizedGaussianFactor::shared_ptr factor;
//    if(const JacobianFactor::shared_ptr rhs = boost::dynamic_pointer_cast<JacobianFactor>(clique->cachedFactor()))
//      factor = LinearizedJacobianFactor::shared_ptr(new LinearizedJacobianFactor(rhs, ordering, linpoint));
//    else if(const HessianFactor::shared_ptr rhs = boost::dynamic_pointer_cast<HessianFactor>(clique->cachedFactor()))
//      factor = LinearizedHessianFactor::shared_ptr(new LinearizedHessianFactor(rhs, ordering, linpoint));
//    else
//      throw std::invalid_argument("In ConcurrentBatchFilter::process(...), cached factor is neither a JacobianFactor nor a HessianFactor");
//    filterSummarization_.push_back(factor);
//  }
//  gttoc(extract_filter_summarization);
//
//  // Find all factors that contain only root clique variables
//  // This information is not contained in either the smoother
//  // summarization or the filter summarization. This must be sent
//  // to the smoother as well. Including it as part of the
//  // root/filter summarization
//  gttic(extract_root_summarization);
//  std::set<size_t> rootFactorSlots = findFactorsWithOnly(rootKeys);
//  BOOST_FOREACH(size_t slot, rootFactorSlots) {
//    filterSummarization_.push_back(graph_.at(slot));
//  }
//  gttoc(extract_root_summarization);
//
////std::cout << "ConcurrentBatchFilter::synchronize(*)  Filter Summarization:" << std::endl;
////BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, filterSummarization_) {
////  if(boost::dynamic_pointer_cast<LinearizedGaussianFactor>(factor))
////    std::cout << "  L( ";
////  else
////    std::cout << "  f( ";
////  BOOST_FOREACH(Key key, factor->keys()) {
////    std::cout << DefaultKeyFormatter(key) << " ";
////  }
////  std::cout << ")" << std::endl;
////}
//
//  gttic(purge_smoother_information);
//  // Remove nonlinear factors being sent to the smoother
//  BOOST_FOREACH(size_t slot, smootherFactorSlots) {
//    removeFactor(slot);
//  }
//
//  // Remove the nonlinear keys being sent to the smoother
//  BOOST_FOREACH(Key key, smootherKeys) {
//    removeKey(key);
//  }
//
//  // Add all of the smoother summarization factors to the filter, keeping track of their locations for later removal
//  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, smootherSummarization) {
//    smootherSummarizationSlots_.push_back(insertFactor(factor));
//  }
//  gttoc(purge_smoother_information);
//
//
//
//
//
////  // Check that solution before marginalization matches solution after marginalization
////  Values solutionAfter;
////  solutionAfter.insert(theta_);
////  solutionAfter.update(smootherSeparatorValues_);
////  Ordering orderingAfter = *graph_.orderingCOLAMD(solutionAfter);
////  GaussianFactorGraph lingraphAfter = *graph_.linearize(solutionAfter, orderingAfter);
////  GaussianMultifrontalSolver gmsAfter(lingraphAfter, true);
////  VectorValues deltaAfter = *gmsAfter.optimize();
////  solutionAfter = solutionAfter.retract(deltaAfter, orderingAfter);
////
////  BOOST_FOREACH(Key key, smootherKeys) {
////    solutionBefore.erase(key);
////  }
////
////std::cout << "Solution before marginalization matches solution after marginalization: " << solutionBefore.equals(solutionAfter, 1.0e-9) << std::endl;
//
//std::cout << "ConcurrentBatchFilter::synchronize(*) End" << std::endl;
//
//  gttoc(synchronize);
//}
//
///* ************************************************************************* */
//void ConcurrentBatchFilter::removeKey(Key key) {
//
//  gttic(remove_keys);
//
//  // Erase the key from the Timestamp->Key map
//  double timestamp = keyTimestampMap_.at(key);
//
//  TimestampKeyMap::iterator iter = timestampKeyMap_.lower_bound(timestamp);
//  while(iter != timestampKeyMap_.end() && iter->first == timestamp) {
//    if(iter->second == key) {
//      timestampKeyMap_.erase(iter++);
//    } else {
//      ++iter;
//    }
//  }
//  // Erase the key from the Key->Timestamp map
//  keyTimestampMap_.erase(key);
//
//  // Erase the key from the values
//  theta_.erase(key);
//
//  // Erase the key from the set of linearized keys
//  // TODO: Should this ever even occur?
//  if(rootValues_.exists(key)) {
//    rootValues_.erase(key);
//  }
//
//  gttoc(remove_keys);
//}
//
///* ************************************************************************* */
//void ConcurrentBatchFilter::updateKeyTimestampMap(const KeyTimestampMap& timestamps) {
//
//  gttic(update_key_timestamp_map);
//
//  // Loop through each key and add/update it in the map
//  BOOST_FOREACH(const KeyTimestampMap::value_type& key_timestamp, timestamps) {
//    // Check to see if this key already exists inthe database
//    KeyTimestampMap::iterator keyIter = keyTimestampMap_.find(key_timestamp.first);
//
//    // If the key already exists
//    if(keyIter != keyTimestampMap_.end()) {
//      // Find the entry in the Timestamp-Key database
//      std::pair<TimestampKeyMap::iterator,TimestampKeyMap::iterator> range = timestampKeyMap_.equal_range(keyIter->second);
//      TimestampKeyMap::iterator timeIter = range.first;
//      while(timeIter->second != key_timestamp.first) {
//        ++timeIter;
//      }
//      // remove the entry in the Timestamp-Key database
//      timestampKeyMap_.erase(timeIter);
//      // insert an entry at the new time
//      timestampKeyMap_.insert(TimestampKeyMap::value_type(key_timestamp.second, key_timestamp.first));
//      // update the Key-Timestamp database
//      keyIter->second = key_timestamp.second;
//    } else {
//      // Add the Key-Timestamp database
//      keyTimestampMap_.insert(key_timestamp);
//      // Add the key to the Timestamp-Key database
//      timestampKeyMap_.insert(TimestampKeyMap::value_type(key_timestamp.second, key_timestamp.first));
//    }
//  }
//
//  gttoc(update_key_timestamp_map);
//}
//
///* ************************************************************************* */
//double ConcurrentBatchFilter::getCurrentTimestamp() const {
//
//  gttic(get_current_timestamp);
//
//  double timestamp;
//  if(timestampKeyMap_.size() > 0) {
//    timestamp = timestampKeyMap_.rbegin()->first;
//  } else {
//    timestamp = -std::numeric_limits<double>::max();
//  }
//
//  gttoc(get_current_timestamp);
//
//  return timestamp;
//}
//
///* ************************************************************************* */
//std::set<Key> ConcurrentBatchFilter::findKeysBefore(double timestamp) const {
//
//  gttic(find_keys_before);
//
//  std::set<Key> keys;
//  TimestampKeyMap::const_iterator end = timestampKeyMap_.lower_bound(timestamp);
//  for(TimestampKeyMap::const_iterator iter = timestampKeyMap_.begin(); iter != end; ++iter) {
//    keys.insert(iter->second);
//  }
//
//  gttoc(find_keys_before);
//
//  return keys;
//}
//
///* ************************************************************************* */
//std::set<Key> ConcurrentBatchFilter::findKeysAfter(double timestamp) const {
//
//  gttic(find_keys_after);
//
//  std::set<Key> keys;
//  TimestampKeyMap::const_iterator begin = timestampKeyMap_.lower_bound(timestamp);
//  for(TimestampKeyMap::const_iterator iter = begin; iter != timestampKeyMap_.end(); ++iter) {
//    keys.insert(iter->second);
//  }
//
//  gttoc(find_keys_after);
//
//  return keys;
//}
//
///* ************************************************************************* */
//std::set<size_t> ConcurrentBatchFilter::findFactorsWithAny(const std::set<Key>& keys) const {
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
//std::set<size_t> ConcurrentBatchFilter::findFactorsWithOnly(const std::set<Key>& keys) const {
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
//std::set<Key> ConcurrentBatchFilter::findSeparatorKeys(const NonlinearFactorGraph& graph) const {
//
//  // Create a random ordering
//  Ordering ordering = *theta_.orderingArbitrary();
//
//  // Calculate a Variable Index using this ordering
//  VariableIndex variableIndex(graph, theta_.size());
//
//  // Find the set of keys outside the smoothing lag
//  std::set<Key> outsideKeys = findKeysBefore(getCurrentTimestamp() - lag_);
//
//  // Find the set of factors that involve OutsideKeys
//  std::set<size_t> outsideFactorSlots;
//  BOOST_FOREACH(Key key, outsideKeys) {
//    Index index = ordering.at(key);
//    BOOST_FOREACH(size_t factorSlot, variableIndex[index]) {
//      outsideFactorSlots.insert(factorSlot);
//    }
//  }
//
//  // Find the set of all keys involved in OutsideFactors
//  std::set<Key> involvedKeys;
//  BOOST_FOREACH(size_t factorSlot, outsideFactorSlots) {
//    BOOST_FOREACH(Key key, graph.at(factorSlot)->keys()) {
//      involvedKeys.insert(key);
//    }
//  }
//
//  // The difference between the OutsideKeys and the InvolvedKeys is the set of keys
//  // inside the lag, but attached to keys outside the lag... i.e. the separator
//  std::set<Key> separatorKeys;
//  std::set_difference(involvedKeys.begin(), involvedKeys.end(), outsideKeys.begin(), outsideKeys.end(), std::inserter(separatorKeys, separatorKeys.end()));
//
//  return separatorKeys;
//}
//
///* ************************************************************************* */
//Ordering ConcurrentBatchFilter::computeOrdering(const NonlinearFactorGraph& graph) const {
//
//  // Find the set of keys outside the smoothing lag
//  std::set<Key> outsideKeys = findKeysBefore(getCurrentTimestamp() - lag_);
//
//  // Find the set of keys inseide the smoother lag
//  std::set<Key> insideKeys = findKeysAfter(getCurrentTimestamp() - lag_);
//
//  // Find the separator keys
//  std::set<Key> separatorKeys = findSeparatorKeys(graph);
//
//  // Remove separator keys from inside keys
//  BOOST_FOREACH(Key key, separatorKeys) {
//    insideKeys.erase(key);
//  }
//
//  // Create a set of COLAMD constraints to create an ordering {OutsideKeys InsideKeys SeparatorKeys}
//  typedef std::map<Key, int> Constraints;
//  Constraints orderingConstraints;
//  int group = 0;
//  if(outsideKeys.size() > 0) {
//    BOOST_FOREACH(Key key, outsideKeys) {
//      orderingConstraints[key] = group;
//    }
//    ++group;
//  }
//  if(insideKeys.size() > 0) {
//    BOOST_FOREACH(Key key, insideKeys) {
//      orderingConstraints[key] = group;
//    }
//    ++group;
//  }
//  if(separatorKeys.size() > 0) {
//    BOOST_FOREACH(Key key, separatorKeys) {
//      orderingConstraints[key] = group;
//    }
//    ++group;
//  }
//
//  // Create the constrained ordering
//  return *graph.orderingCOLAMDConstrained(theta_, orderingConstraints);
//}
//
///* ************************************************************************* */
//NonlinearFactor::shared_ptr ConcurrentBatchFilter::marginalizeKeysFromFactor(const NonlinearFactor::shared_ptr& factor, const std::set<Key>& remainingKeys) const {
//
//  const bool debug = false;
//  if(debug) std::cout << "ConcurrentBatchFilter::marginalizeKeys() START" << std::endl;
//
//  LinearizedGaussianFactor::shared_ptr linearizedFactor = boost::dynamic_pointer_cast<LinearizedGaussianFactor>(factor);
//  assert(linearizedFactor);
//
//  // Sort the keys for this factor
//  std::set<Key> factorKeys;
//  BOOST_FOREACH(Key key, *linearizedFactor) {
//    factorKeys.insert(key);
//  }
//
//  // Calculate the set of keys to marginalize
//  std::set<Key> marginalizeKeys;
//  std::set_difference(factorKeys.begin(), factorKeys.end(), remainingKeys.begin(), remainingKeys.end(), std::inserter(marginalizeKeys, marginalizeKeys.end()));
//
//  //
//  if(marginalizeKeys.size() == 0) {
//    // No keys need to be marginalized out. Simply return the original factor.
//    return linearizedFactor;
//  } else if(marginalizeKeys.size() == linearizedFactor->size()) {
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
//
//    //  (1) construct a linear factor graph
//    GaussianFactorGraph graph;
//    graph.push_back( linearizedFactor->linearize(linearizedFactor->linearizationPoint(), ordering) );
//
//    //  (2) solve for the marginal factor
//    // Perform partial elimination, resulting in a conditional probability ( P(MarginalizedVariable | RemainingVariables)
//    // and factors on the remaining variables ( f(RemainingVariables) ). These are the factors we need to add to iSAM2
//    std::vector<Index> variables;
//    BOOST_FOREACH(Key key, marginalizeKeys) {
//      variables.push_back(ordering.at(key));
//    }
//    std::pair<GaussianFactorGraph::sharedConditional, GaussianFactorGraph> result = graph.eliminate(variables);
//    graph = result.second;
//
//    //  (3) convert the marginal factors into Linearized Factors
//    NonlinearFactor::shared_ptr marginalFactor;
//    assert(graph.size() <= 1);
//    if(graph.size() > 0) {
//      // These factors are all generated from BayesNet conditionals. They should all be Jacobians.
//      JacobianFactor::shared_ptr jacobianFactor = boost::dynamic_pointer_cast<JacobianFactor>(graph.at(0));
//      assert(jacobianFactor);
//      marginalFactor = LinearizedJacobianFactor::shared_ptr(new LinearizedJacobianFactor(jacobianFactor, ordering, linearizedFactor->linearizationPoint()));
//    }
//    return marginalFactor;
//  }
//}

/* ************************************************************************* */

}/// namespace gtsam
