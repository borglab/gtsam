/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ConcurrentIncrementalFilter.cpp
 * @brief   An iSAM2-based Filter that implements the
 *          Concurrent Filtering and Smoothing interface.
 * @author  Stephen Williams
 */
#include <gtsam_unstable/nonlinear/ConcurrentIncrementalFilter.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/base/timing.h>
#include <gtsam/base/debug.h>

namespace gtsam {

/* ************************************************************************* */
void ConcurrentIncrementalFilter::print(const std::string& s, const KeyFormatter& keyFormatter) const {
  std::cout << s;
  isam2_.print("");
}

/* ************************************************************************* */
bool ConcurrentIncrementalFilter::equals(const ConcurrentFilter& rhs, double tol) const {
  const ConcurrentIncrementalFilter* filter = dynamic_cast<const ConcurrentIncrementalFilter*>(&rhs);
  return filter
      && isam2_.equals(filter->isam2_)
      && (currentSmootherSummarizationSlots_.size() == filter->currentSmootherSummarizationSlots_.size())
      && std::equal(currentSmootherSummarizationSlots_.begin(), currentSmootherSummarizationSlots_.end(), filter->currentSmootherSummarizationSlots_.begin())
      && previousSmootherSummarization_.equals(filter->previousSmootherSummarization_)
      && smootherShortcut_.equals(filter->smootherShortcut_)
      && smootherFactors_.equals(filter->smootherFactors_)
      && smootherValues_.equals(filter->smootherValues_);
}

/* ************************************************************************* */
ConcurrentIncrementalFilter::Result ConcurrentIncrementalFilter::update(const NonlinearFactorGraph& newFactors, const Values& newTheta,
    const std::optional<FastList<Key> >& keysToMove, const std::optional< FactorIndices >& removeFactorIndices) {

  gttic(update);

//  const bool debug = ISDEBUG("ConcurrentIncrementalFilter update");
  const bool debug = false;

  if(debug) std::cout << "ConcurrentIncrementalFilter::update  Begin" << std::endl;

  // Create the return result meta-data
  Result result;

  // We do not need to remove any factors at this time
  FactorIndices removedFactors;

  if(removeFactorIndices){
    removedFactors.insert(removedFactors.end(), removeFactorIndices->begin(), removeFactorIndices->end());
  }

  // Generate ordering constraints that force the 'keys to move' to the end
  std::optional<FastMap<Key,int> > orderingConstraints = {};
  if(keysToMove && keysToMove->size() > 0) {
    orderingConstraints = FastMap<Key,int>();
    int group = 1;
    // Set all existing variables to Group1
    if(isam2_.getLinearizationPoint().size() > 0) {
      for(const auto key: isam2_.getLinearizationPoint().keys()) {
        orderingConstraints->operator[](key) = group;
      }
      ++group;
    }
    // Assign new variables to the root
    for(const auto key: newTheta.keys()) {
      orderingConstraints->operator[](key) = group;
    }
    // Set marginalizable variables to Group0
    for(Key key: *keysToMove){
      orderingConstraints->operator[](key) = 0;
    }
  }

  // Create the set of linear keys that iSAM2 should hold constant
  // iSAM2 takes care of this for us; no need to specify additional noRelin keys
  std::optional<FastList<Key> > noRelinKeys = {};

  // Mark additional keys between the 'keys to move' and the leaves
  std::optional<FastList<Key> > additionalKeys = {};
  if(keysToMove && keysToMove->size() > 0) {
    std::set<Key> markedKeys;
    for(Key key: *keysToMove) {
      if(isam2_.getLinearizationPoint().exists(key)) {
        ISAM2Clique::shared_ptr clique = isam2_[key];
        GaussianConditional::const_iterator key_iter = clique->conditional()->begin();
        while(*key_iter != key) {
          markedKeys.insert(*key_iter);
          ++key_iter;
        }
        for(const ISAM2Clique::shared_ptr& child: clique->children) {
          RecursiveMarkAffectedKeys(key, child, markedKeys);
        }
      }
    }
    additionalKeys = FastList<Key>(markedKeys.begin(), markedKeys.end());
  }

  // Update the system using iSAM2
  gttic(isam2);
  ISAM2Result isam2Result = isam2_.update(newFactors, newTheta, removedFactors, orderingConstraints, noRelinKeys, additionalKeys);
  gttoc(isam2);

  if(keysToMove && keysToMove->size() > 0) {

    gttic(cache_smoother_factors);
    // Find the set of factors that will be removed
    FactorIndices removedFactorSlots = FindAdjacentFactors(isam2_, *keysToMove, currentSmootherSummarizationSlots_);
    // Cache these factors for later transmission to the smoother
    NonlinearFactorGraph removedFactors;
    for(size_t slot: removedFactorSlots) {
      const NonlinearFactor::shared_ptr& factor = isam2_.getFactorsUnsafe().at(slot);
      if(factor) {
        smootherFactors_.push_back(factor);
        removedFactors.push_back(factor);
      }
    }
    // Cache removed values for later transmission to the smoother
    for(Key key: *keysToMove) {
      smootherValues_.insert(key, isam2_.getLinearizationPoint().at(key));
    }
    gttoc(cache_smoother_factors);

    gttic(marginalize);
    FactorIndices marginalFactorsIndices;
    FactorIndices deletedFactorsIndices;
    isam2_.marginalizeLeaves(*keysToMove, marginalFactorsIndices, deletedFactorsIndices);
    currentSmootherSummarizationSlots_.insert(currentSmootherSummarizationSlots_.end(), marginalFactorsIndices.begin(), marginalFactorsIndices.end());
    for(size_t index: deletedFactorsIndices) {
      currentSmootherSummarizationSlots_.erase(std::remove(currentSmootherSummarizationSlots_.begin(), currentSmootherSummarizationSlots_.end(), index), currentSmootherSummarizationSlots_.end());
    }
    gttoc(marginalize);

    // Calculate a new shortcut
    updateShortcut(removedFactors);
  }

  // Extract the ConcurrentIncrementalFilter::Result information
  result.iterations = 1;
  result.linearVariables = isam2_.getFixedVariables().size();
  result.nonlinearVariables = isam2_.getLinearizationPoint().size() - result.linearVariables;
  result.newFactorsIndices = isam2Result.newFactorsIndices;
  result.variablesReeliminated = isam2Result.variablesReeliminated;
  result.variablesRelinearized = isam2Result.variablesRelinearized;
//  result.error = isam2_.getFactorsUnsafe().error(isam2_.calculateEstimate());

  if(debug) std::cout << "ConcurrentIncrementalFilter::update  End" << std::endl;

  gttoc(update);

  return result;
}

/* ************************************************************************* */
void ConcurrentIncrementalFilter::presync() {

  gttic(presync);

  gttoc(presync);
}

/* ************************************************************************* */
void ConcurrentIncrementalFilter::synchronize(const NonlinearFactorGraph& smootherSummarization, const Values& smootherSummarizationValues) {

  gttic(synchronize);
//  const bool debug = ISDEBUG("ConcurrentIncrementalFilter synchronize");
  const bool debug = false;

  if(debug) std::cout << "ConcurrentIncrementalFilter::synchronize  Begin" << std::endl;

  // Update the smoother summarization on the old separator
  previousSmootherSummarization_ = smootherSummarization;

  // Find the set of new separator keys
  const KeySet& newSeparatorKeys = isam2_.getFixedVariables();

  // Use the shortcut to calculate an updated marginal on the current separator
  // Combine just the shortcut and the previousSmootherSummarization
  NonlinearFactorGraph graph;
  graph.push_back(previousSmootherSummarization_);
  graph.push_back(smootherShortcut_);
  Values values;
  values.insert(smootherSummarizationValues);
  for(Key key: newSeparatorKeys) {
    if(!values.exists(key)) {
      values.insert(key, isam2_.getLinearizationPoint().at(key));
    }
  }

  // Force iSAM2 not to relinearize anything during this iteration
  FastList<Key> noRelinKeys;
  for(const auto key: isam2_.getLinearizationPoint().keys()) {
    noRelinKeys.push_back(key);
  }

  // Calculate the summarized factor on just the new separator keys
  NonlinearFactorGraph currentSmootherSummarization = internal::calculateMarginalFactors(graph, values, newSeparatorKeys,
      isam2_.params().getEliminationFunction());

  // Remove the old factors on the separator and insert the new ones
  FactorIndices removeFactors(currentSmootherSummarizationSlots_.begin(), currentSmootherSummarizationSlots_.end());
  ISAM2Result result = isam2_.update(currentSmootherSummarization, Values(), removeFactors, {}, noRelinKeys, {}, false);
  currentSmootherSummarizationSlots_ = result.newFactorsIndices;

  // Set the previous smoother summarization to the current smoother summarization and clear the smoother shortcut
  previousSmootherSummarization_ = currentSmootherSummarization;
  smootherShortcut_.resize(0);

  if(debug) std::cout << "ConcurrentIncrementalFilter::synchronize  End" << std::endl;

  gttoc(synchronize);
}

/* ************************************************************************* */
void ConcurrentIncrementalFilter::getSummarizedFactors(NonlinearFactorGraph& filterSummarization, Values& filterSummarizationValues) {

  gttic(get_summarized_factors);

  // Calculate the current filter summarization
  filterSummarization = calculateFilterSummarization();

  // Copy the current separator values into the output
  for(Key key: isam2_.getFixedVariables()) {
    filterSummarizationValues.insert(key, isam2_.getLinearizationPoint().at(key));
  }

  gttoc(get_summarized_factors);
}

/* ************************************************************************* */
void ConcurrentIncrementalFilter::getSmootherFactors(NonlinearFactorGraph& smootherFactors, Values& smootherValues) {

  gttic(get_smoother_factors);

  // Copy the previous calculated smoother summarization factors into the output
  smootherFactors.push_back(smootherFactors_);
  smootherValues.insert(smootherValues_);

  gttoc(get_smoother_factors);
}

/* ************************************************************************* */
void ConcurrentIncrementalFilter::postsync() {

  gttic(postsync);

  // Clear out the factors and values that were just sent to the smoother
  smootherFactors_.resize(0);
  smootherValues_.clear();

  gttoc(postsync);
}


/* ************************************************************************* */
void ConcurrentIncrementalFilter::RecursiveMarkAffectedKeys(const Key& key, const ISAM2Clique::shared_ptr& clique, std::set<Key>& additionalKeys) {

  // Check if the separator keys of the current clique contain the specified key
  if(std::find(clique->conditional()->beginParents(), clique->conditional()->endParents(), key) != clique->conditional()->endParents()) {

    // Mark the frontal keys of the current clique
    for(Key idx: clique->conditional()->frontals()) {
      additionalKeys.insert(idx);
    }

    // Recursively mark all of the children
    for(const ISAM2Clique::shared_ptr& child: clique->children) {
      RecursiveMarkAffectedKeys(key, child, additionalKeys);
    }
  }
  // If the key was not found in the separator/parents, then none of its children can have it either

}

/* ************************************************************************* */
FactorIndices ConcurrentIncrementalFilter::FindAdjacentFactors(const ISAM2& isam2, const FastList<Key>& keys, const FactorIndices& factorsToIgnore) {

  // Identify any new factors to be sent to the smoother (i.e. any factor involving keysToMove)
  FactorIndices removedFactorSlots;
  const VariableIndex& variableIndex = isam2.getVariableIndex();
  for(Key key: keys) {
    const auto& slots = variableIndex[key];
    removedFactorSlots.insert(removedFactorSlots.end(), slots.begin(), slots.end());
  }

  // Sort and remove duplicates
  std::sort(removedFactorSlots.begin(), removedFactorSlots.end());
  removedFactorSlots.erase(std::unique(removedFactorSlots.begin(), removedFactorSlots.end()), removedFactorSlots.end());

  // Remove any linear/marginal factor that made it into the set
  for(size_t index: factorsToIgnore) {
    removedFactorSlots.erase(std::remove(removedFactorSlots.begin(), removedFactorSlots.end(), index), removedFactorSlots.end());
  }

  return removedFactorSlots;
}

///* ************************************************************************* */
// TODO: Make this a static function
void ConcurrentIncrementalFilter::updateShortcut(const NonlinearFactorGraph& removedFactors) {

  // Calculate the set of shortcut keys: NewSeparatorKeys + OldSeparatorKeys
  KeySet shortcutKeys;
  for(size_t slot: currentSmootherSummarizationSlots_) {
    const NonlinearFactor::shared_ptr& factor = isam2_.getFactorsUnsafe().at(slot);
    if(factor) {
      shortcutKeys.insert(factor->begin(), factor->end());
    }
  }
  for(Key key: previousSmootherSummarization_.keys()) {
    shortcutKeys.insert(key);
  }

  // Combine the previous shortcut factor with all of the new factors being sent to the smoother
  NonlinearFactorGraph graph;
  graph.push_back(removedFactors);
  graph.push_back(smootherShortcut_);
  Values values;
  values.insert(smootherValues_);
  values.insert(isam2_.getLinearizationPoint());
  // Calculate the summarized factor on the shortcut keys
  smootherShortcut_ = internal::calculateMarginalFactors(graph, values, shortcutKeys,
      isam2_.params().getEliminationFunction());
}

/* ************************************************************************* */
// TODO: Make this a static function
NonlinearFactorGraph ConcurrentIncrementalFilter::calculateFilterSummarization() const {

  // The filter summarization factors are the resulting marginal factors on the separator
  // variables that result from marginalizing out all of the other variables

  // Find the set of current separator keys
  const KeySet& separatorKeys = isam2_.getFixedVariables();

  // Find all cliques that contain any separator variables
  std::set<ISAM2Clique::shared_ptr> separatorCliques;
  for(Key key: separatorKeys) {
    ISAM2Clique::shared_ptr clique = isam2_[key];
    separatorCliques.insert( clique );
  }

  // Create the set of clique keys LC:
  KeyVector cliqueKeys;
  for(const ISAM2Clique::shared_ptr& clique: separatorCliques) {
    for(Key key: clique->conditional()->frontals()) {
      cliqueKeys.push_back(key);
    }
  }
  std::sort(cliqueKeys.begin(), cliqueKeys.end());

  // Gather all factors that involve only clique keys
  std::set<size_t> cliqueFactorSlots;
  for(Key key: cliqueKeys) {
    for(size_t slot: isam2_.getVariableIndex()[key]) {
      const NonlinearFactor::shared_ptr& factor = isam2_.getFactorsUnsafe().at(slot);
      if(factor) {
        std::set<Key> factorKeys(factor->begin(), factor->end());
        if(std::includes(cliqueKeys.begin(), cliqueKeys.end(), factorKeys.begin(), factorKeys.end())) {
          cliqueFactorSlots.insert(slot);
        }
      }
    }
  }

  // Remove any factor included in the current smoother summarization
  for(size_t slot: currentSmootherSummarizationSlots_) {
    cliqueFactorSlots.erase(slot);
  }

  // Create a factor graph from the identified factors
  NonlinearFactorGraph graph;
  for(size_t slot: cliqueFactorSlots) {
    graph.push_back(isam2_.getFactorsUnsafe().at(slot));
  }

  // Find the set of children of the separator cliques
  std::set<ISAM2Clique::shared_ptr> childCliques;
  // Add all of the children
  for(const ISAM2Clique::shared_ptr& clique: separatorCliques) {
    childCliques.insert(clique->children.begin(), clique->children.end());
  }
  // Remove any separator cliques that were added because they were children of other separator cliques
  for(const ISAM2Clique::shared_ptr& clique: separatorCliques) {
    childCliques.erase(clique);
  }

  // Augment the factor graph with cached factors from the children
  for(const ISAM2Clique::shared_ptr& clique: childCliques) {
    LinearContainerFactor::shared_ptr factor(new LinearContainerFactor(clique->cachedFactor(), isam2_.getLinearizationPoint()));
    graph.push_back( factor );
  }

  // Calculate the marginal factors on the separator
  NonlinearFactorGraph filterSummarization = internal::calculateMarginalFactors(graph, isam2_.getLinearizationPoint(), separatorKeys,
      isam2_.params().getEliminationFunction());

  return filterSummarization;
}

/* ************************************************************************* */
}/// namespace gtsam
