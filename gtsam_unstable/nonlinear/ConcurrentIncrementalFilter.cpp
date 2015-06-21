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
    const boost::optional<FastList<Key> >& keysToMove, const boost::optional< std::vector<size_t> >& removeFactorIndices) {

  gttic(update);

//  const bool debug = ISDEBUG("ConcurrentIncrementalFilter update");
  const bool debug = false;

  if(debug) std::cout << "ConcurrentIncrementalFilter::update  Begin" << std::endl;

  // Create the return result meta-data
  Result result;

  // We do not need to remove any factors at this time
  gtsam::FastVector<size_t> removedFactors;

  if(removeFactorIndices){
    removedFactors.insert(removedFactors.end(), removeFactorIndices->begin(), removeFactorIndices->end());
  }

  // Generate ordering constraints that force the 'keys to move' to the end
  boost::optional<gtsam::FastMap<gtsam::Key,int> > orderingConstraints = boost::none;
  if(keysToMove && keysToMove->size() > 0) {
    orderingConstraints = gtsam::FastMap<gtsam::Key,int>();
    int group = 1;
    // Set all existing variables to Group1
    if(isam2_.getLinearizationPoint().size() > 0) {
      BOOST_FOREACH(const gtsam::Values::ConstKeyValuePair& key_value, isam2_.getLinearizationPoint()) {
        orderingConstraints->operator[](key_value.key) = group;
      }
      ++group;
    }
    // Assign new variables to the root
    BOOST_FOREACH(const gtsam::Values::ConstKeyValuePair& key_value, newTheta) {
      orderingConstraints->operator[](key_value.key) = group;
    }
    // Set marginalizable variables to Group0
    BOOST_FOREACH(gtsam::Key key, *keysToMove){
      orderingConstraints->operator[](key) = 0;
    }
  }

  // Create the set of linear keys that iSAM2 should hold constant
  // iSAM2 takes care of this for us; no need to specify additional noRelin keys
  boost::optional<gtsam::FastList<gtsam::Key> > noRelinKeys = boost::none;

  // Mark additional keys between the 'keys to move' and the leaves
  boost::optional<FastList<Key> > additionalKeys = boost::none;
  if(keysToMove && keysToMove->size() > 0) {
    std::set<Key> markedKeys;
    BOOST_FOREACH(gtsam::Key key, *keysToMove) {
      if(isam2_.getLinearizationPoint().exists(key)) {
        ISAM2Clique::shared_ptr clique = isam2_[key];
        GaussianConditional::const_iterator key_iter = clique->conditional()->begin();
        while(*key_iter != key) {
          markedKeys.insert(*key_iter);
          ++key_iter;
        }
        BOOST_FOREACH(const gtsam::ISAM2Clique::shared_ptr& child, clique->children) {
          RecursiveMarkAffectedKeys(key, child, markedKeys);
        }
      }
    }
    additionalKeys = FastList<Key>(markedKeys.begin(), markedKeys.end());
  }

  // Update the system using iSAM2
  gttic(isam2);
  gtsam::ISAM2Result isam2Result = isam2_.update(newFactors, newTheta, removedFactors, orderingConstraints, noRelinKeys, additionalKeys);
  gttoc(isam2);

  if(keysToMove && keysToMove->size() > 0) {

    gttic(cache_smoother_factors);
    // Find the set of factors that will be removed
    std::vector<size_t> removedFactorSlots = FindAdjacentFactors(isam2_, *keysToMove, currentSmootherSummarizationSlots_);
    // Cache these factors for later transmission to the smoother
    NonlinearFactorGraph removedFactors;
    BOOST_FOREACH(size_t slot, removedFactorSlots) {
      const NonlinearFactor::shared_ptr& factor = isam2_.getFactorsUnsafe().at(slot);
      if(factor) {
        smootherFactors_.push_back(factor);
        removedFactors.push_back(factor);
      }
    }
    // Cache removed values for later transmission to the smoother
    BOOST_FOREACH(Key key, *keysToMove) {
      smootherValues_.insert(key, isam2_.getLinearizationPoint().at(key));
    }
    gttoc(cache_smoother_factors);

    gttic(marginalize);
    std::vector<size_t> marginalFactorsIndices;
    std::vector<size_t> deletedFactorsIndices;
    isam2_.marginalizeLeaves(*keysToMove, marginalFactorsIndices, deletedFactorsIndices);
    currentSmootherSummarizationSlots_.insert(currentSmootherSummarizationSlots_.end(), marginalFactorsIndices.begin(), marginalFactorsIndices.end());
    BOOST_FOREACH(size_t index, deletedFactorsIndices) {
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
  BOOST_FOREACH(Key key, newSeparatorKeys) {
    if(!values.exists(key)) {
      values.insert(key, isam2_.getLinearizationPoint().at(key));
    }
  }

  // Force iSAM2 not to relinearize anything during this iteration
  FastList<Key> noRelinKeys;
  BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, isam2_.getLinearizationPoint()) {
    noRelinKeys.push_back(key_value.key);
  }

  // Calculate the summarized factor on just the new separator keys
  NonlinearFactorGraph currentSmootherSummarization = internal::calculateMarginalFactors(graph, values, newSeparatorKeys,
      isam2_.params().getEliminationFunction());

  // Remove the old factors on the separator and insert the new ones
  FastVector<size_t> removeFactors(currentSmootherSummarizationSlots_.begin(), currentSmootherSummarizationSlots_.end());
  ISAM2Result result = isam2_.update(currentSmootherSummarization, Values(), removeFactors, boost::none, noRelinKeys, boost::none, false);
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
  BOOST_FOREACH(Key key, isam2_.getFixedVariables()) {
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
    BOOST_FOREACH(Key idx, clique->conditional()->frontals()) {
      additionalKeys.insert(idx);
    }

    // Recursively mark all of the children
    BOOST_FOREACH(const ISAM2Clique::shared_ptr& child, clique->children) {
      RecursiveMarkAffectedKeys(key, child, additionalKeys);
    }
  }
  // If the key was not found in the separator/parents, then none of its children can have it either

}

/* ************************************************************************* */
std::vector<size_t> ConcurrentIncrementalFilter::FindAdjacentFactors(const ISAM2& isam2, const FastList<Key>& keys, const std::vector<size_t>& factorsToIgnore) {

  // Identify any new factors to be sent to the smoother (i.e. any factor involving keysToMove)
  std::vector<size_t> removedFactorSlots;
  const VariableIndex& variableIndex = isam2.getVariableIndex();
  BOOST_FOREACH(Key key, keys) {
    const FastVector<size_t>& slots = variableIndex[key];
    removedFactorSlots.insert(removedFactorSlots.end(), slots.begin(), slots.end());
  }

  // Sort and remove duplicates
  std::sort(removedFactorSlots.begin(), removedFactorSlots.end());
  removedFactorSlots.erase(std::unique(removedFactorSlots.begin(), removedFactorSlots.end()), removedFactorSlots.end());

  // Remove any linear/marginal factor that made it into the set
  BOOST_FOREACH(size_t index, factorsToIgnore) {
    removedFactorSlots.erase(std::remove(removedFactorSlots.begin(), removedFactorSlots.end(), index), removedFactorSlots.end());
  }

  return removedFactorSlots;
}

///* ************************************************************************* */
// TODO: Make this a static function
void ConcurrentIncrementalFilter::updateShortcut(const NonlinearFactorGraph& removedFactors) {

  // Calculate the set of shortcut keys: NewSeparatorKeys + OldSeparatorKeys
  KeySet shortcutKeys;
  BOOST_FOREACH(size_t slot, currentSmootherSummarizationSlots_) {
    const NonlinearFactor::shared_ptr& factor = isam2_.getFactorsUnsafe().at(slot);
    if(factor) {
      shortcutKeys.insert(factor->begin(), factor->end());
    }
  }
  BOOST_FOREACH(Key key, previousSmootherSummarization_.keys()) {
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
  BOOST_FOREACH(Key key, separatorKeys) {
    ISAM2Clique::shared_ptr clique = isam2_[key];
    separatorCliques.insert( clique );
  }

  // Create the set of clique keys LC:
  std::vector<Key> cliqueKeys;
  BOOST_FOREACH(const ISAM2Clique::shared_ptr& clique, separatorCliques) {
    BOOST_FOREACH(Key key, clique->conditional()->frontals()) {
      cliqueKeys.push_back(key);
    }
  }
  std::sort(cliqueKeys.begin(), cliqueKeys.end());

  // Gather all factors that involve only clique keys
  std::set<size_t> cliqueFactorSlots;
  BOOST_FOREACH(Key key, cliqueKeys) {
    BOOST_FOREACH(size_t slot, isam2_.getVariableIndex()[key]) {
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
  BOOST_FOREACH(size_t slot, currentSmootherSummarizationSlots_) {
    cliqueFactorSlots.erase(slot);
  }

  // Create a factor graph from the identified factors
  NonlinearFactorGraph graph;
  BOOST_FOREACH(size_t slot, cliqueFactorSlots) {
    graph.push_back(isam2_.getFactorsUnsafe().at(slot));
  }

  // Find the set of children of the separator cliques
  std::set<ISAM2Clique::shared_ptr> childCliques;
  // Add all of the children
  BOOST_FOREACH(const ISAM2Clique::shared_ptr& clique, separatorCliques) {
    childCliques.insert(clique->children.begin(), clique->children.end());
  }
  // Remove any separator cliques that were added because they were children of other separator cliques
  BOOST_FOREACH(const ISAM2Clique::shared_ptr& clique, separatorCliques) {
    childCliques.erase(clique);
  }

  // Augment the factor graph with cached factors from the children
  BOOST_FOREACH(const ISAM2Clique::shared_ptr& clique, childCliques) {
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
