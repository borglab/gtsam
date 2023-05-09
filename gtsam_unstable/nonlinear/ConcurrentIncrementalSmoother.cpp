/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ConcurrentIncrementalSmoother.cpp
 * @brief   An iSAM2-based Smoother that implements the
 *          Concurrent Filtering and Smoothing interface.
 * @author  Stephen Williams
 */

#include <gtsam_unstable/nonlinear/ConcurrentIncrementalSmoother.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/base/timing.h>
#include <gtsam/base/debug.h>

namespace gtsam {

/* ************************************************************************* */
void ConcurrentIncrementalSmoother::print(const std::string& s, const KeyFormatter& keyFormatter) const {
  std::cout << s;
  isam2_.print("");
}

/* ************************************************************************* */
bool ConcurrentIncrementalSmoother::equals(const ConcurrentSmoother& rhs, double tol) const {
  const ConcurrentIncrementalSmoother* smoother = dynamic_cast<const ConcurrentIncrementalSmoother*>(&rhs);
  return smoother
      && isam2_.equals(smoother->isam2_)
      && smootherFactors_.equals(smoother->smootherFactors_)
      && smootherValues_.equals(smoother->smootherValues_)
      && filterSummarizationFactors_.equals(smoother->filterSummarizationFactors_)
      && separatorValues_.equals(smoother->separatorValues_)
      && (filterSummarizationSlots_.size() == smoother->filterSummarizationSlots_.size())
      && std::equal(filterSummarizationSlots_.begin(), filterSummarizationSlots_.end(), smoother->filterSummarizationSlots_.begin())
      && smootherSummarization_.equals(smoother->smootherSummarization_);
}

/* ************************************************************************* */
ConcurrentIncrementalSmoother::Result ConcurrentIncrementalSmoother::update(const NonlinearFactorGraph& newFactors, const Values& newTheta,
    const boost::optional<FactorIndices>& removeFactorIndices) {

  gttic(update);

  // Create the return result meta-data
  Result result;

  FastVector<size_t> removedFactors;

  if(removeFactorIndices){
    // Be very careful to this line
    std::cout << "ConcurrentIncrementalSmoother::update    removeFactorIndices - not implemented yet" << std::endl;
    filterSummarizationSlots_.insert(filterSummarizationSlots_.end(), removeFactorIndices->begin(), removeFactorIndices->end());
    // before it was:
    //  removedFactors.insert(removedFactors.end(), removeFactorIndices->begin(), removeFactorIndices->end());
  }

  // Constrain the separator keys to remain in the root
  // Also, mark the separator keys as fixed linearization points
  FastMap<Key,int> constrainedKeys;
  FastList<Key> noRelinKeys;
  for(const auto key_value: separatorValues_) {
    constrainedKeys[key_value.key] = 1;
    noRelinKeys.push_back(key_value.key);
  }

  // Use iSAM2 to perform an update
  ISAM2Result isam2Result;
  if(isam2_.getFactorsUnsafe().size() + newFactors.size() + smootherFactors_.size() + filterSummarizationFactors_.size() > 0) {
    if(synchronizationUpdatesAvailable_) {
      // Augment any new factors/values with the cached data from the last synchronization
      NonlinearFactorGraph graph(newFactors);
      graph.push_back(smootherFactors_);
      graph.push_back(filterSummarizationFactors_);
      Values values(newTheta);
      // Unfortunately, we must be careful here, as some of the smoother values
      // and/or separator values may have been added previously
      for(const auto key_value: smootherValues_) {
        if(!isam2_.getLinearizationPoint().exists(key_value.key)) {
          values.insert(key_value.key, smootherValues_.at(key_value.key));
        }
      }
      for(const auto key_value: separatorValues_) {
        if(!isam2_.getLinearizationPoint().exists(key_value.key)) {
          values.insert(key_value.key, separatorValues_.at(key_value.key));
        }
      }

      // Update the system using iSAM2
      isam2Result = isam2_.update(graph, values, filterSummarizationSlots_, constrainedKeys, noRelinKeys);

      // Clear out the cache and update the filter summarization slots
      smootherFactors_.resize(0);
      smootherValues_.clear();
      filterSummarizationSlots_.clear();
      filterSummarizationSlots_.insert(filterSummarizationSlots_.end(),
          isam2Result.newFactorsIndices.end()-filterSummarizationFactors_.size(), isam2Result.newFactorsIndices.end());
      filterSummarizationFactors_.resize(0);
      synchronizationUpdatesAvailable_ = false;
    } else {
      // Update the system using iSAM2
      isam2Result = isam2_.update(newFactors, newTheta, FactorIndices(), constrainedKeys, noRelinKeys);
    }
  }

  // Extract the ConcurrentIncrementalSmoother::Result information
  result.iterations = 1;
  result.linearVariables = separatorValues_.size();
  result.nonlinearVariables = isam2_.getLinearizationPoint().size() - separatorValues_.size();
  result.error = isam2_.getFactorsUnsafe().error(isam2_.calculateEstimate());

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
void ConcurrentIncrementalSmoother::presync() {

  gttic(presync);

  gttoc(presync);
}

/* ************************************************************************* */
void ConcurrentIncrementalSmoother::getSummarizedFactors(NonlinearFactorGraph& summarizedFactors, Values& separatorValues) {

  gttic(get_summarized_factors);

  // Copy the previous calculated smoother summarization factors into the output
  summarizedFactors.push_back(smootherSummarization_);

  // Copy the separator values into the output
  separatorValues.insert(separatorValues_);

  gttoc(get_summarized_factors);
}

/* ************************************************************************* */
void ConcurrentIncrementalSmoother::synchronize(const NonlinearFactorGraph& smootherFactors, const Values& smootherValues,
    const NonlinearFactorGraph& summarizedFactors, const Values& separatorValues) {

  gttic(synchronize);

  // Store the new smoother factors and values for addition during the next update call
  smootherFactors_ = smootherFactors;
  smootherValues_ = smootherValues;

  // Store the new filter summarization and separator, to be replaced during the next update call
  filterSummarizationFactors_ = summarizedFactors;
  separatorValues_ = separatorValues;

  // Flag the next smoother update to include the synchronization data
  synchronizationUpdatesAvailable_ = true;

  gttoc(synchronize);
}

/* ************************************************************************* */
void ConcurrentIncrementalSmoother::postsync() {

  gttic(postsync);

  gttoc(postsync);
}

/* ************************************************************************* */
void ConcurrentIncrementalSmoother::updateSmootherSummarization() {

  // The smoother summarization factors are the resulting marginal factors on the separator
  // variables that result from marginalizing out all of the other variables
  // These marginal factors will be cached for later transmission to the filter using
  // linear container factors

  // Find all cliques that contain any separator variables
  std::set<ISAM2Clique::shared_ptr> separatorCliques;
  for(Key key: separatorValues_.keys()) {
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

  // Remove any factor included in the filter summarization
  for(size_t slot: filterSummarizationSlots_) {
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

  // Get the set of separator keys
  KeySet separatorKeys;
  for(const auto key_value: separatorValues_) {
    separatorKeys.insert(key_value.key);
  }

  // Calculate the marginal factors on the separator
  smootherSummarization_ = internal::calculateMarginalFactors(graph, isam2_.getLinearizationPoint(), separatorKeys,
      isam2_.params().getEliminationFunction());
}

/* ************************************************************************* */

}/// namespace gtsam
