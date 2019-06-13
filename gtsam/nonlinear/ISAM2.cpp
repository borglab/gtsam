/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ISAM2.cpp
 * @brief   Incremental update functionality (ISAM2) for BayesTree, with fluid
 * relinearization.
 * @author  Michael Kaess, Richard Roberts, Frank Dellaert
 */

#include <gtsam/nonlinear/ISAM2.h>

#include <gtsam/base/debug.h>
#include <gtsam/base/timing.h>
#include <gtsam/inference/BayesTree-inst.h>
#include <gtsam/inference/JunctionTree-inst.h>  // We need the inst file because we'll make a special JT templated on ISAM2
#include <gtsam/linear/GaussianEliminationTree.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

#include <boost/range/adaptors.hpp>
#include <boost/range/algorithm/copy.hpp>
namespace br {
using namespace boost::range;
using namespace boost::adaptors;
}  // namespace br

#include <algorithm>
#include <limits>
#include <map>
#include <utility>

using namespace std;

namespace gtsam {

// Instantiate base class
template class BayesTree<ISAM2Clique>;

static const bool kDisableReordering = false;
static const double kBatchThreshold = 0.65;

/* ************************************************************************* */
// Special BayesTree class that uses ISAM2 cliques - this is the result of
// reeliminating ISAM2 subtrees.
class ISAM2BayesTree : public ISAM2::Base {
 public:
  typedef ISAM2::Base Base;
  typedef ISAM2BayesTree This;
  typedef boost::shared_ptr<This> shared_ptr;

  ISAM2BayesTree() {}
};

/* ************************************************************************* */
// Special JunctionTree class that produces ISAM2 BayesTree cliques, used for
// reeliminating ISAM2 subtrees.
class ISAM2JunctionTree
    : public JunctionTree<ISAM2BayesTree, GaussianFactorGraph> {
 public:
  typedef JunctionTree<ISAM2BayesTree, GaussianFactorGraph> Base;
  typedef ISAM2JunctionTree This;
  typedef boost::shared_ptr<This> shared_ptr;

  explicit ISAM2JunctionTree(const GaussianEliminationTree& eliminationTree)
      : Base(eliminationTree) {}
};

/* ************************************************************************* */
ISAM2::ISAM2(const ISAM2Params& params) : params_(params), update_count_(0) {
  if (params_.optimizationParams.type() == typeid(ISAM2DoglegParams))
    doglegDelta_ =
        boost::get<ISAM2DoglegParams>(params_.optimizationParams).initialDelta;
}

/* ************************************************************************* */
ISAM2::ISAM2() : update_count_(0) {
  if (params_.optimizationParams.type() == typeid(ISAM2DoglegParams))
    doglegDelta_ =
        boost::get<ISAM2DoglegParams>(params_.optimizationParams).initialDelta;
}

/* ************************************************************************* */
bool ISAM2::equals(const ISAM2& other, double tol) const {
  return Base::equals(other, tol) && theta_.equals(other.theta_, tol) &&
         variableIndex_.equals(other.variableIndex_, tol) &&
         nonlinearFactors_.equals(other.nonlinearFactors_, tol) &&
         fixedVariables_ == other.fixedVariables_;
}

/* ************************************************************************* */
KeySet ISAM2::getAffectedFactors(const KeyList& keys) const {
  static const bool debug = false;
  if (debug) cout << "Getting affected factors for ";
  if (debug) {
    for (const Key key : keys) {
      cout << key << " ";
    }
  }
  if (debug) cout << endl;

  NonlinearFactorGraph allAffected;
  KeySet indices;
  for (const Key key : keys) {
    const VariableIndex::Factors& factors(variableIndex_[key]);
    indices.insert(factors.begin(), factors.end());
  }
  if (debug) cout << "Affected factors are: ";
  if (debug) {
    for (const size_t index : indices) {
      cout << index << " ";
    }
  }
  if (debug) cout << endl;
  return indices;
}

/* ************************************************************************* */
// retrieve all factors that ONLY contain the affected variables
// (note that the remaining stuff is summarized in the cached factors)

GaussianFactorGraph::shared_ptr ISAM2::relinearizeAffectedFactors(
    const FastList<Key>& affectedKeys, const KeySet& relinKeys) const {
  gttic(getAffectedFactors);
  KeySet candidates = getAffectedFactors(affectedKeys);
  gttoc(getAffectedFactors);

  NonlinearFactorGraph nonlinearAffectedFactors;

  gttic(affectedKeysSet);
  // for fast lookup below
  KeySet affectedKeysSet;
  affectedKeysSet.insert(affectedKeys.begin(), affectedKeys.end());
  gttoc(affectedKeysSet);

  gttic(check_candidates_and_linearize);
  auto linearized = boost::make_shared<GaussianFactorGraph>();
  for (Key idx : candidates) {
    bool inside = true;
    bool useCachedLinear = params_.cacheLinearizedFactors;
    for (Key key : nonlinearFactors_[idx]->keys()) {
      if (affectedKeysSet.find(key) == affectedKeysSet.end()) {
        inside = false;
        break;
      }
      if (useCachedLinear && relinKeys.find(key) != relinKeys.end())
        useCachedLinear = false;
    }
    if (inside) {
      if (useCachedLinear) {
#ifdef GTSAM_EXTRA_CONSISTENCY_CHECKS
        assert(linearFactors_[idx]);
        assert(linearFactors_[idx]->keys() == nonlinearFactors_[idx]->keys());
#endif
        linearized->push_back(linearFactors_[idx]);
      } else {
        auto linearFactor = nonlinearFactors_[idx]->linearize(theta_);
        linearized->push_back(linearFactor);
        if (params_.cacheLinearizedFactors) {
#ifdef GTSAM_EXTRA_CONSISTENCY_CHECKS
          assert(linearFactors_[idx]->keys() == linearFactor->keys());
#endif
          linearFactors_[idx] = linearFactor;
        }
      }
    }
  }
  gttoc(check_candidates_and_linearize);

  return linearized;
}

/* ************************************************************************* */
// find intermediate (linearized) factors from cache that are passed into the
// affected area

GaussianFactorGraph ISAM2::getCachedBoundaryFactors(const Cliques& orphans) {
  GaussianFactorGraph cachedBoundary;

  for (sharedClique orphan : orphans) {
    // retrieve the cached factor and add to boundary
    cachedBoundary.push_back(orphan->cachedFactor());
  }

  return cachedBoundary;
}

/* ************************************************************************* */
boost::shared_ptr<KeySet> ISAM2::recalculate(
    const KeySet& markedKeys, const KeySet& relinKeys,
    const KeyVector& observedKeys, const KeySet& unusedIndices,
    const boost::optional<FastMap<Key, int> >& constrainKeys,
    ISAM2Result* result) {
  // TODO(dellaert):  new factors are linearized twice,
  // the newFactors passed in are not used.

  const bool debug = ISDEBUG("ISAM2 recalculate");

  // Input: BayesTree(this), newFactors

// figures for paper, disable for timing
#ifdef PRINT_STATS
  static int counter = 0;
  int maxClique = 0;
  double avgClique = 0;
  int numCliques = 0;
  int nnzR = 0;
  if (counter > 0) {  // cannot call on empty tree
    GaussianISAM2_P::CliqueData cdata = this->getCliqueData();
    GaussianISAM2_P::CliqueStats cstats = cdata.getStats();
    maxClique = cstats.maxCONDITIONALSize;
    avgClique = cstats.avgCONDITIONALSize;
    numCliques = cdata.conditionalSizes.size();
    nnzR = calculate_nnz(this->root());
  }
  counter++;
#endif

  if (debug) {
    cout << "markedKeys: ";
    for (const Key key : markedKeys) {
      cout << key << " ";
    }
    cout << endl;
    cout << "observedKeys: ";
    for (const Key key : observedKeys) {
      cout << key << " ";
    }
    cout << endl;
  }

  // 1. Remove top of Bayes tree and convert to a factor graph:
  // (a) For each affected variable, remove the corresponding clique and all
  // parents up to the root. (b) Store orphaned sub-trees \BayesTree_{O} of
  // removed cliques.
  gttic(removetop);
  Cliques orphans;
  GaussianBayesNet affectedBayesNet;
  this->removeTop(KeyVector(markedKeys.begin(), markedKeys.end()),
                  affectedBayesNet, orphans);
  gttoc(removetop);

  //    FactorGraph<GaussianFactor> factors(affectedBayesNet);
  // bug was here: we cannot reuse the original factors, because then the cached
  // factors get messed up [all the necessary data is actually contained in the
  // affectedBayesNet, including what was passed in from the boundaries,
  //  so this would be correct; however, in the process we also generate new
  //  cached_ entries that will be wrong (ie. they don't contain what would be
  //  passed up at a certain point if batch elimination was done, but that's
  //  what we need); we could choose not to update cached_ from here, but then
  //  the new information (and potentially different variable ordering) is not
  //  reflected in the cached_ values which again will be wrong]
  // so instead we have to retrieve the original linearized factors AND add the
  // cached factors from the boundary

  // BEGIN OF COPIED CODE

  // ordering provides all keys in conditionals, there cannot be others because
  // path to root included
  gttic(affectedKeys);
  FastList<Key> affectedKeys;
  for (const auto& conditional : affectedBayesNet)
    affectedKeys.insert(affectedKeys.end(), conditional->beginFrontals(),
                        conditional->endFrontals());
  gttoc(affectedKeys);

  boost::shared_ptr<KeySet> affectedKeysSet(
      new KeySet());  // Will return this result

  if (affectedKeys.size() >= theta_.size() * kBatchThreshold) {
    // Do a batch step - reorder and relinearize all variables
    gttic(batch);

    gttic(add_keys);
    br::copy(variableIndex_ | br::map_keys,
             std::inserter(*affectedKeysSet, affectedKeysSet->end()));

    // Removed unused keys:
    VariableIndex affectedFactorsVarIndex = variableIndex_;

    affectedFactorsVarIndex.removeUnusedVariables(unusedIndices.begin(),
                                                  unusedIndices.end());

    for (const Key key : unusedIndices) {
      affectedKeysSet->erase(key);
    }
    gttoc(add_keys);

    gttic(ordering);
    Ordering order;
    if (constrainKeys) {
      order =
          Ordering::ColamdConstrained(affectedFactorsVarIndex, *constrainKeys);
    } else {
      if (theta_.size() > observedKeys.size()) {
        // Only if some variables are unconstrained
        FastMap<Key, int> constraintGroups;
        for (Key var : observedKeys) constraintGroups[var] = 1;
        order = Ordering::ColamdConstrained(affectedFactorsVarIndex,
                                            constraintGroups);
      } else {
        order = Ordering::Colamd(affectedFactorsVarIndex);
      }
    }
    gttoc(ordering);

    gttic(linearize);
    GaussianFactorGraph linearized = *nonlinearFactors_.linearize(theta_);
    if (params_.cacheLinearizedFactors) linearFactors_ = linearized;
    gttoc(linearize);

    gttic(eliminate);
    ISAM2BayesTree::shared_ptr bayesTree =
        ISAM2JunctionTree(
            GaussianEliminationTree(linearized, affectedFactorsVarIndex, order))
            .eliminate(params_.getEliminationFunction())
            .first;
    gttoc(eliminate);

    gttic(insert);
    this->clear();
    this->roots_.insert(this->roots_.end(), bayesTree->roots().begin(),
                        bayesTree->roots().end());
    this->nodes_.insert(bayesTree->nodes().begin(), bayesTree->nodes().end());
    gttoc(insert);

    result->variablesReeliminated = affectedKeysSet->size();
    result->factorsRecalculated = nonlinearFactors_.size();

    lastAffectedMarkedCount = markedKeys.size();
    lastAffectedVariableCount = affectedKeysSet->size();
    lastAffectedFactorCount = linearized.size();

    // Reeliminated keys for detailed results
    if (params_.enableDetailedResults) {
      for (Key key : theta_.keys()) {
        result->detail->variableStatus[key].isReeliminated = true;
      }
    }

    gttoc(batch);

  } else {
    gttic(incremental);

    // 2. Add the new factors \Factors' into the resulting factor graph
    FastList<Key> affectedAndNewKeys;
    affectedAndNewKeys.insert(affectedAndNewKeys.end(), affectedKeys.begin(),
                              affectedKeys.end());
    affectedAndNewKeys.insert(affectedAndNewKeys.end(), observedKeys.begin(),
                              observedKeys.end());
    gttic(relinearizeAffected);
    GaussianFactorGraph factors(
        *relinearizeAffectedFactors(affectedAndNewKeys, relinKeys));
    if (debug) factors.print("Relinearized factors: ");
    gttoc(relinearizeAffected);

    if (debug) {
      cout << "Affected keys: ";
      for (const Key key : affectedKeys) {
        cout << key << " ";
      }
      cout << endl;
    }

    // Reeliminated keys for detailed results
    if (params_.enableDetailedResults) {
      for (Key key : affectedAndNewKeys) {
        result->detail->variableStatus[key].isReeliminated = true;
      }
    }

    result->variablesReeliminated = affectedAndNewKeys.size();
    result->factorsRecalculated = factors.size();
    lastAffectedMarkedCount = markedKeys.size();
    lastAffectedVariableCount = affectedKeys.size();
    lastAffectedFactorCount = factors.size();

#ifdef PRINT_STATS
    // output for generating figures
    cout << "linear: #markedKeys: " << markedKeys.size()
         << " #affectedVariables: " << affectedKeys.size()
         << " #affectedFactors: " << factors.size()
         << " maxCliqueSize: " << maxClique << " avgCliqueSize: " << avgClique
         << " #Cliques: " << numCliques << " nnzR: " << nnzR << endl;
#endif

    gttic(cached);
    // add the cached intermediate results from the boundary of the orphans ...
    GaussianFactorGraph cachedBoundary = getCachedBoundaryFactors(orphans);
    if (debug) cachedBoundary.print("Boundary factors: ");
    factors.push_back(cachedBoundary);
    gttoc(cached);

    gttic(orphans);
    // Add the orphaned subtrees
    for (const sharedClique& orphan : orphans)
      factors += boost::make_shared<BayesTreeOrphanWrapper<Clique> >(orphan);
    gttoc(orphans);

    // END OF COPIED CODE

    // 3. Re-order and eliminate the factor graph into a Bayes net (Algorithm
    // [alg:eliminate]), and re-assemble into a new Bayes tree (Algorithm
    // [alg:BayesTree])

    gttic(reorder_and_eliminate);

    gttic(list_to_set);
    // create a partial reordering for the new and contaminated factors
    // markedKeys are passed in: those variables will be forced to the end in
    // the ordering
    affectedKeysSet->insert(markedKeys.begin(), markedKeys.end());
    affectedKeysSet->insert(affectedKeys.begin(), affectedKeys.end());
    gttoc(list_to_set);

    VariableIndex affectedFactorsVarIndex(factors);

    gttic(ordering_constraints);
    // Create ordering constraints
    FastMap<Key, int> constraintGroups;
    if (constrainKeys) {
      constraintGroups = *constrainKeys;
    } else {
      constraintGroups = FastMap<Key, int>();
      const int group =
          observedKeys.size() < affectedFactorsVarIndex.size() ? 1 : 0;
      for (Key var : observedKeys)
        constraintGroups.insert(make_pair(var, group));
    }

    // Remove unaffected keys from the constraints
    for (FastMap<Key, int>::iterator iter = constraintGroups.begin();
         iter != constraintGroups.end();
         /*Incremented in loop ++iter*/) {
      if (unusedIndices.exists(iter->first) ||
          !affectedKeysSet->exists(iter->first))
        constraintGroups.erase(iter++);
      else
        ++iter;
    }
    gttoc(ordering_constraints);

    // Generate ordering
    gttic(Ordering);
    Ordering ordering =
        Ordering::ColamdConstrained(affectedFactorsVarIndex, constraintGroups);
    gttoc(Ordering);

    ISAM2BayesTree::shared_ptr bayesTree =
        ISAM2JunctionTree(
            GaussianEliminationTree(factors, affectedFactorsVarIndex, ordering))
            .eliminate(params_.getEliminationFunction())
            .first;

    gttoc(reorder_and_eliminate);

    gttic(reassemble);
    this->roots_.insert(this->roots_.end(), bayesTree->roots().begin(),
                        bayesTree->roots().end());
    this->nodes_.insert(bayesTree->nodes().begin(), bayesTree->nodes().end());
    gttoc(reassemble);

    // 4. The orphans have already been inserted during elimination

    gttoc(incremental);
  }

  // Root clique variables for detailed results
  if (params_.enableDetailedResults) {
    for (const sharedNode& root : this->roots())
      for (Key var : *root->conditional())
        result->detail->variableStatus[var].inRootClique = true;
  }

  return affectedKeysSet;
}

/* ************************************************************************* */
void ISAM2::addVariables(const Values& newTheta) {
  const bool debug = ISDEBUG("ISAM2 AddVariables");

  theta_.insert(newTheta);
  if (debug) newTheta.print("The new variables are: ");
  // Add zeros into the VectorValues
  delta_.insert(newTheta.zeroVectors());
  deltaNewton_.insert(newTheta.zeroVectors());
  RgProd_.insert(newTheta.zeroVectors());
}

/* ************************************************************************* */
void ISAM2::removeVariables(const KeySet& unusedKeys) {
  variableIndex_.removeUnusedVariables(unusedKeys.begin(), unusedKeys.end());
  for (Key key : unusedKeys) {
    delta_.erase(key);
    deltaNewton_.erase(key);
    RgProd_.erase(key);
    deltaReplacedMask_.erase(key);
    Base::nodes_.unsafe_erase(key);
    theta_.erase(key);
    fixedVariables_.erase(key);
  }
}

/* ************************************************************************* */
void ISAM2::expmapMasked(const KeySet& mask) {
  assert(theta_.size() == delta_.size());
  Values::iterator key_value;
  VectorValues::const_iterator key_delta;
#ifdef GTSAM_USE_TBB
  for (key_value = theta_.begin(); key_value != theta_.end(); ++key_value) {
    key_delta = delta_.find(key_value->key);
#else
  for (key_value = theta_.begin(), key_delta = delta_.begin();
       key_value != theta_.end(); ++key_value, ++key_delta) {
    assert(key_value->key == key_delta->first);
#endif
    Key var = key_value->key;
    assert(static_cast<size_t>(delta_[var].size()) == key_value->value.dim());
    assert(delta_[var].allFinite());
    if (mask.exists(var)) {
      Value* retracted = key_value->value.retract_(delta_[var]);
      key_value->value = *retracted;
      retracted->deallocate_();
#ifndef NDEBUG
      // If debugging, invalidate delta_ entries to Inf, to trigger assertions
      // if we try to re-use them.
      delta_[var] = Vector::Constant(delta_[var].rows(),
                                     numeric_limits<double>::infinity());
#endif
    }
  }
}

/* ************************************************************************* */
ISAM2Result ISAM2::update(
    const NonlinearFactorGraph& newFactors, const Values& newTheta,
    const FactorIndices& removeFactorIndices,
    const boost::optional<FastMap<Key, int> >& constrainedKeys,
    const boost::optional<FastList<Key> >& noRelinKeys,
    const boost::optional<FastList<Key> >& extraReelimKeys,
    bool force_relinearize) {
  const bool debug = ISDEBUG("ISAM2 update");
  const bool verbose = ISDEBUG("ISAM2 update verbose");

  gttic(ISAM2_update);

  this->update_count_++;

  lastAffectedVariableCount = 0;
  lastAffectedFactorCount = 0;
  lastAffectedCliqueCount = 0;
  lastAffectedMarkedCount = 0;
  lastBacksubVariableCount = 0;
  lastNnzTop = 0;
  ISAM2Result result;
  if (params_.enableDetailedResults)
    result.detail = ISAM2Result::DetailedResults();
  const bool relinearizeThisStep =
      force_relinearize || (params_.enableRelinearization &&
                            update_count_ % params_.relinearizeSkip == 0);

  if (verbose) {
    cout << "ISAM2::update\n";
    this->print("ISAM2: ");
  }

  // Update delta if we need it to check relinearization later
  if (relinearizeThisStep) {
    gttic(updateDelta);
    updateDelta(kDisableReordering);
    gttoc(updateDelta);
  }

  gttic(push_back_factors);
  // 1. Add any new factors \Factors:=\Factors\cup\Factors'.
  // Add the new factor indices to the result struct
  if (debug || verbose) newFactors.print("The new factors are: ");
  Impl::AddFactorsStep1(newFactors, params_.findUnusedFactorSlots,
                        &nonlinearFactors_, &result.newFactorsIndices);

  // Remove the removed factors
  NonlinearFactorGraph removeFactors;
  removeFactors.reserve(removeFactorIndices.size());
  for (size_t index : removeFactorIndices) {
    removeFactors.push_back(nonlinearFactors_[index]);
    nonlinearFactors_.remove(index);
    if (params_.cacheLinearizedFactors) linearFactors_.remove(index);
  }

  // Remove removed factors from the variable index so we do not attempt to
  // relinearize them
  variableIndex_.remove(removeFactorIndices.begin(), removeFactorIndices.end(),
                        removeFactors);

  // Compute unused keys and indices
  KeySet unusedKeys;
  KeySet unusedIndices;
  {
    // Get keys from removed factors and new factors, and compute unused keys,
    // i.e., keys that are empty now and do not appear in the new factors.
    KeySet removedAndEmpty;
    for (Key key : removeFactors.keys()) {
      if (variableIndex_[key].empty())
        removedAndEmpty.insert(removedAndEmpty.end(), key);
    }
    KeySet newFactorSymbKeys = newFactors.keys();
    std::set_difference(removedAndEmpty.begin(), removedAndEmpty.end(),
                        newFactorSymbKeys.begin(), newFactorSymbKeys.end(),
                        std::inserter(unusedKeys, unusedKeys.end()));

    // Get indices for unused keys
    for (Key key : unusedKeys) {
      unusedIndices.insert(unusedIndices.end(), key);
    }
  }
  gttoc(push_back_factors);

  gttic(add_new_variables);
  // 2. Initialize any new variables \Theta_{new} and add
  // \Theta:=\Theta\cup\Theta_{new}.
  addVariables(newTheta);
  // New keys for detailed results
  if (params_.enableDetailedResults) {
    for (Key key : newTheta.keys()) {
      result.detail->variableStatus[key].isNew = true;
    }
  }
  gttoc(add_new_variables);

  gttic(evaluate_error_before);
  if (params_.evaluateNonlinearError)
    result.errorBefore.reset(nonlinearFactors_.error(calculateEstimate()));
  gttoc(evaluate_error_before);

  gttic(gather_involved_keys);
  // 3. Mark linear update
  KeySet markedKeys = newFactors.keys();  // Get keys from new factors
  // Also mark keys involved in removed factors
  {
    KeySet markedRemoveKeys =
        removeFactors.keys();  // Get keys involved in removed factors
    markedKeys.insert(
        markedRemoveKeys.begin(),
        markedRemoveKeys.end());  // Add to the overall set of marked keys
  }
  // Also mark any provided extra re-eliminate keys
  if (extraReelimKeys) {
    for (Key key : *extraReelimKeys) {
      markedKeys.insert(key);
    }
  }

  // Observed keys for detailed results
  if (params_.enableDetailedResults) {
    for (Key key : markedKeys) {
      result.detail->variableStatus[key].isObserved = true;
    }
  }
  // NOTE: we use assign instead of the iterator constructor here because this
  // is a vector of size_t, so the constructor unintentionally resolves to
  // vector(size_t count, Key value) instead of the iterator constructor.
  KeyVector observedKeys;
  observedKeys.reserve(markedKeys.size());
  for (Key index : markedKeys) {
    if (unusedIndices.find(index) ==
        unusedIndices.end())  // Only add if not unused
      observedKeys.push_back(
          index);  // Make a copy of these, as we'll soon add to them
  }
  gttoc(gather_involved_keys);

  // Check relinearization if we're at the nth step, or we are using a looser
  // loop relin threshold
  KeySet relinKeys;
  if (relinearizeThisStep) {
    gttic(gather_relinearize_keys);
    // 4. Mark keys in \Delta above threshold \beta:
    // J=\{\Delta_{j}\in\Delta|\Delta_{j}\geq\beta\}.
    if (params_.enablePartialRelinearizationCheck)
      relinKeys = Impl::CheckRelinearizationPartial(
          roots_, delta_, params_.relinearizeThreshold);
    else
      relinKeys =
          Impl::CheckRelinearizationFull(delta_, params_.relinearizeThreshold);
    if (kDisableReordering)
      relinKeys = Impl::CheckRelinearizationFull(
          delta_, 0.0);  // This is used for debugging

    // Remove from relinKeys any keys whose linearization points are fixed
    for (Key key : fixedVariables_) {
      relinKeys.erase(key);
    }
    if (noRelinKeys) {
      for (Key key : *noRelinKeys) {
        relinKeys.erase(key);
      }
    }

    // Above relin threshold keys for detailed results
    if (params_.enableDetailedResults) {
      for (Key key : relinKeys) {
        result.detail->variableStatus[key].isAboveRelinThreshold = true;
        result.detail->variableStatus[key].isRelinearized = true;
      }
    }

    // Add the variables being relinearized to the marked keys
    KeySet markedRelinMask;
    for (const Key key : relinKeys) markedRelinMask.insert(key);
    markedKeys.insert(relinKeys.begin(), relinKeys.end());
    gttoc(gather_relinearize_keys);

    gttic(fluid_find_all);
    // 5. Mark all cliques that involve marked variables \Theta_{J} and all
    // their ancestors.
    if (!relinKeys.empty()) {
      for (const sharedClique& root : roots_)
        // add other cliques that have the marked ones in the separator
        root->findAll(markedRelinMask, &markedKeys);

      // Relin involved keys for detailed results
      if (params_.enableDetailedResults) {
        KeySet involvedRelinKeys;
        for (const sharedClique& root : roots_)
          root->findAll(markedRelinMask, &involvedRelinKeys);
        for (Key key : involvedRelinKeys) {
          if (!result.detail->variableStatus[key].isAboveRelinThreshold) {
            result.detail->variableStatus[key].isRelinearizeInvolved = true;
            result.detail->variableStatus[key].isRelinearized = true;
          }
        }
      }
    }
    gttoc(fluid_find_all);

    gttic(expmap);
    // 6. Update linearization point for marked variables:
    // \Theta_{J}:=\Theta_{J}+\Delta_{J}.
    if (!relinKeys.empty()) expmapMasked(markedRelinMask);
    gttoc(expmap);

    result.variablesRelinearized = markedKeys.size();
  } else {
    result.variablesRelinearized = 0;
  }

  gttic(linearize_new);
  // 7. Linearize new factors
  if (params_.cacheLinearizedFactors) {
    gttic(linearize);
    auto linearFactors = newFactors.linearize(theta_);
    if (params_.findUnusedFactorSlots) {
      linearFactors_.resize(nonlinearFactors_.size());
      for (size_t newFactorI = 0; newFactorI < newFactors.size(); ++newFactorI)
        linearFactors_[result.newFactorsIndices[newFactorI]] =
            (*linearFactors)[newFactorI];
    } else {
      linearFactors_.push_back(*linearFactors);
    }
    assert(nonlinearFactors_.size() == linearFactors_.size());
    gttoc(linearize);
  }
  gttoc(linearize_new);

  gttic(augment_VI);
  // Augment the variable index with the new factors
  if (params_.findUnusedFactorSlots)
    variableIndex_.augment(newFactors, result.newFactorsIndices);
  else
    variableIndex_.augment(newFactors);
  gttoc(augment_VI);

  gttic(recalculate);
  // 8. Redo top of Bayes tree
  boost::shared_ptr<KeySet> replacedKeys;
  if (!markedKeys.empty() || !observedKeys.empty())
    replacedKeys = recalculate(markedKeys, relinKeys, observedKeys,
                               unusedIndices, constrainedKeys, &result);

  // Update replaced keys mask (accumulates until back-substitution takes place)
  if (replacedKeys)
    deltaReplacedMask_.insert(replacedKeys->begin(), replacedKeys->end());
  gttoc(recalculate);

  // Update data structures to remove unused keys
  if (!unusedKeys.empty()) {
    gttic(remove_variables);
    removeVariables(unusedKeys);
    gttoc(remove_variables);
  }
  result.cliques = this->nodes().size();

  gttic(evaluate_error_after);
  if (params_.evaluateNonlinearError)
    result.errorAfter.reset(nonlinearFactors_.error(calculateEstimate()));
  gttoc(evaluate_error_after);

  return result;
}

/* ************************************************************************* */
void ISAM2::marginalizeLeaves(
    const FastList<Key>& leafKeysList,
    boost::optional<FactorIndices&> marginalFactorsIndices,
    boost::optional<FactorIndices&> deletedFactorsIndices) {
  // Convert to ordered set
  KeySet leafKeys(leafKeysList.begin(), leafKeysList.end());

  // Keep track of marginal factors - map from clique to the marginal factors
  // that should be incorporated into it, passed up from it's children.
  //  multimap<sharedClique, GaussianFactor::shared_ptr> marginalFactors;
  map<Key, vector<GaussianFactor::shared_ptr> > marginalFactors;

  // Keep track of variables removed in subtrees
  KeySet leafKeysRemoved;

  // Keep track of factors that get summarized by removing cliques
  KeySet factorIndicesToRemove;

  // Remove the subtree and throw away the cliques
  auto trackingRemoveSubtree = [&](const sharedClique& subtreeRoot) {
    const Cliques removedCliques = this->removeSubtree(subtreeRoot);
    for (const sharedClique& removedClique : removedCliques) {
      auto cg = removedClique->conditional();
      marginalFactors.erase(cg->front());
      leafKeysRemoved.insert(cg->beginFrontals(), cg->endFrontals());
      for (Key frontal : cg->frontals()) {
        // Add to factors to remove
        const auto& involved = variableIndex_[frontal];
        factorIndicesToRemove.insert(involved.begin(), involved.end());
#if !defined(NDEBUG)
        // Check for non-leaf keys
        if (!leafKeys.exists(frontal))
          throw std::runtime_error(
              "Requesting to marginalize variables that are not leaves, "
              "the ISAM2 object is now in an inconsistent state so should "
              "no longer be used.");
#endif
      }
    }
    return removedCliques;
  };

  // Remove each variable and its subtrees
  for (Key j : leafKeys) {
    if (!leafKeysRemoved.exists(j)) {  // If the index was not already removed
                                       // by removing another subtree

      // Traverse up the tree to find the root of the marginalized subtree
      sharedClique clique = nodes_[j];
      while (!clique->parent_._empty()) {
        // Check if parent contains a marginalized leaf variable.  Only need to
        // check the first variable because it is the closest to the leaves.
        sharedClique parent = clique->parent();
        if (leafKeys.exists(parent->conditional()->front()))
          clique = parent;
        else
          break;
      }

      // See if we should remove the whole clique
      bool marginalizeEntireClique = true;
      for (Key frontal : clique->conditional()->frontals()) {
        if (!leafKeys.exists(frontal)) {
          marginalizeEntireClique = false;
          break;
        }
      }

      // Remove either the whole clique or part of it
      if (marginalizeEntireClique) {
        // Remove the whole clique and its subtree, and keep the marginal
        // factor.
        auto marginalFactor = clique->cachedFactor();
        // We do not need the marginal factors associated with this clique
        // because their information is already incorporated in the new
        // marginal factor.  So, now associate this marginal factor with the
        // parent of this clique.
        marginalFactors[clique->parent()->conditional()->front()].push_back(
            marginalFactor);
        // Now remove this clique and its subtree - all of its marginal
        // information has been stored in marginalFactors.
        trackingRemoveSubtree(clique);
      } else {
        // Reeliminate the current clique and the marginals from its children,
        // then keep only the marginal on the non-marginalized variables.  We
        // get the childrens' marginals from any existing children, plus
        // the marginals from the marginalFactors multimap, which come from any
        // subtrees already marginalized out.

        // Add child marginals and remove marginalized subtrees
        GaussianFactorGraph graph;
        KeySet factorsInSubtreeRoot;
        Cliques subtreesToRemove;
        for (const sharedClique& child : clique->children) {
          // Remove subtree if child depends on any marginalized keys
          for (Key parent : child->conditional()->parents()) {
            if (leafKeys.exists(parent)) {
              subtreesToRemove.push_back(child);
              graph.push_back(child->cachedFactor());  // Add child marginal
              break;
            }
          }
        }
        Cliques childrenRemoved;
        for (const sharedClique& subtree : subtreesToRemove) {
          const Cliques removed = trackingRemoveSubtree(subtree);
          childrenRemoved.insert(childrenRemoved.end(), removed.begin(),
                                 removed.end());
        }

        // Add the factors that are pulled into the current clique by the
        // marginalized variables. These are the factors that involve
        // *marginalized* frontal variables in this clique but do not involve
        // frontal variables of any of its children.
        // TODO(dellaert): reuse cached linear factors
        KeySet factorsFromMarginalizedInClique_step1;
        for (Key frontal : clique->conditional()->frontals()) {
          if (leafKeys.exists(frontal))
            factorsFromMarginalizedInClique_step1.insert(
                variableIndex_[frontal].begin(), variableIndex_[frontal].end());
        }
        // Remove any factors in subtrees that we're removing at this step
        for (const sharedClique& removedChild : childrenRemoved) {
          for (Key indexInClique : removedChild->conditional()->frontals()) {
            for (Key factorInvolving : variableIndex_[indexInClique]) {
              factorsFromMarginalizedInClique_step1.erase(factorInvolving);
            }
          }
        }
        // Create factor graph from factor indices
        for (size_t i : factorsFromMarginalizedInClique_step1) {
          graph.push_back(nonlinearFactors_[i]->linearize(theta_));
        }

        // Reeliminate the linear graph to get the marginal and discard the
        // conditional
        auto cg = clique->conditional();
        const KeySet cliqueFrontals(cg->beginFrontals(), cg->endFrontals());
        KeyVector cliqueFrontalsToEliminate;
        std::set_intersection(cliqueFrontals.begin(), cliqueFrontals.end(),
                              leafKeys.begin(), leafKeys.end(),
                              std::back_inserter(cliqueFrontalsToEliminate));
        auto eliminationResult1 = params_.getEliminationFunction()(
            graph, Ordering(cliqueFrontalsToEliminate));

        // Add the resulting marginal
        if (eliminationResult1.second)
          marginalFactors[cg->front()].push_back(eliminationResult1.second);

        // Split the current clique
        // Find the position of the last leaf key in this clique
        DenseIndex nToRemove = 0;
        while (leafKeys.exists(cg->keys()[nToRemove])) ++nToRemove;

        // Make the clique's matrix appear as a subset
        const DenseIndex dimToRemove = cg->matrixObject().offset(nToRemove);
        cg->matrixObject().firstBlock() = nToRemove;
        cg->matrixObject().rowStart() = dimToRemove;

        // Change the keys in the clique
        KeyVector originalKeys;
        originalKeys.swap(cg->keys());
        cg->keys().assign(originalKeys.begin() + nToRemove, originalKeys.end());
        cg->nrFrontals() -= nToRemove;

        // Add to factorIndicesToRemove any factors involved in frontals of
        // current clique
        for (Key frontal : cliqueFrontalsToEliminate) {
          const auto& involved = variableIndex_[frontal];
          factorIndicesToRemove.insert(involved.begin(), involved.end());
        }

        // Add removed keys
        leafKeysRemoved.insert(cliqueFrontalsToEliminate.begin(),
                               cliqueFrontalsToEliminate.end());
      }
    }
  }

  // At this point we have updated the BayesTree, now update the remaining iSAM2
  // data structures

  // Gather factors to add - the new marginal factors
  GaussianFactorGraph factorsToAdd;
  for (const auto& key_factors : marginalFactors) {
    for (const auto& factor : key_factors.second) {
      if (factor) {
        factorsToAdd.push_back(factor);
        if (marginalFactorsIndices)
          marginalFactorsIndices->push_back(nonlinearFactors_.size());
        nonlinearFactors_.push_back(
            boost::make_shared<LinearContainerFactor>(factor));
        if (params_.cacheLinearizedFactors) linearFactors_.push_back(factor);
        for (Key factorKey : *factor) {
          fixedVariables_.insert(factorKey);
        }
      }
    }
  }
  variableIndex_.augment(factorsToAdd);  // Augment the variable index

  // Remove the factors to remove that have been summarized in the newly-added
  // marginal factors
  NonlinearFactorGraph removedFactors;
  for (size_t i : factorIndicesToRemove) {
    removedFactors.push_back(nonlinearFactors_[i]);
    nonlinearFactors_.remove(i);
    if (params_.cacheLinearizedFactors) linearFactors_.remove(i);
  }
  variableIndex_.remove(factorIndicesToRemove.begin(),
                        factorIndicesToRemove.end(), removedFactors);

  if (deletedFactorsIndices)
    deletedFactorsIndices->assign(factorIndicesToRemove.begin(),
                                  factorIndicesToRemove.end());

  // Remove the marginalized variables
  removeVariables(KeySet(leafKeys.begin(), leafKeys.end()));
}

/* ************************************************************************* */
void ISAM2::updateDelta(bool forceFullSolve) const {
  gttic(updateDelta);
  if (params_.optimizationParams.type() == typeid(ISAM2GaussNewtonParams)) {
    // If using Gauss-Newton, update with wildfireThreshold
    const ISAM2GaussNewtonParams& gaussNewtonParams =
        boost::get<ISAM2GaussNewtonParams>(params_.optimizationParams);
    const double effectiveWildfireThreshold =
        forceFullSolve ? 0.0 : gaussNewtonParams.wildfireThreshold;
    gttic(Wildfire_update);
    lastBacksubVariableCount = Impl::UpdateGaussNewtonDelta(
        roots_, deltaReplacedMask_, effectiveWildfireThreshold, &delta_);
    deltaReplacedMask_.clear();
    gttoc(Wildfire_update);

  } else if (params_.optimizationParams.type() == typeid(ISAM2DoglegParams)) {
    // If using Dogleg, do a Dogleg step
    const ISAM2DoglegParams& doglegParams =
        boost::get<ISAM2DoglegParams>(params_.optimizationParams);
    const double effectiveWildfireThreshold =
        forceFullSolve ? 0.0 : doglegParams.wildfireThreshold;

    // Do one Dogleg iteration
    gttic(Dogleg_Iterate);

    // Compute Newton's method step
    gttic(Wildfire_update);
    lastBacksubVariableCount = Impl::UpdateGaussNewtonDelta(
        roots_, deltaReplacedMask_, effectiveWildfireThreshold, &deltaNewton_);
    gttoc(Wildfire_update);

    // Compute steepest descent step
    const VectorValues gradAtZero = this->gradientAtZero();  // Compute gradient
    Impl::UpdateRgProd(roots_, deltaReplacedMask_, gradAtZero,
                       &RgProd_);  // Update RgProd
    const VectorValues dx_u = Impl::ComputeGradientSearch(
        gradAtZero, RgProd_);  // Compute gradient search point

    // Clear replaced keys mask because now we've updated deltaNewton_ and
    // RgProd_
    deltaReplacedMask_.clear();

    // Compute dogleg point
    DoglegOptimizerImpl::IterationResult doglegResult(
        DoglegOptimizerImpl::Iterate(
            *doglegDelta_, doglegParams.adaptationMode, dx_u, deltaNewton_,
            *this, nonlinearFactors_, theta_, nonlinearFactors_.error(theta_),
            doglegParams.verbose));
    gttoc(Dogleg_Iterate);

    gttic(Copy_dx_d);
    // Update Delta and linear step
    doglegDelta_ = doglegResult.delta;
    delta_ =
        doglegResult
            .dx_d;  // Copy the VectorValues containing with the linear solution
    gttoc(Copy_dx_d);
  }
}

/* ************************************************************************* */
Values ISAM2::calculateEstimate() const {
  gttic(ISAM2_calculateEstimate);
  const VectorValues& delta(getDelta());
  gttic(Expmap);
  return theta_.retract(delta);
  gttoc(Expmap);
}

/* ************************************************************************* */
const Value& ISAM2::calculateEstimate(Key key) const {
  const Vector& delta = getDelta()[key];
  return *theta_.at(key).retract_(delta);
}

/* ************************************************************************* */
Values ISAM2::calculateBestEstimate() const {
  updateDelta(true);  // Force full solve when updating delta_
  return theta_.retract(delta_);
}

/* ************************************************************************* */
Matrix ISAM2::marginalCovariance(Key key) const {
  return marginalFactor(key, params_.getEliminationFunction())
      ->information()
      .inverse();
}

/* ************************************************************************* */
const VectorValues& ISAM2::getDelta() const {
  if (!deltaReplacedMask_.empty()) updateDelta();
  return delta_;
}

/* ************************************************************************* */
double ISAM2::error(const VectorValues& x) const {
  return GaussianFactorGraph(*this).error(x);
}

/* ************************************************************************* */
static void gradientAtZeroTreeAdder(const boost::shared_ptr<ISAM2Clique>& root,
                                    VectorValues* g) {
  // Loop through variables in each clique, adding contributions
  DenseIndex variablePosition = 0;
  for (GaussianConditional::const_iterator jit = root->conditional()->begin();
       jit != root->conditional()->end(); ++jit) {
    const DenseIndex dim = root->conditional()->getDim(jit);
    pair<VectorValues::iterator, bool> pos_ins = g->tryInsert(
        *jit, root->gradientContribution().segment(variablePosition, dim));
    if (!pos_ins.second)
      pos_ins.first->second +=
          root->gradientContribution().segment(variablePosition, dim);
    variablePosition += dim;
  }

  // Recursively add contributions from children
  typedef boost::shared_ptr<ISAM2Clique> sharedClique;
  for (const sharedClique& child : root->children) {
    gradientAtZeroTreeAdder(child, g);
  }
}

/* ************************************************************************* */
VectorValues ISAM2::gradientAtZero() const {
  // Create result
  VectorValues g;

  // Sum up contributions for each clique
  for (const ISAM2::sharedClique& root : this->roots())
    gradientAtZeroTreeAdder(root, &g);

  return g;
}

}  // namespace gtsam
