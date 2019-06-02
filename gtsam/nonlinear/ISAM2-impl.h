/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ISAM2-impl.h
 * @brief   Incremental update functionality (ISAM2) for BayesTree, with fluid
 * relinearization.
 * @author  Michael Kaess, Richard Roberts, Frank Dellaert
 */

#pragma once

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Result.h>

#include <gtsam/base/debug.h>
#include <gtsam/inference/JunctionTree-inst.h>  // We need the inst file because we'll make a special JT templated on ISAM2
#include <gtsam/inference/Symbol.h>
#include <gtsam/inference/VariableIndex.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianEliminationTree.h>

#include <boost/range/adaptors.hpp>
#include <boost/range/algorithm/copy.hpp>
namespace br {
using namespace boost::range;
using namespace boost::adaptors;
}  // namespace br

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

namespace gtsam {

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
struct GTSAM_EXPORT DeltaImpl {
  struct GTSAM_EXPORT PartialSolveResult {
    ISAM2::sharedClique bayesTree;
  };

  struct GTSAM_EXPORT ReorderingMode {
    size_t nFullSystemVars;
    enum { /*AS_ADDED,*/ COLAMD } algorithm;
    enum { NO_CONSTRAINT, CONSTRAIN_LAST } constrain;
    boost::optional<FastMap<Key, int> > constrainedKeys;
  };

  /**
   * Update the Newton's method step point, using wildfire
   */
  static size_t UpdateGaussNewtonDelta(const ISAM2::Roots& roots,
                                       const KeySet& replacedKeys,
                                       double wildfireThreshold,
                                       VectorValues* delta);

  /**
   * Update the RgProd (R*g) incrementally taking into account which variables
   * have been recalculated in \c replacedKeys.  Only used in Dogleg.
   */
  static size_t UpdateRgProd(const ISAM2::Roots& roots,
                             const KeySet& replacedKeys,
                             const VectorValues& gradAtZero,
                             VectorValues* RgProd);

  /**
   * Compute the gradient-search point.  Only used in Dogleg.
   */
  static VectorValues ComputeGradientSearch(const VectorValues& gradAtZero,
                                            const VectorValues& RgProd);
};

/* ************************************************************************* */
/**
 * Implementation functions for update method
 * All of the methods below have clear inputs and outputs, even if not
 * functional: iSAM2 is inherintly imperative.
 */
struct GTSAM_EXPORT UpdateImpl {
  const ISAM2Params& params_;
  const ISAM2UpdateParams& updateParams_;
  UpdateImpl(const ISAM2Params& params, const ISAM2UpdateParams& updateParams)
      : params_(params), updateParams_(updateParams) {}

  // Provide some debugging information at the start of update
  static void LogStartingUpdate(const NonlinearFactorGraph& newFactors,
                                const ISAM2& isam2) {
    gttic(pushBackFactors);
    const bool debug = ISDEBUG("ISAM2 update");
    const bool verbose = ISDEBUG("ISAM2 update verbose");

    if (verbose) {
      std::cout << "ISAM2::update\n";
      isam2.print("ISAM2: ");
    }

    // Add the new factor indices to the result struct
    if (debug || verbose) {
      newFactors.print("The new factors are: ");
    }
  }

  // Add any new factors \Factors:=\Factors\cup\Factors'.
  void pushBackFactors(const NonlinearFactorGraph& newFactors,
                       NonlinearFactorGraph* nonlinearFactors,
                       GaussianFactorGraph* linearFactors,
                       VariableIndex* variableIndex,
                       ISAM2Result* result) const {
    gttic(pushBackFactors);

    // Perform the first part of the bookkeeping updates for adding new factors.
    // Adds them to the complete list of nonlinear factors, and populates the
    // list of new factor indices, both optionally finding and reusing empty
    // factor slots.
    result->newFactorsIndices = nonlinearFactors->add_factors(
        newFactors, params_.findUnusedFactorSlots);

    // Remove the removed factors
    NonlinearFactorGraph removedFactors;
    removedFactors.reserve(updateParams_.removeFactorIndices.size());
    for (const auto index : updateParams_.removeFactorIndices) {
      removedFactors.push_back(nonlinearFactors->at(index));
      nonlinearFactors->remove(index);
      if (params_.cacheLinearizedFactors) linearFactors->remove(index);
    }

    // Remove removed factors from the variable index so we do not attempt to
    // relinearize them
    variableIndex->remove(updateParams_.removeFactorIndices.begin(),
                          updateParams_.removeFactorIndices.end(),
                          removedFactors);
    result->keysWithRemovedFactors = removedFactors.keys();

    // Compute unused keys and indices
    // Get keys from removed factors and new factors, and compute unused keys,
    // i.e., keys that are empty now and do not appear in the new factors.
    KeySet removedAndEmpty;
    for (Key key : result->keysWithRemovedFactors) {
      if (variableIndex->empty(key))
        removedAndEmpty.insert(removedAndEmpty.end(), key);
    }
    KeySet newFactorSymbKeys = newFactors.keys();
    std::set_difference(
        removedAndEmpty.begin(), removedAndEmpty.end(),
        newFactorSymbKeys.begin(), newFactorSymbKeys.end(),
        std::inserter(result->unusedKeys, result->unusedKeys.end()));
  }

  // Mark linear update
  void gatherInvolvedKeys(const NonlinearFactorGraph& newFactors,
                          const NonlinearFactorGraph& nonlinearFactors,
                          ISAM2Result* result) const {
    gttic(gatherInvolvedKeys);
    result->markedKeys = newFactors.keys();  // Get keys from new factors
    // Also mark keys involved in removed factors
    result->markedKeys.insert(result->keysWithRemovedFactors.begin(),
                              result->keysWithRemovedFactors.end());

    // Also mark any provided extra re-eliminate keys
    if (updateParams_.extraReelimKeys) {
      for (Key key : *updateParams_.extraReelimKeys) {
        result->markedKeys.insert(key);
      }
    }

    // Also, keys that were not observed in existing factors, but whose affected
    // keys have been extended now (e.g. smart factors)
    if (updateParams_.newAffectedKeys) {
      for (const auto& factorAddedKeys : *updateParams_.newAffectedKeys) {
        const auto factorIdx = factorAddedKeys.first;
        const auto& affectedKeys = nonlinearFactors.at(factorIdx)->keys();
        result->markedKeys.insert(affectedKeys.begin(), affectedKeys.end());
      }
    }

    // Observed keys for detailed results
    if (params_.enableDetailedResults) {
      for (Key key : result->markedKeys) {
        result->detail->variableStatus[key].isObserved = true;
      }
    }

    for (Key index : result->markedKeys) {
      // Only add if not unused
      if (result->unusedKeys.find(index) == result->unusedKeys.end())
        // Make a copy of these, as we'll soon add to them
        result->observedKeys.push_back(index);
    }
  }

  static void CheckRelinearizationRecursiveMap(
      const FastMap<char, Vector>& thresholds, const VectorValues& delta,
      const ISAM2::sharedClique& clique, KeySet* relinKeys) {
    // Check the current clique for relinearization
    bool relinearize = false;
    for (Key var : *clique->conditional()) {
      // Find the threshold for this variable type
      const Vector& threshold = thresholds.find(Symbol(var).chr())->second;

      const Vector& deltaVar = delta[var];

      // Verify the threshold vector matches the actual variable size
      if (threshold.rows() != deltaVar.rows())
        throw std::invalid_argument(
            "Relinearization threshold vector dimensionality for '" +
            std::string(1, Symbol(var).chr()) +
            "' passed into iSAM2 parameters does not match actual variable "
            "dimensionality.");

      // Check for relinearization
      if ((deltaVar.array().abs() > threshold.array()).any()) {
        relinKeys->insert(var);
        relinearize = true;
      }
    }

    // If this node was relinearized, also check its children
    if (relinearize) {
      for (const ISAM2::sharedClique& child : clique->children) {
        CheckRelinearizationRecursiveMap(thresholds, delta, child, relinKeys);
      }
    }
  }

  static void CheckRelinearizationRecursiveDouble(
      double threshold, const VectorValues& delta,
      const ISAM2::sharedClique& clique, KeySet* relinKeys) {
    // Check the current clique for relinearization
    bool relinearize = false;
    for (Key var : *clique->conditional()) {
      double maxDelta = delta[var].lpNorm<Eigen::Infinity>();
      if (maxDelta >= threshold) {
        relinKeys->insert(var);
        relinearize = true;
      }
    }

    // If this node was relinearized, also check its children
    if (relinearize) {
      for (const ISAM2::sharedClique& child : clique->children) {
        CheckRelinearizationRecursiveDouble(threshold, delta, child, relinKeys);
      }
    }
  }

  /**
   * Find the set of variables to be relinearized according to
   * relinearizeThreshold. This check is performed recursively, starting at the
   * top of the tree. Once a variable in the tree does not need to be
   * relinearized, no further checks in that branch are performed. This is an
   * approximation of the Full version, designed to save time at the expense of
   * accuracy.
   * @param delta The linear delta to check against the threshold
   * @param keyFormatter Formatter for printing nonlinear keys during debugging
   * @return The set of variable indices in delta whose magnitude is greater
   * than or equal to relinearizeThreshold
   */
  static KeySet CheckRelinearizationPartial(
      const ISAM2::Roots& roots, const VectorValues& delta,
      const ISAM2Params::RelinearizationThreshold& relinearizeThreshold) {
    KeySet relinKeys;
    for (const ISAM2::sharedClique& root : roots) {
      if (relinearizeThreshold.type() == typeid(double))
        CheckRelinearizationRecursiveDouble(
            boost::get<double>(relinearizeThreshold), delta, root, &relinKeys);
      else if (relinearizeThreshold.type() == typeid(FastMap<char, Vector>))
        CheckRelinearizationRecursiveMap(
            boost::get<FastMap<char, Vector> >(relinearizeThreshold), delta,
            root, &relinKeys);
    }
    return relinKeys;
  }

  /**
   * Find the set of variables to be relinearized according to
   * relinearizeThreshold. Any variables in the VectorValues delta whose vector
   * magnitude is greater than or equal to relinearizeThreshold are returned.
   * @param delta The linear delta to check against the threshold
   * @param keyFormatter Formatter for printing nonlinear keys during debugging
   * @return The set of variable indices in delta whose magnitude is greater
   * than or equal to relinearizeThreshold
   */
  static KeySet CheckRelinearizationFull(
      const VectorValues& delta,
      const ISAM2Params::RelinearizationThreshold& relinearizeThreshold) {
    KeySet relinKeys;

    if (const double* threshold = boost::get<double>(&relinearizeThreshold)) {
      for (const VectorValues::KeyValuePair& key_delta : delta) {
        double maxDelta = key_delta.second.lpNorm<Eigen::Infinity>();
        if (maxDelta >= *threshold) relinKeys.insert(key_delta.first);
      }
    } else if (const FastMap<char, Vector>* thresholds =
                   boost::get<FastMap<char, Vector> >(&relinearizeThreshold)) {
      for (const VectorValues::KeyValuePair& key_delta : delta) {
        const Vector& threshold =
            thresholds->find(Symbol(key_delta.first).chr())->second;
        if (threshold.rows() != key_delta.second.rows())
          throw std::invalid_argument(
              "Relinearization threshold vector dimensionality for '" +
              std::string(1, Symbol(key_delta.first).chr()) +
              "' passed into iSAM2 parameters does not match actual variable "
              "dimensionality.");
        if ((key_delta.second.array().abs() > threshold.array()).any())
          relinKeys.insert(key_delta.first);
      }
    }

    return relinKeys;
  }

  // Mark keys in \Delta above threshold \beta:
  KeySet gatherRelinearizeKeys(const ISAM2::Roots& roots,
                               const VectorValues& delta,
                               const KeySet& fixedVariables,
                               ISAM2Result* result) const {
    gttic(gatherRelinearizeKeys);
    // J=\{\Delta_{j}\in\Delta|\Delta_{j}\geq\beta\}.
    KeySet relinKeys =
        params_.enablePartialRelinearizationCheck
            ? CheckRelinearizationPartial(roots, delta,
                                          params_.relinearizeThreshold)
            : CheckRelinearizationFull(delta, params_.relinearizeThreshold);
    if (updateParams_.forceFullSolve)
      relinKeys = CheckRelinearizationFull(delta, 0.0);  // for debugging

    // Remove from relinKeys any keys whose linearization points are fixed
    for (Key key : fixedVariables) {
      relinKeys.erase(key);
    }
    if (updateParams_.noRelinKeys) {
      for (Key key : *updateParams_.noRelinKeys) {
        relinKeys.erase(key);
      }
    }

    // Record above relinerization threshold keys in detailed results
    if (params_.enableDetailedResults) {
      for (Key key : relinKeys) {
        result->detail->variableStatus[key].isAboveRelinThreshold = true;
        result->detail->variableStatus[key].isRelinearized = true;
      }
    }

    // Add the variables being relinearized to the marked keys
    result->markedKeys.insert(relinKeys.begin(), relinKeys.end());
    return relinKeys;
  }

  // Mark all cliques that involve marked variables \Theta_{J} and all
  // their ancestors.
  void fluidFindAll(const ISAM2::Roots& roots, const KeySet& relinKeys,
                    ISAM2Result* result) const {
    gttic(fluidFindAll);
    for (const auto& root : roots)
      // add other cliques that have the marked ones in the separator
      root->findAll(relinKeys, &result->markedKeys);

    // Relinearization-involved keys for detailed results
    if (params_.enableDetailedResults) {
      KeySet involvedRelinKeys;
      for (const auto& root : roots)
        root->findAll(relinKeys, &involvedRelinKeys);
      for (Key key : involvedRelinKeys) {
        if (!result->detail->variableStatus[key].isAboveRelinThreshold) {
          result->detail->variableStatus[key].isRelinearizeInvolved = true;
          result->detail->variableStatus[key].isRelinearized = true;
        }
      }
    }
  }

  /**
   * Apply expmap to the given values, but only for indices appearing in
   * \c mask.  Values are expmapped in-place.
   * \param mask Mask on linear indices, only \c true entries are expmapped
   */
  static void ExpmapMasked(const VectorValues& delta, const KeySet& mask,
                           Values* theta) {
    gttic(ExpmapMasked);
    assert(theta->size() == delta.size());
    Values::iterator key_value;
    VectorValues::const_iterator key_delta;
#ifdef GTSAM_USE_TBB
    for (key_value = theta->begin(); key_value != theta->end(); ++key_value) {
      key_delta = delta.find(key_value->key);
#else
    for (key_value = theta->begin(), key_delta = delta.begin();
         key_value != theta->end(); ++key_value, ++key_delta) {
      assert(key_value->key == key_delta->first);
#endif
      Key var = key_value->key;
      assert(static_cast<size_t>(delta[var].size()) == key_value->value.dim());
      assert(delta[var].allFinite());
      if (mask.exists(var)) {
        Value* retracted = key_value->value.retract_(delta[var]);
        key_value->value = *retracted;
        retracted->deallocate_();
      }
    }
  }

  // Linearize new factors
  void linearizeNewFactors(const NonlinearFactorGraph& newFactors,
                           const Values& theta, size_t numNonlinearFactors,
                           const FactorIndices& newFactorsIndices,
                           GaussianFactorGraph* linearFactors) const {
    gttic(linearizeNewFactors);
    auto linearized = newFactors.linearize(theta);
    if (params_.findUnusedFactorSlots) {
      linearFactors->resize(numNonlinearFactors);
      for (size_t i = 0; i < newFactors.size(); ++i)
        (*linearFactors)[newFactorsIndices[i]] = (*linearized)[i];
    } else {
      linearFactors->push_back(*linearized);
    }
    assert(linearFactors->size() == numNonlinearFactors);
  }

  void augmentVariableIndex(const NonlinearFactorGraph& newFactors,
                            const FactorIndices& newFactorsIndices,
                            VariableIndex* variableIndex) const {
    gttic(augmentVariableIndex);
    // Augment the variable index with the new factors
    if (params_.findUnusedFactorSlots)
      variableIndex->augment(newFactors, newFactorsIndices);
    else
      variableIndex->augment(newFactors);

    // Augment it with existing factors which now affect to more variables:
    if (updateParams_.newAffectedKeys) {
      for (const auto& factorAddedKeys : *updateParams_.newAffectedKeys) {
        const auto factorIdx = factorAddedKeys.first;
        variableIndex->augmentExistingFactor(factorIdx, factorAddedKeys.second);
      }
    }
  }

  static void LogRecalculateKeys(const ISAM2Result& result) {
    const bool debug = ISDEBUG("ISAM2 recalculate");

    if (debug) {
      std::cout << "markedKeys: ";
      for (const Key key : result.markedKeys) {
        std::cout << key << " ";
      }
      std::cout << std::endl;
      std::cout << "observedKeys: ";
      for (const Key key : result.observedKeys) {
        std::cout << key << " ";
      }
      std::cout << std::endl;
    }
  }

  static FactorIndexSet GetAffectedFactors(const KeyList& keys,
                                           const VariableIndex& variableIndex) {
    gttic(GetAffectedFactors);
    FactorIndexSet indices;
    for (const Key key : keys) {
      const FactorIndices& factors(variableIndex[key]);
      indices.insert(factors.begin(), factors.end());
    }
    return indices;
  }

  // find intermediate (linearized) factors from cache that are passed into the
  // affected area
  static GaussianFactorGraph GetCachedBoundaryFactors(
      const ISAM2::Cliques& orphans) {
    GaussianFactorGraph cachedBoundary;

    for (const auto& orphan : orphans) {
      // retrieve the cached factor and add to boundary
      cachedBoundary.push_back(orphan->cachedFactor());
    }

    return cachedBoundary;
  }

  // retrieve all factors that ONLY contain the affected variables
  // (note that the remaining stuff is summarized in the cached factors)
  GaussianFactorGraph relinearizeAffectedFactors(
      const FastList<Key>& affectedKeys, const KeySet& relinKeys,
      const NonlinearFactorGraph& nonlinearFactors,
      const VariableIndex& variableIndex, const Values& theta,
      GaussianFactorGraph* linearFactors) const {
    FactorIndexSet candidates = GetAffectedFactors(affectedKeys, variableIndex);

    gttic(affectedKeysSet);
    // for fast lookup below
    KeySet affectedKeysSet;
    affectedKeysSet.insert(affectedKeys.begin(), affectedKeys.end());
    gttoc(affectedKeysSet);

    gttic(check_candidates_and_linearize);
    GaussianFactorGraph linearized;
    for (const FactorIndex idx : candidates) {
      bool inside = true;
      bool useCachedLinear = params_.cacheLinearizedFactors;
      for (Key key : nonlinearFactors[idx]->keys()) {
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
          assert((*linearFactors)[idx]);
          assert((*linearFactors)[idx]->keys() ==
                 nonlinearFactors[idx]->keys());
#endif
          linearized.push_back((*linearFactors)[idx]);
        } else {
          auto linearFactor = nonlinearFactors[idx]->linearize(theta);
          linearized.push_back(linearFactor);
          if (params_.cacheLinearizedFactors) {
#ifdef GTSAM_EXTRA_CONSISTENCY_CHECKS
            assert((*linearFactors)[idx]->keys() == linearFactor->keys());
#endif
            (*linearFactors)[idx] = linearFactor;
          }
        }
      }
    }
    gttoc(check_candidates_and_linearize);

    return linearized;
  }

  // Do a batch step - reorder and relinearize all variables
  void recalculateBatch(const Values& theta, const VariableIndex& variableIndex,
                        const NonlinearFactorGraph& nonlinearFactors,
                        GaussianFactorGraph* linearFactors,
                        KeySet* affectedKeysSet, ISAM2::Roots* roots,
                        ISAM2::Nodes* nodes, ISAM2Result* result) const {
    gttic(batch);

    gttic(add_keys);
    br::copy(variableIndex | br::map_keys,
             std::inserter(*affectedKeysSet, affectedKeysSet->end()));

    // Removed unused keys:
    VariableIndex affectedFactorsVarIndex = variableIndex;

    affectedFactorsVarIndex.removeUnusedVariables(result->unusedKeys.begin(),
                                                  result->unusedKeys.end());

    for (const Key key : result->unusedKeys) {
      affectedKeysSet->erase(key);
    }
    gttoc(add_keys);

    gttic(ordering);
    Ordering order;
    if (updateParams_.constrainedKeys) {
      order = Ordering::ColamdConstrained(affectedFactorsVarIndex,
                                          *updateParams_.constrainedKeys);
    } else {
      if (theta.size() > result->observedKeys.size()) {
        // Only if some variables are unconstrained
        FastMap<Key, int> constraintGroups;
        for (Key var : result->observedKeys) constraintGroups[var] = 1;
        order = Ordering::ColamdConstrained(affectedFactorsVarIndex,
                                            constraintGroups);
      } else {
        order = Ordering::Colamd(affectedFactorsVarIndex);
      }
    }
    gttoc(ordering);

    gttic(linearize);
    GaussianFactorGraph linearized = *nonlinearFactors.linearize(theta);
    if (params_.cacheLinearizedFactors) *linearFactors = linearized;
    gttoc(linearize);

    gttic(eliminate);
    ISAM2BayesTree::shared_ptr bayesTree =
        ISAM2JunctionTree(
            GaussianEliminationTree(linearized, affectedFactorsVarIndex, order))
            .eliminate(params_.getEliminationFunction())
            .first;
    gttoc(eliminate);

    gttic(insert);
    roots->clear();
    roots->insert(roots->end(), bayesTree->roots().begin(),
                  bayesTree->roots().end());
    nodes->clear();
    nodes->insert(bayesTree->nodes().begin(), bayesTree->nodes().end());
    gttoc(insert);

    result->variablesReeliminated = affectedKeysSet->size();
    result->factorsRecalculated = nonlinearFactors.size();

    // Reeliminated keys for detailed results
    if (params_.enableDetailedResults) {
      for (Key key : theta.keys()) {
        result->detail->variableStatus[key].isReeliminated = true;
      }
    }
  }

  void recalculateIncremental(const Values& theta,
                              const VariableIndex& variableIndex,
                              const NonlinearFactorGraph& nonlinearFactors,
                              const ISAM2::Cliques& orphans,
                              const KeySet& relinKeys,
                              GaussianFactorGraph* linearFactors,
                              const FastList<Key>& affectedKeys,
                              KeySet* affectedKeysSet, ISAM2::Roots* roots,
                              ISAM2::Nodes* nodes, ISAM2Result* result) const {
    gttic(incremental);
    const bool debug = ISDEBUG("ISAM2 recalculate");

    // 2. Add the new factors \Factors' into the resulting factor graph
    FastList<Key> affectedAndNewKeys;
    affectedAndNewKeys.insert(affectedAndNewKeys.end(), affectedKeys.begin(),
                              affectedKeys.end());
    affectedAndNewKeys.insert(affectedAndNewKeys.end(),
                              result->observedKeys.begin(),
                              result->observedKeys.end());
    gttic(relinearizeAffected);
    GaussianFactorGraph factors = relinearizeAffectedFactors(
        affectedAndNewKeys, relinKeys, nonlinearFactors, variableIndex, theta,
        linearFactors);
    gttoc(relinearizeAffected);

    if (debug) {
      factors.print("Relinearized factors: ");
      std::cout << "Affected keys: ";
      for (const Key key : affectedKeys) {
        std::cout << key << " ";
      }
      std::cout << std::endl;
    }

    // Reeliminated keys for detailed results
    if (params_.enableDetailedResults) {
      for (Key key : affectedAndNewKeys) {
        result->detail->variableStatus[key].isReeliminated = true;
      }
    }

    result->variablesReeliminated = affectedAndNewKeys.size();
    result->factorsRecalculated = factors.size();

    gttic(cached);
    // add the cached intermediate results from the boundary of the orphans
    // ...
    GaussianFactorGraph cachedBoundary = GetCachedBoundaryFactors(orphans);
    if (debug) cachedBoundary.print("Boundary factors: ");
    factors.push_back(cachedBoundary);
    gttoc(cached);

    gttic(orphans);
    // Add the orphaned subtrees
    for (const auto& orphan : orphans)
      factors +=
          boost::make_shared<BayesTreeOrphanWrapper<ISAM2::Clique> >(orphan);
    gttoc(orphans);

    // END OF COPIED CODE

    // 3. Re-order and eliminate the factor graph into a Bayes net (Algorithm
    // [alg:eliminate]), and re-assemble into a new Bayes tree (Algorithm
    // [alg:BayesTree])

    gttic(reorder_and_eliminate);

    gttic(list_to_set);
    // create a partial reordering for the new and contaminated factors
    // result->markedKeys are passed in: those variables will be forced to the
    // end in the ordering
    affectedKeysSet->insert(result->markedKeys.begin(),
                            result->markedKeys.end());
    affectedKeysSet->insert(affectedKeys.begin(), affectedKeys.end());
    gttoc(list_to_set);

    VariableIndex affectedFactorsVarIndex(factors);

    gttic(ordering_constraints);
    // Create ordering constraints
    FastMap<Key, int> constraintGroups;
    if (updateParams_.constrainedKeys) {
      constraintGroups = *updateParams_.constrainedKeys;
    } else {
      constraintGroups = FastMap<Key, int>();
      const int group =
          result->observedKeys.size() < affectedFactorsVarIndex.size() ? 1 : 0;
      for (Key var : result->observedKeys)
        constraintGroups.insert(std::make_pair(var, group));
    }

    // Remove unaffected keys from the constraints
    for (FastMap<Key, int>::iterator iter = constraintGroups.begin();
         iter != constraintGroups.end();
         /*Incremented in loop ++iter*/) {
      if (result->unusedKeys.exists(iter->first) ||
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
    roots->insert(roots->end(), bayesTree->roots().begin(),
                  bayesTree->roots().end());
    nodes->insert(bayesTree->nodes().begin(), bayesTree->nodes().end());
    gttoc(reassemble);

    // 4. The orphans have already been inserted during elimination
  }

  KeySet recalculate(const Values& theta, const VariableIndex& variableIndex,
                     const NonlinearFactorGraph& nonlinearFactors,
                     const GaussianBayesNet& affectedBayesNet,
                     const ISAM2::Cliques& orphans, const KeySet& relinKeys,
                     GaussianFactorGraph* linearFactors, ISAM2::Roots* roots,
                     ISAM2::Nodes* nodes, ISAM2Result* result) const {
    gttic(recalculate);
    LogRecalculateKeys(*result);

    // FactorGraph<GaussianFactor> factors(affectedBayesNet);
    // bug was here: we cannot reuse the original factors, because then the
    // cached factors get messed up [all the necessary data is actually
    // contained in the affectedBayesNet, including what was passed in from the
    // boundaries, so this would be correct; however, in the process we also
    // generate new cached_ entries that will be wrong (ie. they don't contain
    // what would be passed up at a certain point if batch elimination was done,
    // but that's what we need); we could choose not to update cached_ from
    // here, but then the new information (and potentially different variable
    // ordering) is not reflected in the cached_ values which again will be
    // wrong] so instead we have to retrieve the original linearized factors AND
    // add the cached factors from the boundary

    // BEGIN OF COPIED CODE

    // ordering provides all keys in conditionals, there cannot be others
    // because path to root included
    gttic(affectedKeys);
    FastList<Key> affectedKeys;
    for (const auto& conditional : affectedBayesNet)
      affectedKeys.insert(affectedKeys.end(), conditional->beginFrontals(),
                          conditional->endFrontals());
    gttoc(affectedKeys);

    KeySet affectedKeysSet;  // Will return this result

    static const double kBatchThreshold = 0.65;
    if (affectedKeys.size() >= theta.size() * kBatchThreshold) {
      // Do a batch step - reorder and relinearize all variables
      recalculateBatch(theta, variableIndex, nonlinearFactors, linearFactors,
                       &affectedKeysSet, roots, nodes, result);
    } else {
      recalculateIncremental(theta, variableIndex, nonlinearFactors, orphans,
                             relinKeys, linearFactors, affectedKeys,
                             &affectedKeysSet, roots, nodes, result);
    }

    // Root clique variables for detailed results
    if (params_.enableDetailedResults) {
      for (const auto& root : *roots)
        for (Key var : *root->conditional())
          result->detail->variableStatus[var].inRootClique = true;
    }

    return affectedKeysSet;
  }
};

}  // namespace gtsam
