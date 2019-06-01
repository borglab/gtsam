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

#include <gtsam/nonlinear/ISAM2-impl.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Result.h>

#include <gtsam/base/debug.h>
#include <gtsam/base/timing.h>
#include <gtsam/inference/BayesTree-inst.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

#include <map>

using namespace std;

namespace gtsam {

// Instantiate base class
template class BayesTree<ISAM2Clique>;

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
void ISAM2::addVariables(
    const Values& newTheta,
    ISAM2Result::DetailedResults::StatusMap* variableStatus) {
  gttic(addNewVariables);
  // \Theta:=\Theta\cup\Theta_{new}.
  const bool debug = ISDEBUG("ISAM2 AddVariables");

  theta_.insert(newTheta);
  if (debug) newTheta.print("The new variables are: ");
  // Add zeros into the VectorValues
  delta_.insert(newTheta.zeroVectors());
  deltaNewton_.insert(newTheta.zeroVectors());
  RgProd_.insert(newTheta.zeroVectors());

  // New keys for detailed results
  if (variableStatus && params_.enableDetailedResults) {
    for (Key key : newTheta.keys()) {
      (*variableStatus)[key].isNew = true;
    }
  }
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
ISAM2Result ISAM2::update(
    const NonlinearFactorGraph& newFactors, const Values& newTheta,
    const FactorIndices& removeFactorIndices,
    const boost::optional<FastMap<Key, int> >& constrainedKeys,
    const boost::optional<FastList<Key> >& noRelinKeys,
    const boost::optional<FastList<Key> >& extraReelimKeys,
    bool force_relinearize) {
  ISAM2UpdateParams params;
  params.constrainedKeys = constrainedKeys;
  params.extraReelimKeys = extraReelimKeys;
  params.force_relinearize = force_relinearize;
  params.noRelinKeys = noRelinKeys;
  params.removeFactorIndices = removeFactorIndices;

  return update(newFactors, newTheta, params);
}

/* ************************************************************************* */
ISAM2Result ISAM2::update(const NonlinearFactorGraph& newFactors,
                          const Values& newTheta,
                          const ISAM2UpdateParams& updateParams) {
  gttic(ISAM2_update);
  this->update_count_++;
  UpdateImpl::LogStartingUpdate(newFactors, *this);

  ISAM2Result result;
  if (params_.enableDetailedResults)
    result.detail = ISAM2Result::DetailedResults();
  const bool relinearizeThisStep =
      updateParams.force_relinearize ||
      (params_.enableRelinearization &&
       update_count_ % params_.relinearizeSkip == 0);

  // Update delta if we need it to check relinearization later
  if (relinearizeThisStep) {
    updateDelta(updateParams.forceFullSolve);
  }

  UpdateImpl update(params_, updateParams);

  // 1. Add any new factors \Factors:=\Factors\cup\Factors'.
  update.pushBackFactors(newFactors, &nonlinearFactors_, &linearFactors_,
                         &variableIndex_, &result);

  // 2. Initialize any new variables \Theta_{new} and add
  addVariables(newTheta, result.detail ? &result.detail->variableStatus : 0);

  gttic(evaluate_error_before);
  if (params_.evaluateNonlinearError)
    result.errorBefore.reset(nonlinearFactors_.error(calculateEstimate()));
  gttoc(evaluate_error_before);

  // 3. Mark linear update
  update.gatherInvolvedKeys(newFactors, nonlinearFactors_, &result);

  // Check relinearization if we're at the nth step, or we are using a looser
  // loop relin threshold
  KeySet relinKeys;
  if (relinearizeThisStep) {
    relinKeys =
        update.relinearize(roots_, delta_, fixedVariables_, &theta_, &result);
  } else {
    result.variablesRelinearized = 0;
  }

  // 7. Linearize new factors
  update.linearizeNewFactors(newFactors, theta_, nonlinearFactors_.size(),
                             result.newFactorsIndices, &linearFactors_);
  update.augmentVariableIndex(newFactors, result.newFactorsIndices,
                              &variableIndex_);

  // 8. Redo top of Bayes tree
  if (!result.markedKeys.empty() || !result.observedKeys.empty()) {
    // Remove top of Bayes tree and convert to a factor graph:
    // (a) For each affected variable, remove the corresponding clique and all
    // parents up to the root. (b) Store orphaned sub-trees \BayesTree_{O} of
    // removed cliques.
    GaussianBayesNet affectedBayesNet;
    Cliques orphans;
    this->removeTop(
        KeyVector(result.markedKeys.begin(), result.markedKeys.end()),
        affectedBayesNet, orphans);

    KeySet affectedKeysSet = update.recalculate(
        theta_, variableIndex_, nonlinearFactors_, affectedBayesNet, orphans,
        relinKeys, &linearFactors_, &roots_, &nodes_, &result);
    // Update replaced keys mask (accumulates until back-substitution takes
    // place)
    deltaReplacedMask_.insert(affectedKeysSet.begin(), affectedKeysSet.end());
  }

  // Update data structures to remove unused keys
  if (!result.unusedKeys.empty()) {
    gttic(remove_variables);
    removeVariables(result.unusedKeys);
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
  FactorIndexSet factorIndicesToRemove;

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
        for (const auto index : factorsFromMarginalizedInClique_step1) {
          graph.push_back(nonlinearFactors_[index]->linearize(theta_));
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
  for (const auto index : factorIndicesToRemove) {
    removedFactors.push_back(nonlinearFactors_[index]);
    nonlinearFactors_.remove(index);
    if (params_.cacheLinearizedFactors) linearFactors_.remove(index);
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
// Marked const but actually changes mutable delta
void ISAM2::updateDelta(bool forceFullSolve) const {
  gttic(updateDelta);
  if (params_.optimizationParams.type() == typeid(ISAM2GaussNewtonParams)) {
    // If using Gauss-Newton, update with wildfireThreshold
    const ISAM2GaussNewtonParams& gaussNewtonParams =
        boost::get<ISAM2GaussNewtonParams>(params_.optimizationParams);
    const double effectiveWildfireThreshold =
        forceFullSolve ? 0.0 : gaussNewtonParams.wildfireThreshold;
    gttic(Wildfire_update);
    DeltaImpl::UpdateGaussNewtonDelta(roots_, deltaReplacedMask_,
                                      effectiveWildfireThreshold, &delta_);
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
    DeltaImpl::UpdateGaussNewtonDelta(
        roots_, deltaReplacedMask_, effectiveWildfireThreshold, &deltaNewton_);
    gttoc(Wildfire_update);

    // Compute steepest descent step
    const VectorValues gradAtZero = this->gradientAtZero();  // Compute gradient
    DeltaImpl::UpdateRgProd(roots_, deltaReplacedMask_, gradAtZero,
                            &RgProd_);  // Update RgProd
    const VectorValues dx_u = DeltaImpl::ComputeGradientSearch(
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
  } else {
    throw std::runtime_error("iSAM2: unknown ISAM2Params type");
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
VectorValues ISAM2::gradientAtZero() const {
  // Create result
  VectorValues g;

  // Sum up contributions for each clique
  for (const auto& root : this->roots()) root->addGradientAtZero(&g);

  return g;
}

}  // namespace gtsam
