/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ISAM2-inl.h
 * @brief   Incremental update functionality (ISAM2) for BayesTree, with fluid relinearization.
 * @author  Michael Kaess, Richard Roberts
 */

#pragma once

#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <gtsam/base/timing.h>
#include <gtsam/base/debug.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/inference/BayesTree-inl.h>
#include <gtsam/linear/HessianFactor.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2-impl-inl.h>


namespace gtsam {

using namespace std;

static const bool disableReordering = false;
static const double batchThreshold = 0.65;
static const bool latestLast = true;
static const bool structuralLast = false;

/* ************************************************************************* */
template<class CONDITIONAL, class VALUES, class GRAPH>
ISAM2<CONDITIONAL, VALUES, GRAPH>::ISAM2(const ISAM2Params& params):
    delta_(Permutation(), deltaUnpermuted_), params_(params) {}

/* ************************************************************************* */
template<class CONDITIONAL, class VALUES, class GRAPH>
ISAM2<CONDITIONAL, VALUES, GRAPH>::ISAM2():
    delta_(Permutation(), deltaUnpermuted_) {}

/* ************************************************************************* */
template<class CONDITIONAL, class VALUES, class GRAPH>
FastList<size_t> ISAM2<CONDITIONAL, VALUES, GRAPH>::getAffectedFactors(const FastList<Index>& keys) const {
  static const bool debug = false;
  if(debug) cout << "Getting affected factors for ";
  if(debug) { BOOST_FOREACH(const Index key, keys) { cout << key << " "; } }
  if(debug) cout << endl;

  FactorGraph<NonlinearFactor<VALUES> > allAffected;
  FastList<size_t> indices;
  BOOST_FOREACH(const Index key, keys) {
//    const list<size_t> l = nonlinearFactors_.factors(key);
//    indices.insert(indices.begin(), l.begin(), l.end());
    const VariableIndex::Factors& factors(variableIndex_[key]);
    BOOST_FOREACH(size_t factor, factors) {
      if(debug) cout << "Variable " << key << " affects factor " << factor << endl;
      indices.push_back(factor);
    }
  }
  indices.sort();
  indices.unique();
  if(debug) cout << "Affected factors are: ";
  if(debug) { BOOST_FOREACH(const size_t index, indices) { cout << index << " "; } }
  if(debug) cout << endl;
  return indices;
}

/* ************************************************************************* */
// retrieve all factors that ONLY contain the affected variables
// (note that the remaining stuff is summarized in the cached factors)
template<class CONDITIONAL, class VALUES, class GRAPH>
FactorGraph<GaussianFactor>::shared_ptr
ISAM2<CONDITIONAL, VALUES, GRAPH>::relinearizeAffectedFactors(const FastList<Index>& affectedKeys) const {

  tic(1,"getAffectedFactors");
  FastList<size_t> candidates = getAffectedFactors(affectedKeys);
  toc(1,"getAffectedFactors");

  GRAPH nonlinearAffectedFactors;

  tic(2,"affectedKeysSet");
  // for fast lookup below
  FastSet<Index> affectedKeysSet;
  affectedKeysSet.insert(affectedKeys.begin(), affectedKeys.end());
  toc(2,"affectedKeysSet");

  tic(3,"check candidates");
  BOOST_FOREACH(size_t idx, candidates) {
    bool inside = true;
    BOOST_FOREACH(const Symbol& key, nonlinearFactors_[idx]->keys()) {
      Index var = ordering_[key];
      if (affectedKeysSet.find(var) == affectedKeysSet.end()) {
        inside = false;
        break;
      }
    }
    if (inside)
      nonlinearAffectedFactors.push_back(nonlinearFactors_[idx]);
  }
  toc(3,"check candidates");

  tic(4,"linearize");
  FactorGraph<GaussianFactor>::shared_ptr linearized(nonlinearAffectedFactors.linearize(theta_, ordering_));
  toc(4,"linearize");
  return linearized;
}

/* ************************************************************************* */
// find intermediate (linearized) factors from cache that are passed into the affected area
template<class CONDITIONAL, class VALUES, class GRAPH>
FactorGraph<typename ISAM2<CONDITIONAL, VALUES, GRAPH>::CacheFactor>
ISAM2<CONDITIONAL, VALUES, GRAPH>::getCachedBoundaryFactors(Cliques& orphans) {

  static const bool debug = false;

  FactorGraph<CacheFactor> cachedBoundary;

  BOOST_FOREACH(sharedClique orphan, orphans) {
    // find the last variable that was eliminated
    Index key = (*orphan)->frontals().back();
#ifndef NDEBUG
//    typename BayesNet<CONDITIONAL>::const_iterator it = orphan->end();
//    const CONDITIONAL& lastCONDITIONAL = **(--it);
//    typename CONDITIONAL::const_iterator keyit = lastCONDITIONAL.endParents();
//    const Index lastKey = *(--keyit);
//    assert(key == lastKey);
#endif
    // retrieve the cached factor and add to boundary
    cachedBoundary.push_back(boost::dynamic_pointer_cast<CacheFactor>(orphan->cachedFactor()));
    if(debug) { cout << "Cached factor for variable " << key; orphan->cachedFactor()->print(""); }
  }

  return cachedBoundary;
}

template<class CONDITIONAL, class VALUES, class GRAPH>
boost::shared_ptr<FastSet<Index> > ISAM2<CONDITIONAL, VALUES, GRAPH>::recalculate(
    const FastSet<Index>& markedKeys, const FastSet<Index>& structuralKeys, const FastVector<Index>& newKeys, const FactorGraph<GaussianFactor>::shared_ptr newFactors, ISAM2Result& result) {

  // TODO:  new factors are linearized twice, the newFactors passed in are not used.

  static const bool debug = ISDEBUG("ISAM2 recalculate");

  // Input: BayesTree(this), newFactors

//#define PRINT_STATS // figures for paper, disable for timing
#ifdef PRINT_STATS
  static int counter = 0;
  int maxClique = 0;
  double avgClique = 0;
  int numCliques = 0;
  int nnzR = 0;
  if (counter>0) { // cannot call on empty tree
    GaussianISAM2_P::CliqueData cdata =  this->getCliqueData();
    GaussianISAM2_P::CliqueStats cstats = cdata.getStats();
    maxClique = cstats.maxCONDITIONALSize;
    avgClique = cstats.avgCONDITIONALSize;
    numCliques = cdata.conditionalSizes.size();
    nnzR = calculate_nnz(this->root());
  }
  counter++;
#endif

  FastSet<Index> affectedStructuralKeys;
  if(structuralLast) {
    tic(0, "affectedStructuralKeys");
    affectedStructuralKeys.insert(structuralKeys.begin(), structuralKeys.end());
    // For each structural variable, collect the variables up the path to the root,
    // which will be constrained to the back of the ordering.
    BOOST_FOREACH(Index key, structuralKeys) {
      sharedClique clique = this->nodes_[key];
      while(clique) {
        affectedStructuralKeys.insert((*clique)->beginFrontals(), (*clique)->endFrontals());
        clique = clique->parent_.lock();
      }
    }
    toc(0, "affectedStructuralKeys");
  }

  if(debug) {
    cout << "markedKeys: ";
    BOOST_FOREACH(const Index key, markedKeys) { cout << key << " "; }
    cout << endl;
    cout << "newKeys: ";
    BOOST_FOREACH(const Index key, newKeys) { cout << key << " "; }
    cout << endl;
    cout << "structuralKeys: ";
    BOOST_FOREACH(const Index key, structuralKeys) { cout << key << " "; }
    cout << endl;
    cout << "affectedStructuralKeys: ";
    BOOST_FOREACH(const Index key, affectedStructuralKeys) { cout << key << " "; }
    cout << endl;
  }

  // 1. Remove top of Bayes tree and convert to a factor graph:
  // (a) For each affected variable, remove the corresponding clique and all parents up to the root.
  // (b) Store orphaned sub-trees \BayesTree_{O} of removed cliques.
  tic(1, "removetop");
  Cliques orphans;
  BayesNet<GaussianConditional> affectedBayesNet;
  this->removeTop(markedKeys, affectedBayesNet, orphans);
  toc(1, "removetop");

  if(debug) affectedBayesNet.print("Removed top: ");
  if(debug) orphans.print("Orphans: ");

  //    FactorGraph<GaussianFactor> factors(affectedBayesNet);
  // bug was here: we cannot reuse the original factors, because then the cached factors get messed up
  // [all the necessary data is actually contained in the affectedBayesNet, including what was passed in from the boundaries,
  //  so this would be correct; however, in the process we also generate new cached_ entries that will be wrong (ie. they don't
  //  contain what would be passed up at a certain point if batch elimination was done, but that's what we need); we could choose
  //  not to update cached_ from here, but then the new information (and potentially different variable ordering) is not reflected
  //  in the cached_ values which again will be wrong]
  // so instead we have to retrieve the original linearized factors AND add the cached factors from the boundary

  // BEGIN OF COPIED CODE

  // ordering provides all keys in conditionals, there cannot be others because path to root included
  tic(2,"affectedKeys");
  FastList<Index> affectedKeys = affectedBayesNet.ordering();
  toc(2,"affectedKeys");

  if(affectedKeys.size() >= theta_.size() * batchThreshold) {

    tic(3,"batch");

    tic(0,"add keys");
    boost::shared_ptr<FastSet<Index> > affectedKeysSet(new FastSet<Index>());
    BOOST_FOREACH(const Ordering::value_type& key_index, ordering_) { affectedKeysSet->insert(key_index.second); }
    toc(0,"add keys");

    tic(1,"reorder");
    tic(1,"CCOLAMD");
    // Do a batch step - reorder and relinearize all variables
    vector<int> cmember(theta_.size(), 0);
    if(structuralLast) {
      if(theta_.size() > affectedStructuralKeys.size()) {
        BOOST_FOREACH(Index var, affectedStructuralKeys) { cmember[var] = 1; }
        if(latestLast) { BOOST_FOREACH(Index var, newKeys) { cmember[var] = 2; } }
      }
    } else if(latestLast) {
      FastSet<Index> newKeysSet(newKeys.begin(), newKeys.end());
      if(theta_.size() > newKeysSet.size()) {
        BOOST_FOREACH(Index var, newKeys) { cmember[var] = 1; }
      }
    }
    Permutation::shared_ptr colamd(Inference::PermutationCOLAMD_(variableIndex_, cmember));
    Permutation::shared_ptr colamdInverse(colamd->inverse());
    toc(1,"CCOLAMD");

    // Reorder
    tic(2,"permute global variable index");
    variableIndex_.permute(*colamd);
    toc(2,"permute global variable index");
    tic(3,"permute delta");
    delta_.permute(*colamd);
    toc(3,"permute delta");
    tic(4,"permute ordering");
    ordering_.permuteWithInverse(*colamdInverse);
    toc(4,"permute ordering");
    toc(1,"reorder");

    tic(2,"linearize");
    GaussianFactorGraph factors(*nonlinearFactors_.linearize(theta_, ordering_));
    toc(2,"linearize");

    tic(5,"eliminate");
    GaussianJunctionTree jt(factors, variableIndex_);
    sharedClique newRoot = jt.eliminate(EliminatePreferLDL, true);
    if(debug) newRoot->print("Eliminated: ");
    toc(5,"eliminate");

    tic(6,"insert");
    BayesTree<CONDITIONAL>::clear();
    this->insert(newRoot);
    toc(6,"insert");

    toc(3,"batch");

    result.variablesReeliminated = affectedKeysSet->size();

    lastAffectedMarkedCount = markedKeys.size();
    lastAffectedVariableCount = affectedKeysSet->size();
    lastAffectedFactorCount = factors.size();

    return affectedKeysSet;

  } else {

    tic(4,"incremental");

    // 2. Add the new factors \Factors' into the resulting factor graph
    FastList<Index> affectedAndNewKeys;
    affectedAndNewKeys.insert(affectedAndNewKeys.end(), affectedKeys.begin(), affectedKeys.end());
    affectedAndNewKeys.insert(affectedAndNewKeys.end(), newKeys.begin(), newKeys.end());
    tic(1,"relinearizeAffected");
    GaussianFactorGraph factors(*relinearizeAffectedFactors(affectedAndNewKeys));
    toc(1,"relinearizeAffected");

    if(debug) { cout << "Affected keys: "; BOOST_FOREACH(const Index key, affectedKeys) { cout << key << " "; } cout << endl; }

    result.variablesReeliminated = affectedAndNewKeys.size();
    lastAffectedMarkedCount = markedKeys.size();
    lastAffectedVariableCount = affectedKeys.size();
    lastAffectedFactorCount = factors.size();

#ifdef PRINT_STATS
    // output for generating figures
    cout << "linear: #markedKeys: " << markedKeys.size() << " #affectedVariables: " << affectedKeys.size()
              << " #affectedFactors: " << factors.size() << " maxCliqueSize: " << maxClique
              << " avgCliqueSize: " << avgClique << " #Cliques: " << numCliques << " nnzR: " << nnzR << endl;
#endif

    tic(2,"cached");
    // add the cached intermediate results from the boundary of the orphans ...
    FactorGraph<CacheFactor> cachedBoundary = getCachedBoundaryFactors(orphans);
    if(debug) cachedBoundary.print("Boundary factors: ");
    factors.reserve(factors.size() + cachedBoundary.size());
    // Copy so that we can later permute factors
    BOOST_FOREACH(const CacheFactor::shared_ptr& cached, cachedBoundary) {
      factors.push_back(GaussianFactor::shared_ptr(new CacheFactor(*cached)));
    }
    toc(2,"cached");

    // END OF COPIED CODE

    // 3. Re-order and eliminate the factor graph into a Bayes net (Algorithm [alg:eliminate]), and re-assemble into a new Bayes tree (Algorithm [alg:BayesTree])

    tic(4,"reorder and eliminate");

    tic(1,"list to set");
    // create a partial reordering for the new and contaminated factors
    // markedKeys are passed in: those variables will be forced to the end in the ordering
    boost::shared_ptr<FastSet<Index> > affectedKeysSet(new FastSet<Index>(markedKeys));
    affectedKeysSet->insert(affectedKeys.begin(), affectedKeys.end());
    toc(1,"list to set");

    tic(2,"PartialSolve");
    typename Impl::ReorderingMode reorderingMode;
    reorderingMode.nFullSystemVars = ordering_.nVars();
    reorderingMode.algorithm = Impl::ReorderingMode::COLAMD;
    reorderingMode.constrain = Impl::ReorderingMode::LATEST_LAST;
    reorderingMode.latestKeys = FastSet<Index>(newKeys.begin(), newKeys.end());
    typename Impl::PartialSolveResult partialSolveResult =
        Impl::PartialSolve(factors, *affectedKeysSet, reorderingMode);
    toc(2,"PartialSolve");

    // We now need to permute everything according this partial reordering: the
    // delta vector, the global ordering, and the factors we're about to
    // re-eliminate.  The reordered variables are also mentioned in the
    // orphans and the leftover cached factors.
    tic(3,"permute global variable index");
    variableIndex_.permute(partialSolveResult.fullReordering);
    toc(3,"permute global variable index");
    tic(4,"permute delta");
    delta_.permute(partialSolveResult.fullReordering);
    toc(4,"permute delta");
    tic(5,"permute ordering");
    ordering_.permuteWithInverse(partialSolveResult.fullReorderingInverse);
    toc(5,"permute ordering");

    toc(4,"reorder and eliminate");

    tic(6,"re-assemble");
    if(partialSolveResult.bayesTree) {
      assert(!this->root_);
      this->insert(partialSolveResult.bayesTree);
    }
    toc(6,"re-assemble");

    // 4. Insert the orphans back into the new Bayes tree.
    tic(7,"orphans");
    tic(1,"permute");
    BOOST_FOREACH(sharedClique orphan, orphans) {
      (void)orphan->permuteSeparatorWithInverse(partialSolveResult.fullReorderingInverse);
    }
    toc(1,"permute");
    tic(2,"insert");
    // add orphans to the bottom of the new tree
    BOOST_FOREACH(sharedClique orphan, orphans) {
      // Because the affectedKeysSelector is sorted, the orphan separator keys
      // will be sorted correctly according to the new elimination order after
      // applying the permutation, so findParentClique, which looks for the
      // lowest-ordered parent, will still work.
      Index parentRepresentative = Base::findParentClique((*orphan)->parents());
      sharedClique parent = (*this)[parentRepresentative];
      parent->children_ += orphan;
      orphan->parent_ = parent; // set new parent!
    }
    toc(2,"insert");
    toc(7,"orphans");

    toc(4,"incremental");

    return affectedKeysSet;
  }
}

/* ************************************************************************* */
template<class CONDITIONAL, class VALUES, class GRAPH>
ISAM2Result ISAM2<CONDITIONAL, VALUES, GRAPH>::update(
    const GRAPH& newFactors, const Values& newTheta, bool force_relinearize) {

  static const bool debug = ISDEBUG("ISAM2 update");
  static const bool verbose = ISDEBUG("ISAM2 update verbose");

  static int count = 0;
  count++;

  lastAffectedVariableCount = 0;
  lastAffectedFactorCount = 0;
  lastAffectedCliqueCount = 0;
  lastAffectedMarkedCount = 0;
  lastBacksubVariableCount = 0;
  lastNnzTop = 0;
  ISAM2Result result;

  if(verbose) {
    cout << "ISAM2::update\n";
    this->print("ISAM2: ");
  }

  tic(0,"push_back factors");
  // 1. Add any new factors \Factors:=\Factors\cup\Factors'.
  if(debug || verbose) newFactors.print("The new factors are: ");
  nonlinearFactors_.push_back(newFactors);
  toc(0,"push_back factors");

  tic(1,"add new variables");
  // 2. Initialize any new variables \Theta_{new} and add \Theta:=\Theta\cup\Theta_{new}.
  Impl::AddVariables(newTheta, theta_, delta_, ordering_, Base::nodes_);
  toc(1,"add new variables");

  tic(2,"evaluate error before");
  if(params_.evaluateNonlinearError)
    result.errorBefore.reset(nonlinearFactors_.error(calculateEstimate()));
  toc(2,"evaluate error before");

  tic(3,"gather involved keys");
  // 3. Mark linear update
  FastSet<Index> markedKeys = Impl::IndicesFromFactors(ordering_, newFactors); // Get keys from new factors
  FastVector<Index> newKeys(markedKeys.begin(), markedKeys.end());             // Make a copy of these, as we'll soon add to them
  FastSet<Index> structuralKeys;
  if(structuralLast) structuralKeys = markedKeys;                              // If we're using structural-last ordering, make another copy
  toc(3,"gather involved keys");

  // Check relinearization if we're at the nth step, or we are using a looser loop relin threshold
  if (force_relinearize || (params_.enableRelinearization && count % params_.relinearizeSkip == 0)) { // todo: every n steps
    tic(4,"gather relinearize keys");
    vector<bool> markedRelinMask(ordering_.nVars(), false);
    // 4. Mark keys in \Delta above threshold \beta: J=\{\Delta_{j}\in\Delta|\Delta_{j}\geq\beta\}.
    FastSet<Index> relinKeys = Impl::CheckRelinearization(delta_, params_.relinearizeThreshold);
    if(disableReordering) relinKeys = Impl::CheckRelinearization(delta_, 0.0); // This is used for debugging

    // Add the variables being relinearized to the marked keys
    BOOST_FOREACH(const Index j, relinKeys) { markedRelinMask[j] = true; }
    markedKeys.insert(relinKeys.begin(), relinKeys.end());
    toc(4,"gather relinearize keys");

    tic(5,"fluid find_all");
    // 5. Mark all cliques that involve marked variables \Theta_{J} and all their ancestors.
    if (!relinKeys.empty() && this->root())
      Impl::FindAll(this->root(), markedKeys, markedRelinMask); // add other cliques that have the marked ones in the separator
    toc(5,"fluid find_all");

    tic(6,"expmap");
    // 6. Update linearization point for marked variables: \Theta_{J}:=\Theta_{J}+\Delta_{J}.
    if (!relinKeys.empty())
      Impl::ExpmapMasked(theta_, delta_, ordering_, markedRelinMask, delta_);
    toc(6,"expmap");

    result.variablesRelinearized = markedKeys.size();

#ifndef NDEBUG
    lastRelinVariables_ = markedRelinMask;
#endif
  } else {
    result.variablesRelinearized = 0;
#ifndef NDEBUG
    lastRelinVariables_ = vector<bool>(ordering_.nVars(), false);
#endif
  }

  tic(7,"linearize new");
  tic(1,"linearize");
  // 7. Linearize new factors
  FactorGraph<GaussianFactor>::shared_ptr linearFactors = newFactors.linearize(theta_, ordering_);
  toc(1,"linearize");

  tic(2,"augment VI");
  // Augment the variable index with the new factors
  variableIndex_.augment(*linearFactors);
  toc(2,"augment VI");
  toc(7,"linearize new");

  tic(8,"recalculate");
  // 8. Redo top of Bayes tree
  boost::shared_ptr<FastSet<Index> > replacedKeys;
  if(!markedKeys.empty() || !newKeys.empty())
    replacedKeys = recalculate(markedKeys, structuralKeys, newKeys, linearFactors, result);
  toc(8,"recalculate");

  tic(9,"solve");
  // 9. Solve
  if (params_.wildfireThreshold <= 0.0 || disableReordering) {
    VectorValues newDelta(theta_.dims(ordering_));
    optimize2(this->root(), newDelta);
    if(debug) newDelta.print("newDelta: ");
    assert(newDelta.size() == delta_.size());
    delta_.permutation() = Permutation::Identity(delta_.size());
    delta_.container() = newDelta;
    lastBacksubVariableCount = theta_.size();
  } else {
    vector<bool> replacedKeysMask(variableIndex_.size(), false);
    if(replacedKeys) {
      BOOST_FOREACH(const Index var, *replacedKeys) {
        replacedKeysMask[var] = true; } }
    lastBacksubVariableCount = optimize2(this->root(), params_.wildfireThreshold, replacedKeysMask, delta_); // modifies delta_

#ifndef NDEBUG
    for(size_t j=0; j<delta_.container().size(); ++j)
      assert(delta_.container()[j].unaryExpr(&isfinite<double>).all());
#endif
  }
  toc(9,"solve");

  tic(10,"evaluate error after");
  if(params_.evaluateNonlinearError)
    result.errorAfter.reset(nonlinearFactors_.error(calculateEstimate()));
  toc(10,"evaluate error after");

  return result;
}

/* ************************************************************************* */
template<class CONDITIONAL, class VALUES, class GRAPH>
VALUES ISAM2<CONDITIONAL, VALUES, GRAPH>::calculateEstimate() const {
  // We use ExpmapMasked here instead of regular expmap because the former
  // handles Permuted<VectorValues>
	VALUES ret(theta_);
  vector<bool> mask(ordering_.nVars(), true);
  Impl::ExpmapMasked(ret, delta_, ordering_, mask);
  return ret;
}

/* ************************************************************************* */
template<class CONDITIONAL, class VALUES, class GRAPH>
template<class KEY>
typename KEY::Value ISAM2<CONDITIONAL, VALUES, GRAPH>::calculateEstimate(const KEY& key) const {
  const Index index = getOrdering()[key];
  const SubVector delta = getDelta()[index];
  return getLinearizationPoint()[key].retract(delta);
}

/* ************************************************************************* */
template<class CONDITIONAL, class VALUES, class GRAPH>
VALUES ISAM2<CONDITIONAL, VALUES, GRAPH>::calculateBestEstimate() const {
  VectorValues delta(theta_.dims(ordering_));
  optimize2(this->root(), delta);
  return theta_.retract(delta, ordering_);
}

}
/// namespace gtsam
