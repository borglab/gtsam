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

#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <gtsam/base/timing.h>
#include <gtsam/base/debug.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/inference/BayesTree-inl.h>
#include <gtsam/linear/HessianFactor.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/DoglegOptimizerImpl.h>


namespace gtsam {

using namespace std;

static const bool disableReordering = false;
static const double batchThreshold = 0.65;

/* ************************************************************************* */
ISAM2::ISAM2(const ISAM2Params& params):
    delta_(deltaUnpermuted_), deltaNewton_(deltaNewtonUnpermuted_), RgProd_(RgProdUnpermuted_),
    deltaDoglegUptodate_(true), deltaUptodate_(true), params_(params) {
  // See note in gtsam/base/boost_variant_with_workaround.h
  if(params_.optimizationParams.type() == typeid(ISAM2DoglegParams))
    doglegDelta_ = boost::get<ISAM2DoglegParams>(params_.optimizationParams).initialDelta;
}

/* ************************************************************************* */
ISAM2::ISAM2():
    delta_(deltaUnpermuted_), deltaNewton_(deltaNewtonUnpermuted_), RgProd_(RgProdUnpermuted_),
    deltaDoglegUptodate_(true), deltaUptodate_(true) {
  // See note in gtsam/base/boost_variant_with_workaround.h
  if(params_.optimizationParams.type() == typeid(ISAM2DoglegParams))
    doglegDelta_ = boost::get<ISAM2DoglegParams>(params_.optimizationParams).initialDelta;
}

/* ************************************************************************* */
FastList<size_t> ISAM2::getAffectedFactors(const FastList<Index>& keys) const {
  static const bool debug = false;
  if(debug) cout << "Getting affected factors for ";
  if(debug) { BOOST_FOREACH(const Index key, keys) { cout << key << " "; } }
  if(debug) cout << endl;

  FactorGraph<NonlinearFactor > allAffected;
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
FactorGraph<GaussianFactor>::shared_ptr
ISAM2::relinearizeAffectedFactors(const FastList<Index>& affectedKeys) const {

  tic(1,"getAffectedFactors");
  FastList<size_t> candidates = getAffectedFactors(affectedKeys);
  toc(1,"getAffectedFactors");

  NonlinearFactorGraph nonlinearAffectedFactors;

  tic(2,"affectedKeysSet");
  // for fast lookup below
  FastSet<Index> affectedKeysSet;
  affectedKeysSet.insert(affectedKeys.begin(), affectedKeys.end());
  toc(2,"affectedKeysSet");

  tic(3,"check candidates");
  BOOST_FOREACH(size_t idx, candidates) {
    bool inside = true;
    BOOST_FOREACH(Key key, nonlinearFactors_[idx]->keys()) {
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
FactorGraph<ISAM2::CacheFactor>
ISAM2::getCachedBoundaryFactors(Cliques& orphans) {

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

boost::shared_ptr<FastSet<Index> > ISAM2::recalculate(
    const FastSet<Index>& markedKeys, const FastVector<Index>& newKeys, const FactorGraph<GaussianFactor>::shared_ptr newFactors,
    const boost::optional<FastSet<Index> >& constrainKeys, ISAM2Result& result) {

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

  if(debug) {
    cout << "markedKeys: ";
    BOOST_FOREACH(const Index key, markedKeys) { cout << key << " "; }
    cout << endl;
    cout << "newKeys: ";
    BOOST_FOREACH(const Index key, newKeys) { cout << key << " "; }
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
    FastSet<Index> constrainedKeysSet;
    if(constrainKeys)
      constrainedKeysSet = *constrainKeys;
    else
      constrainedKeysSet.insert(newKeys.begin(), newKeys.end());
    if(theta_.size() > constrainedKeysSet.size()) {
      BOOST_FOREACH(Index var, constrainedKeysSet) { cmember[var] = 1; }
    }
    Permutation::shared_ptr colamd(inference::PermutationCOLAMD_(variableIndex_, cmember));
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
    JunctionTree<GaussianFactorGraph, Base::Clique> jt(factors, variableIndex_);
    sharedClique newRoot = jt.eliminate(EliminatePreferLDL);
    if(debug) newRoot->print("Eliminated: ");
    toc(5,"eliminate");

    tic(6,"insert");
    this->clear();
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
    if(debug) factors.print("Relinearized factors: ");
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
    Impl::ReorderingMode reorderingMode;
    reorderingMode.nFullSystemVars = ordering_.nVars();
    reorderingMode.algorithm = Impl::ReorderingMode::COLAMD;
    reorderingMode.constrain = Impl::ReorderingMode::CONSTRAIN_LAST;
    if(constrainKeys)
      reorderingMode.constrainedKeys = *constrainKeys;
    else
      reorderingMode.constrainedKeys = FastSet<Index>(newKeys.begin(), newKeys.end());
    Impl::PartialSolveResult partialSolveResult =
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
    deltaNewton_.permute(partialSolveResult.fullReordering);
    RgProd_.permute(partialSolveResult.fullReordering);
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
ISAM2Result ISAM2::update(
    const NonlinearFactorGraph& newFactors, const Values& newTheta, const FastVector<size_t>& removeFactorIndices,
    const boost::optional<FastSet<Key> >& constrainedKeys, bool force_relinearize) {

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
  const bool relinearizeThisStep = force_relinearize || (params_.enableRelinearization && count % params_.relinearizeSkip == 0);

  if(verbose) {
    cout << "ISAM2::update\n";
    this->print("ISAM2: ");
  }

  // Update delta if we need it to check relinearization later
  if(relinearizeThisStep) {
    tic(0, "updateDelta");
    updateDelta(disableReordering);
    toc(0, "updateDelta");
  }

  tic(1,"push_back factors");
  // Add the new factor indices to the result struct
  result.newFactorsIndices.resize(newFactors.size());
  for(size_t i=0; i<newFactors.size(); ++i)
    result.newFactorsIndices[i] = i + nonlinearFactors_.size();

  // 1. Add any new factors \Factors:=\Factors\cup\Factors'.
  if(debug || verbose) newFactors.print("The new factors are: ");
  nonlinearFactors_.push_back(newFactors);

  // Remove the removed factors
  NonlinearFactorGraph removeFactors; removeFactors.reserve(removeFactorIndices.size());
  BOOST_FOREACH(size_t index, removeFactorIndices) {
    removeFactors.push_back(nonlinearFactors_[index]);
    nonlinearFactors_.remove(index);
  }

  // Remove removed factors from the variable index so we do not attempt to relinearize them
  variableIndex_.remove(removeFactorIndices, *removeFactors.symbolic(ordering_));
  toc(1,"push_back factors");

  tic(2,"add new variables");
  // 2. Initialize any new variables \Theta_{new} and add \Theta:=\Theta\cup\Theta_{new}.
  Impl::AddVariables(newTheta, theta_, delta_, deltaNewton_, RgProd_, deltaReplacedMask_, ordering_, Base::nodes_);
  toc(2,"add new variables");

  tic(3,"evaluate error before");
  if(params_.evaluateNonlinearError)
    result.errorBefore.reset(nonlinearFactors_.error(calculateEstimate()));
  toc(3,"evaluate error before");

  tic(4,"gather involved keys");
  // 3. Mark linear update
  FastSet<Index> markedKeys = Impl::IndicesFromFactors(ordering_, newFactors); // Get keys from new factors
  // Also mark keys involved in removed factors
  {
    FastSet<Index> markedRemoveKeys = Impl::IndicesFromFactors(ordering_, removeFactors); // Get keys involved in removed factors
    markedKeys.insert(markedRemoveKeys.begin(), markedRemoveKeys.end()); // Add to the overall set of marked keys
  }
  // NOTE: we use assign instead of the iterator constructor here because this
  // is a vector of size_t, so the constructor unintentionally resolves to
  // vector(size_t count, Index value) instead of the iterator constructor.
  FastVector<Index> newKeys; newKeys.assign(markedKeys.begin(), markedKeys.end());             // Make a copy of these, as we'll soon add to them
  toc(4,"gather involved keys");

  // Check relinearization if we're at the nth step, or we are using a looser loop relin threshold
  if (relinearizeThisStep) {
    tic(5,"gather relinearize keys");
    vector<bool> markedRelinMask(ordering_.nVars(), false);
    // 4. Mark keys in \Delta above threshold \beta: J=\{\Delta_{j}\in\Delta|\Delta_{j}\geq\beta\}.
    FastSet<Index> relinKeys = Impl::CheckRelinearization(delta_, ordering_, params_.relinearizeThreshold);
    if(disableReordering) relinKeys = Impl::CheckRelinearization(delta_, ordering_, 0.0); // This is used for debugging

    // Add the variables being relinearized to the marked keys
    BOOST_FOREACH(const Index j, relinKeys) { markedRelinMask[j] = true; }
    markedKeys.insert(relinKeys.begin(), relinKeys.end());
    toc(5,"gather relinearize keys");

    tic(6,"fluid find_all");
    // 5. Mark all cliques that involve marked variables \Theta_{J} and all their ancestors.
    if (!relinKeys.empty() && this->root())
      Impl::FindAll(this->root(), markedKeys, markedRelinMask); // add other cliques that have the marked ones in the separator
    toc(6,"fluid find_all");

    tic(7,"expmap");
    // 6. Update linearization point for marked variables: \Theta_{J}:=\Theta_{J}+\Delta_{J}.
    if (!relinKeys.empty())
      Impl::ExpmapMasked(theta_, delta_, ordering_, markedRelinMask, delta_);
    toc(7,"expmap");

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

  tic(8,"linearize new");
  tic(1,"linearize");
  // 7. Linearize new factors
  FactorGraph<GaussianFactor>::shared_ptr linearFactors = newFactors.linearize(theta_, ordering_);
  toc(1,"linearize");

  tic(2,"augment VI");
  // Augment the variable index with the new factors
  variableIndex_.augment(*linearFactors);
  toc(2,"augment VI");
  toc(8,"linearize new");

  tic(9,"recalculate");
  // 8. Redo top of Bayes tree
  // Convert constrained symbols to indices
  boost::optional<FastSet<Index> > constrainedIndices;
  if(constrainedKeys) {
    constrainedIndices.reset(FastSet<Index>());
    BOOST_FOREACH(Key key, *constrainedKeys) {
      constrainedIndices->insert(ordering_[key]);
    }
  }
  boost::shared_ptr<FastSet<Index> > replacedKeys;
  if(!markedKeys.empty() || !newKeys.empty())
    replacedKeys = recalculate(markedKeys, newKeys, linearFactors, constrainedIndices, result);

  // Update replaced keys mask (accumulates until back-substitution takes place)
  if(replacedKeys) {
    BOOST_FOREACH(const Index var, *replacedKeys) {
      deltaReplacedMask_[var] = true; } }
  toc(9,"recalculate");

  //tic(9,"solve");
  // 9. Solve
  if(debug) delta_.print("delta_: ");
  //toc(9,"solve");

  tic(10,"evaluate error after");
  if(params_.evaluateNonlinearError)
    result.errorAfter.reset(nonlinearFactors_.error(calculateEstimate()));
  toc(10,"evaluate error after");

  result.cliques = this->nodes().size();
  deltaDoglegUptodate_ = false;
  deltaUptodate_ = false;

  return result;
}

/* ************************************************************************* */
void ISAM2::updateDelta(bool forceFullSolve) const {

  if(params_.optimizationParams.type() == typeid(ISAM2GaussNewtonParams)) {
    // If using Gauss-Newton, update with wildfireThreshold
    const ISAM2GaussNewtonParams& gaussNewtonParams =
        boost::get<ISAM2GaussNewtonParams>(params_.optimizationParams);
    const double effectiveWildfireThreshold = forceFullSolve ? 0.0 : gaussNewtonParams.wildfireThreshold;
    tic(0, "Wildfire update");
    lastBacksubVariableCount = Impl::UpdateDelta(this->root(), deltaReplacedMask_, delta_, effectiveWildfireThreshold);
    toc(0, "Wildfire update");

  } else if(params_.optimizationParams.type() == typeid(ISAM2DoglegParams)) {
    // If using Dogleg, do a Dogleg step
    const ISAM2DoglegParams& doglegParams =
        boost::get<ISAM2DoglegParams>(params_.optimizationParams);

    // Do one Dogleg iteration
    tic(1, "Dogleg Iterate");
    DoglegOptimizerImpl::IterationResult doglegResult = DoglegOptimizerImpl::Iterate(
        *doglegDelta_, doglegParams.adaptationMode, *this, nonlinearFactors_, theta_, ordering_, nonlinearFactors_.error(theta_), doglegParams.verbose);
    toc(1, "Dogleg Iterate");

    // Update Delta and linear step
    doglegDelta_ = doglegResult.Delta;
    delta_.permutation() = Permutation::Identity(delta_.size());  // Dogleg solves for the full delta so there is no permutation
    delta_.container() = doglegResult.dx_d; // Copy the VectorValues containing with the linear solution
  }

  deltaUptodate_ = true;
}

/* ************************************************************************* */
Values ISAM2::calculateEstimate() const {
  // We use ExpmapMasked here instead of regular expmap because the former
  // handles Permuted<VectorValues>
	Values ret(theta_);
  vector<bool> mask(ordering_.nVars(), true);
  Impl::ExpmapMasked(ret, getDelta(), ordering_, mask);
  return ret;
}

/* ************************************************************************* */
Values ISAM2::calculateBestEstimate() const {
  VectorValues delta(theta_.dims(ordering_));
  internal::optimizeInPlace<Base>(this->root(), delta);
  return theta_.retract(delta, ordering_);
}

/* ************************************************************************* */
const Permuted<VectorValues>& ISAM2::getDelta() const {
  if(!deltaUptodate_)
    updateDelta();
  return delta_;
}

/* ************************************************************************* */
VectorValues optimize(const ISAM2& isam) {
  tic(0, "allocateVectorValues");
  VectorValues delta = *allocateVectorValues(isam);
  toc(0, "allocateVectorValues");
  optimizeInPlace(isam, delta);
  return delta;
}

/* ************************************************************************* */
void optimizeInPlace(const ISAM2& isam, VectorValues& delta) {
  // We may need to update the solution calcaulations
  if(!isam.deltaDoglegUptodate_) {
    tic(1, "UpdateDoglegDeltas");
    ISAM2::Impl::UpdateDoglegDeltas(isam, isam.deltaReplacedMask_, isam.deltaNewton_, isam.RgProd_);
    isam.deltaDoglegUptodate_ = true;
    toc(1, "UpdateDoglegDeltas");
  }

  tic(2, "copy delta");
  delta = isam.deltaNewton_;
  toc(2, "copy delta");
}

/* ************************************************************************* */
VectorValues optimizeGradientSearch(const ISAM2& isam) {
  tic(0, "Allocate VectorValues");
  VectorValues grad = *allocateVectorValues(isam);
  toc(0, "Allocate VectorValues");

  optimizeGradientSearchInPlace(isam, grad);

  return grad;
}

/* ************************************************************************* */
void optimizeGradientSearchInPlace(const ISAM2& isam, VectorValues& grad) {
  // We may need to update the solution calcaulations
  if(!isam.deltaDoglegUptodate_) {
    tic(1, "UpdateDoglegDeltas");
    ISAM2::Impl::UpdateDoglegDeltas(isam, isam.deltaReplacedMask_, isam.deltaNewton_, isam.RgProd_);
    isam.deltaDoglegUptodate_ = true;
    toc(1, "UpdateDoglegDeltas");
  }

  tic(1, "Compute Gradient");
  // Compute gradient (call gradientAtZero function, which is defined for various linear systems)
  gradientAtZero(isam, grad);
  double gradientSqNorm = grad.dot(grad);
  toc(1, "Compute Gradient");

  tic(2, "Compute minimizing step size");
  // Compute minimizing step size
  double RgNormSq = isam.RgProd_.container().vector().squaredNorm();
  double step = -gradientSqNorm / RgNormSq;
  toc(2, "Compute minimizing step size");

  tic(4, "Compute point");
  // Compute steepest descent point
  grad.vector() *= step;
  toc(4, "Compute point");
}

/* ************************************************************************* */
VectorValues gradient(const ISAM2& bayesTree, const VectorValues& x0) {
  return gradient(FactorGraph<JacobianFactor>(bayesTree), x0);
}

/* ************************************************************************* */
static void gradientAtZeroTreeAdder(const boost::shared_ptr<ISAM2Clique>& root, VectorValues& g) {
  // Loop through variables in each clique, adding contributions
  int variablePosition = 0;
  for(GaussianConditional::const_iterator jit = root->conditional()->begin(); jit != root->conditional()->end(); ++jit) {
    const int dim = root->conditional()->dim(jit);
    g[*jit] += root->gradientContribution().segment(variablePosition, dim);
    variablePosition += dim;
  }

  // Recursively add contributions from children
  typedef boost::shared_ptr<ISAM2Clique> sharedClique;
  BOOST_FOREACH(const sharedClique& child, root->children()) {
    gradientAtZeroTreeAdder(child, g);
  }
}

/* ************************************************************************* */
void gradientAtZero(const ISAM2& bayesTree, VectorValues& g) {
  // Zero-out gradient
  g.setZero();

  // Sum up contributions for each clique
  gradientAtZeroTreeAdder(bayesTree.root(), g);
}

}
/// namespace gtsam
