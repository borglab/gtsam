/**
 * @file    ISAM2-inl.h
 * @brief   Incremental update functionality (ISAM2) for BayesTree, with fluid relinearization.
 * @author  Michael Kaess, Richard Roberts
 */

#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <set>
#include <limits>
#include <numeric>

#include <gtsam/base/timing.h>
#include <gtsam/base/debug.h>
#include <gtsam/nonlinear/NonlinearFactorGraph-inl.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/GaussianJunctionTree.h>

#include <gtsam/inference/BayesTree-inl.h>

#include <gtsam/inference/GenericSequentialSolver-inl.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2-impl-inl.h>


namespace gtsam {

using namespace std;

static const bool disableReordering = false;
static const double batchThreshold = 0.65;
static const bool latestLast = true;
static const bool structuralLast = false;

/* ************************************************************************* */
template<class Conditional, class Values>
ISAM2<Conditional, Values>::ISAM2(const ISAM2Params& params):
    delta_(Permutation(), deltaUnpermuted_), params_(params) {}

/* ************************************************************************* */
template<class Conditional, class Values>
ISAM2<Conditional, Values>::ISAM2():
    delta_(Permutation(), deltaUnpermuted_) {}

/** Create a Bayes Tree from a nonlinear factor graph */
//template<class Conditional, class Values>
//ISAM2<Conditional, Values>::ISAM2(const NonlinearFactorGraph<Values>& nlfg, const Ordering& ordering, const Values& config) :
//BayesTree<Conditional>(nlfg.linearize(config)->eliminate(ordering)), theta_(config),
//variableIndex_(nlfg.symbolic(config, ordering), config.dims(ordering)), deltaUnpermuted_(variableIndex_.dims()),
//delta_(Permutation::Identity(variableIndex_.size())), nonlinearFactors_(nlfg), ordering_(ordering) {
//  // todo: repeats calculation above, just to set "cached"
//  // De-referencing shared pointer can be quite expensive because creates temporary
//  _eliminate_const(*nlfg.linearize(config, ordering), cached_, ordering);
//}

/* ************************************************************************* */
template<class Conditional, class Values>
FastList<size_t> ISAM2<Conditional, Values>::getAffectedFactors(const FastList<Index>& keys) const {
  static const bool debug = false;
  if(debug) cout << "Getting affected factors for ";
  if(debug) { BOOST_FOREACH(const Index key, keys) { cout << key << " "; } }
  if(debug) cout << endl;

  FactorGraph<NonlinearFactor<Values> > allAffected;
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
template<class Conditional, class Values>
FactorGraph<GaussianFactor>::shared_ptr
ISAM2<Conditional, Values>::relinearizeAffectedFactors(const FastList<Index>& affectedKeys) const {

  tic(1,"getAffectedFactors");
  FastList<size_t> candidates = getAffectedFactors(affectedKeys);
  toc(1,"getAffectedFactors");

  NonlinearFactorGraph<Values> nonlinearAffectedFactors;

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
template<class Conditional, class Values>
FactorGraph<typename ISAM2<Conditional, Values>::CacheFactor>
ISAM2<Conditional, Values>::getCachedBoundaryFactors(Cliques& orphans) {

  static const bool debug = false;

  FactorGraph<CacheFactor> cachedBoundary;

  BOOST_FOREACH(sharedClique orphan, orphans) {
    // find the last variable that was eliminated
    Index key = (*orphan)->frontals().back();
#ifndef NDEBUG
//    typename BayesNet<Conditional>::const_iterator it = orphan->end();
//    const Conditional& lastConditional = **(--it);
//    typename Conditional::const_iterator keyit = lastConditional.endParents();
//    const Index lastKey = *(--keyit);
//    assert(key == lastKey);
#endif
    // retrieve the cached factor and add to boundary
    cachedBoundary.push_back(boost::dynamic_pointer_cast<CacheFactor>(orphan->cachedFactor()));
    if(debug) { cout << "Cached factor for variable " << key; orphan->cachedFactor()->print(""); }
  }

  return cachedBoundary;
}

template<class Conditional, class Values>
boost::shared_ptr<FastSet<Index> > ISAM2<Conditional, Values>::recalculate(const FastSet<Index>& markedKeys, const FastSet<Index>& structuralKeys, const vector<Index>& newKeys, const FactorGraph<GaussianFactor>::shared_ptr newFactors) {

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
    maxClique = cstats.maxConditionalSize;
    avgClique = cstats.avgConditionalSize;
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

//  if(debug) newFactors->print("Recalculating factors: ");
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
    sharedClique newRoot = jt.eliminate(EliminateQR, true);
    if(debug) newRoot->print("Eliminated: ");
    toc(5,"eliminate");

    tic(6,"insert");
    BayesTree<Conditional>::clear();
    assert(!this->root_);
    this->insert(newRoot);
    assert(this->root_);
    toc(6,"insert");

    toc(3,"batch");

    lastAffectedMarkedCount = markedKeys.size();
    lastAffectedVariableCount = affectedKeysSet->size();
    lastAffectedFactorCount = factors.size();

    return affectedKeysSet;

  } else {

    tic(4,"incremental");

    FastList<Index> affectedAndNewKeys;
    affectedAndNewKeys.insert(affectedAndNewKeys.end(), affectedKeys.begin(), affectedKeys.end());
    affectedAndNewKeys.insert(affectedAndNewKeys.end(), newKeys.begin(), newKeys.end());
    tic(1,"relinearizeAffected");
    GaussianFactorGraph factors(*relinearizeAffectedFactors(affectedAndNewKeys));
    toc(1,"relinearizeAffected");

#ifndef NDEBUG
    // The relinearized variables should not appear anywhere in the orphans
    BOOST_FOREACH(boost::shared_ptr<const typename BayesTree<Conditional>::Clique> clique, orphans) {
      BOOST_FOREACH(const Index key, (*clique)->frontals()) {
        assert(lastRelinVariables_[key] == false);
      }
    }
#endif

    //  if(debug) factors.print("Affected factors: ");
    if(debug) { cout << "Affected keys: "; BOOST_FOREACH(const Index key, affectedKeys) { cout << key << " "; } cout << endl; }

    lastAffectedMarkedCount = markedKeys.size();
    lastAffectedVariableCount = affectedKeys.size();
    lastAffectedFactorCount = factors.size();

#ifdef PRINT_STATS
    // output for generating figures
    cout << "linear: #markedKeys: " << markedKeys.size() << " #affectedVariables: " << affectedKeys.size()
              << " #affectedFactors: " << factors.size() << " maxCliqueSize: " << maxClique
              << " avgCliqueSize: " << avgClique << " #Cliques: " << numCliques << " nnzR: " << nnzR << endl;
#endif

    //#ifndef NDEBUG
    //  for(Index var=0; var<cached_.size(); ++var) {
    //    if(find(affectedKeys.begin(), affectedKeys.end(), var) == affectedKeys.end() ||
    //        lastRelinVariables_[var] == true) {
    //      assert(!cached_[var] || find(cached_[var]->begin(), cached_[var]->end(), var) == cached_[var]->end());
    //    }
    //  }
    //#endif

    tic(2,"cached");
    // add the cached intermediate results from the boundary of the orphans ...
    FactorGraph<CacheFactor> cachedBoundary = getCachedBoundaryFactors(orphans);
    if(debug) cachedBoundary.print("Boundary factors: ");
    factors.reserve(factors.size() + cachedBoundary.size());
    // Copy so that we can later permute factors
    BOOST_FOREACH(const CacheFactor::shared_ptr& cached, cachedBoundary) {
#ifndef NDEBUG
      BOOST_FOREACH(const Index key, *cached) {
        assert(lastRelinVariables_[key] == false);
      }
#endif
      factors.push_back(GaussianFactor::shared_ptr(new CacheFactor(*cached)));
    }
    //  factors.push_back(cachedBoundary);
    toc(2,"cached");

    // END OF COPIED CODE


    // 2. Add the new factors \Factors' into the resulting factor graph
    tic(3,"newfactors");
    if (newFactors) {
#ifndef NDEBUG
      BOOST_FOREACH(const GaussianFactor::shared_ptr& newFactor, *newFactors) {
        bool found = false;
        BOOST_FOREACH(const GaussianFactor::shared_ptr& affectedFactor, factors) {
          if(newFactor->equals(*affectedFactor, 1e-6))
            found = true;
        }
        assert(found);
      }
#endif
      //factors.push_back(*newFactors);
    }
    toc(3,"newfactors");

    // 3. Re-order and eliminate the factor graph into a Bayes net (Algorithm [alg:eliminate]), and re-assemble into a new Bayes tree (Algorithm [alg:BayesTree])

    tic(4,"reorder");

    //#define PRESORT_ALPHA

    tic(1,"select affected variables");
    // create a partial reordering for the new and contaminated factors
    // markedKeys are passed in: those variables will be forced to the end in the ordering
    boost::shared_ptr<FastSet<Index> > affectedKeysSet(new FastSet<Index>(markedKeys));
    affectedKeysSet->insert(affectedKeys.begin(), affectedKeys.end());
    //#ifndef NDEBUG
    //  // All affected keys should be contiguous and at the end of the elimination order
    //  for(set<Index>::const_iterator key=affectedKeysSet->begin(); key!=affectedKeysSet->end(); ++key) {
    //    if(key != affectedKeysSet->begin()) {
    //      set<Index>::const_iterator prev = key; --prev;
    //      assert(*prev == *key - 1);
    //    }
    //  }
    //  assert(*(affectedKeysSet->end()) == variableIndex_.size() - 1);
    //#endif

#ifndef NDEBUG
    // Debug check that all variables involved in the factors to be re-eliminated
    // are in affectedKeys, since we will use it to select a subset of variables.
    BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, factors) {
      BOOST_FOREACH(Index key, factor->keys()) {
        assert(find(affectedKeysSet->begin(), affectedKeysSet->end(), key) != affectedKeysSet->end());
      }
    }
#endif
    Permutation affectedKeysSelector(affectedKeysSet->size()); // Create a permutation that pulls the affected keys to the front
    Permutation affectedKeysSelectorInverse(affectedKeysSet->size() > 0 ? *(--affectedKeysSet->end())+1 : 0 /*ordering_.nVars()*/); // And its inverse
#ifndef NDEBUG
    // If debugging, fill with invalid values that will trip asserts if dereferenced
    std::fill(affectedKeysSelectorInverse.begin(), affectedKeysSelectorInverse.end(), numeric_limits<Index>::max());
#endif
    { Index position=0; BOOST_FOREACH(Index var, *affectedKeysSet) {
      affectedKeysSelector[position] = var;
      affectedKeysSelectorInverse[var] = position;
      ++ position; } }
    //  if(disableReordering) { assert(affectedKeysSelector.equals(Permutation::Identity(ordering_.nVars()))); assert(affectedKeysSelectorInverse.equals(Permutation::Identity(ordering_.nVars()))); }
    if(debug) affectedKeysSelector.print("affectedKeysSelector: ");
    if(debug) affectedKeysSelectorInverse.print("affectedKeysSelectorInverse: ");
#ifndef NDEBUG
    VariableIndex beforePermutationIndex(factors);
#endif
    factors.permuteWithInverse(affectedKeysSelectorInverse);
    if(debug) factors.print("Factors to reorder/re-eliminate: ");
    toc(1,"select affected variables");
    tic(2,"variable index");
    VariableIndex affectedFactorsIndex(factors); // Create a variable index for the factors to be re-eliminated
#ifndef NDEBUG
    //  beforePermutationIndex.permute(affectedKeysSelector);
    //  assert(assert_equal(affectedFactorsIndex, beforePermutationIndex));
#endif
    if(debug) affectedFactorsIndex.print("affectedFactorsIndex: ");
    toc(2,"variable index");
    tic(3,"ccolamd");
#ifdef PRESORT_ALPHA
    Permutation alphaOrder(affectedKeysSet->size());
    vector<Symbol> orderedKeys; orderedKeys.reserve(ordering_.size());
    Index alphaVar = 0;
    BOOST_FOREACH(const Ordering::value_type& key_order, ordering_) {
      Permutation::const_iterator selected = find(affectedKeysSelector.begin(), affectedKeysSelector.end(), key_order.second);
      if(selected != affectedKeysSelector.end()) {
        Index selectedVar = selected - affectedKeysSelector.begin();
        alphaOrder[alphaVar] = selectedVar;
        ++ alphaVar;
      }
    }
    assert(alphaVar == affectedKeysSet->size());
    vector<Index> markedKeysSelected; markedKeysSelected.reserve(markedKeys.size());
    BOOST_FOREACH(Index var, markedKeys) { markedKeysSelected.push_back(alphaOrder[affectedKeysSelectorInverse[var]]); }
    GaussianVariableIndex<> origAffectedFactorsIndex(affectedFactorsIndex);
    affectedFactorsIndex.permute(alphaOrder);
    Permutation::shared_ptr affectedColamd(Inference::PermutationCOLAMD(affectedFactorsIndex, markedKeysSelected));
    affectedFactorsIndex.permute(*alphaOrder.inverse());
    affectedColamd = alphaOrder.permute(*affectedColamd);
#else
    //  vector<Index> markedKeysSelected; markedKeysSelected.reserve(markedKeys.size());
    //  BOOST_FOREACH(Index var, markedKeys) { markedKeysSelected.push_back(affectedKeysSelectorInverse[var]); }
    //  vector<Index> newKeysSelected; newKeysSelected.reserve(newKeys.size());
    //  BOOST_FOREACH(Index var, newKeys) { newKeysSelected.push_back(affectedKeysSelectorInverse[var]); }
    vector<int> cmember(affectedKeysSelector.size(), 0);
    if(structuralLast) {
      if(affectedKeysSelector.size() > affectedStructuralKeys.size()) {
        BOOST_FOREACH(Index var, affectedStructuralKeys) { cmember[affectedKeysSelectorInverse[var]] = 1; }
        if(latestLast) { BOOST_FOREACH(Index var, newKeys) { cmember[affectedKeysSelectorInverse[var]] = 2; } }
      }
    } else if(latestLast) {
      FastSet<Index> newKeysSet(newKeys.begin(), newKeys.end());
      if(theta_.size() > newKeysSet.size()) {
        BOOST_FOREACH(Index var, newKeys) { cmember[affectedKeysSelectorInverse[var]] = 1; }
      }
    }
    Permutation::shared_ptr affectedColamd(Inference::PermutationCOLAMD_(affectedFactorsIndex, cmember));
    if(disableReordering) {
      affectedColamd.reset(new Permutation(Permutation::Identity(affectedKeysSelector.size())));
      //    assert(affectedColamd->equals(Permutation::Identity(ordering_.nVars())));
    }
#endif
    toc(3,"ccolamd");
    tic(4,"ccolamd permutations");
    Permutation::shared_ptr affectedColamdInverse(affectedColamd->inverse());
    //  if(disableReordering) assert(affectedColamdInverse->equals(Permutation::Identity(ordering_.nVars())));
    if(debug) affectedColamd->print("affectedColamd: ");
    if(debug) affectedColamdInverse->print("affectedColamdInverse: ");
    Permutation::shared_ptr partialReordering(
        Permutation::Identity(ordering_.nVars()).partialPermutation(affectedKeysSelector, *affectedColamd));
    Permutation::shared_ptr partialReorderingInverse(
        Permutation::Identity(ordering_.nVars()).partialPermutation(affectedKeysSelector, *affectedColamdInverse));
    //  if(disableReordering) { assert(partialReordering->equals(Permutation::Identity(ordering_.nVars()))); assert(partialReorderingInverse->equals(Permutation::Identity(ordering_.nVars()))); }
    if(debug) partialReordering->print("partialReordering: ");
    toc(4,"ccolamd permutations");

    // We now need to permute everything according this partial reordering: the
    // delta vector, the global ordering, and the factors we're about to
    // re-eliminate.  The reordered variables are also mentioned in the
    // orphans and the leftover cached factors.
    // NOTE: We have shared_ptr's to cached factors that we permute here, thus we
    // undo this permutation after elimination.
    tic(5,"permute global variable index");
    variableIndex_.permute(*partialReordering);
    toc(5,"permute global variable index");
    tic(6,"permute affected variable index");
    affectedFactorsIndex.permute(*affectedColamd);
    toc(6,"permute affected variable index");
    tic(7,"permute delta");
    delta_.permute(*partialReordering);
    toc(7,"permute delta");
    tic(8,"permute ordering");
    ordering_.permuteWithInverse(*partialReorderingInverse);
    toc(8,"permute ordering");
    tic(9,"permute affected factors");
    factors.permuteWithInverse(*affectedColamdInverse);
    toc(9,"permute affected factors");

    if(debug) factors.print("Colamd-ordered affected factors: ");

#ifndef NDEBUG
    VariableIndex fromScratchIndex(factors);
    assert(assert_equal(fromScratchIndex, affectedFactorsIndex));
    //  beforePermutationIndex.permute(*affectedColamd);
    //  assert(assert_equal(fromScratchIndex, beforePermutationIndex));
#endif

    //  Permutation::shared_ptr reorderedSelectorInverse(affectedKeysSelector.permute(*affectedColamd));
    //  reorderedSelectorInverse->print("reorderedSelectorInverse: ");
    toc(4,"reorder");

    // eliminate into a Bayes net
    tic(5,"eliminate");
    GaussianJunctionTree jt(factors);
    sharedClique newRoot = jt.eliminate(EliminateQR, true);
    if(debug && newRoot) cout << "Re-eliminated BT:\n";
    if(debug && newRoot) newRoot->printTree("");
    toc(5,"eliminate");

    tic(6,"re-assemble");
    tic(1,"permute eliminated");
    if(newRoot) newRoot->permuteWithInverse(affectedKeysSelector);
    if(debug && newRoot) cout << "Full var-ordered eliminated BT:\n";
    if(debug && newRoot) newRoot->printTree("");
    toc(1,"permute eliminated");
    tic(2,"insert");
    if(newRoot) {
      assert(!this->root_);
      this->insert(newRoot);
    }
    toc(2,"insert");
    toc(6,"re-assemble");

    // 4. Insert the orphans back into the new Bayes tree.
    tic(7,"orphans");
    tic(1,"permute");
    BOOST_FOREACH(sharedClique orphan, orphans) {
      (void)orphan->permuteSeparatorWithInverse(*partialReorderingInverse);
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

  // Output: BayesTree(this)

//  boost::shared_ptr<set<Index> > affectedKeysSet(new set<Index>());
//  affectedKeysSet->insert(affectedKeys.begin(), affectedKeys.end());
}

///* ************************************************************************* */
//template<class Conditional, class Values>
//void ISAM2<Conditional, Values>::linear_update(const GaussianFactorGraph& newFactors) {
//  const list<Index> markedKeys = newFactors.keys();
//  recalculate(markedKeys, &newFactors);
//}

/* ************************************************************************* */
// find all variables that are directly connected by a measurement to one of the marked variables
template<class Conditional, class Values>
void ISAM2<Conditional, Values>::find_all(sharedClique clique, FastSet<Index>& keys, const vector<bool>& markedMask) {
  static const bool debug = false;
  // does the separator contain any of the variables?
  bool found = false;
  BOOST_FOREACH(const Index& key, (*clique)->parents()) {
    if (markedMask[key])
      found = true;
  }
  if (found) {
    // then add this clique
    keys.insert((*clique)->beginFrontals(), (*clique)->endFrontals());
    if(debug) clique->print("Key(s) marked in clique ");
    if(debug) cout << "so marking key " << (*clique)->keys().front() << endl;
  }
  BOOST_FOREACH(const sharedClique& child, clique->children_) {
    find_all(child, keys, markedMask);
  }
}

/* ************************************************************************* */
struct _SelectiveExpmap {
  const Permuted<VectorValues>& delta;
  const Ordering& ordering;
  const vector<bool>& mask;
  _SelectiveExpmap(const Permuted<VectorValues>& _delta, const Ordering& _ordering, const vector<bool>& _mask) :
    delta(_delta), ordering(_ordering), mask(_mask) {}
  template<typename I>
  void operator()(I it_x) {
    Index var = ordering[it_x->first];
    if(ISDEBUG("ISAM2 update verbose")) {
      if(mask[var])
        cout << "expmap " << (string)it_x->first << " (j = " << var << "), delta = " << delta[var].transpose() << endl;
      else
        cout << "       " << (string)it_x->first << " (j = " << var << "), delta = " << delta[var].transpose() << endl;
    }
    if(mask[var]) it_x->second = it_x->second.expmap(delta[var]);
  }
};
#ifndef NDEBUG
// This debug version sets delta entries that are applied to "Inf".  The
// idea is that if a delta is applied, the variable is being relinearized,
// so the same delta should not be re-applied because it will be recalc-
// ulated.  This is a debug check to prevent against a mix-up of indices
// or not keeping track of recalculated variables.
struct _SelectiveExpmapAndClear {
  Permuted<VectorValues>& delta;
  const Ordering& ordering;
  const vector<bool>& mask;
  _SelectiveExpmapAndClear(Permuted<VectorValues>& _delta, const Ordering& _ordering, const vector<bool>& _mask) :
    delta(_delta), ordering(_ordering), mask(_mask) {}
  template<typename I>
  void operator()(I it_x) {
    Index var = ordering[it_x->first];
    if(ISDEBUG("ISAM2 update verbose")) {
      if(mask[var])
        cout << "expmap " << (string)it_x->first << " (j = " << var << "), delta = " << delta[var].transpose() << endl;
      else
        cout << "       " << (string)it_x->first << " (j = " << var << "), delta = " << delta[var].transpose() << endl;
    }
    assert(delta[var].size() == (int)it_x->second.dim());
    assert(delta[var].unaryExpr(&isfinite<double>).all());
    if(disableReordering) {
      assert(mask[var]);
      //assert(it_x->first.index() == var);
      //assert(equal(delta[var], delta.container()[var]));
      assert(delta.permutation()[var] == var);
    }
    if(mask[var]) {
      it_x->second = it_x->second.expmap(delta[var]);
      delta[var].operator=(Vector::Constant(delta[var].rows(), numeric_limits<double>::infinity())); // Strange syntax to work with clang++ (bug in clang?)
    }
  }
};
#endif

/* ************************************************************************* */
template<class Conditional, class Values>
void ISAM2<Conditional, Values>::update(
    const NonlinearFactorGraph<Values>& newFactors, const Values& newTheta, bool force_relinearize) {

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

  if(verbose) {
    cout << "ISAM2::update\n";
    this->print("ISAM2: ");
  }

  tic(1,"push_back factors");
  // 1. Add any new factors \Factors:=\Factors\cup\Factors'.
  if(debug || verbose) newFactors.print("The new factors are: ");
  nonlinearFactors_.push_back(newFactors);
  toc(1,"push_back factors");

  tic(2,"add new variables");
  // 2. Initialize any new variables \Theta_{new} and add \Theta:=\Theta\cup\Theta_{new}.
  Impl::AddVariables(newTheta, theta_, delta_, ordering_, Base::nodes_);
  toc(2,"add new variables");

  tic(3,"gather involved keys");
  // 3. Mark linear update
  FastSet<Index> markedKeys = Impl::IndicesFromFactors(ordering_, newFactors); // Get keys from new factors
  vector<Index> newKeys; newKeys.reserve(markedKeys.size());
  newKeys.assign(markedKeys.begin(), markedKeys.end());                        // Make a copy of these, as we'll soon add to them
  FastSet<Index> structuralKeys;
  if(structuralLast) structuralKeys = markedKeys;                              // If we're using structural-last ordering, make another copy
  toc(3,"gather involved keys");

  vector<bool> markedRelinMask(ordering_.nVars(), false);
  bool relinAny = false;
  // Check relinearization if we're at a 10th step, or we are using a looser loop relin threshold
  if (force_relinearize || (params_.enableRelinearization && count % params_.relinearizeSkip == 0)) { // todo: every n steps
    tic(4,"gather relinearize keys");
    // 4. Mark keys in \Delta above threshold \beta: J=\{\Delta_{j}\in\Delta|\Delta_{j}\geq\beta\}.
    for(Index var=0; var<delta_.size(); ++var) {
      //cout << var << ": " << delta_[var].transpose() << endl;
      double maxDelta = delta_[var].lpNorm<Eigen::Infinity>();
      if(maxDelta >= params_.relinearizeThreshold || disableReordering) {
        markedRelinMask[var] = true;
        markedKeys.insert(var);
        if(!relinAny) relinAny = true;
      }
    }
    toc(4,"gather relinearize keys");

    tic(5,"fluid find_all");
    // 5. Mark all cliques that involve marked variables \Theta_{J} and all their ancestors.
    if (relinAny) {
      // mark all cliques that involve marked variables
      if(this->root())
        find_all(this->root(), markedKeys, markedRelinMask); // add other cliques that have the marked ones in the separator
      // richard commented these out since now using an array to mark keys
      //affectedKeys.sort(); // remove duplicates
      //affectedKeys.unique();
      // merge with markedKeys
    }
    // richard commented these out since now using an array to mark keys
    //markedKeys.splice(markedKeys.begin(), affectedKeys, affectedKeys.begin(), affectedKeys.end());
    //markedKeys.sort(); // remove duplicates
    //markedKeys.unique();
//    BOOST_FOREACH(const Index var, affectedKeys) {
//      markedKeys.push_back(var);
//    }
    toc(5,"fluid find_all");
  }

  tic(6,"expmap");
  // 6. Update linearization point for marked variables: \Theta_{J}:=\Theta_{J}+\Delta_{J}.
  if (relinAny) {
#ifndef NDEBUG
    _SelectiveExpmapAndClear selectiveExpmap(delta_, ordering_, markedRelinMask);
#else
    _SelectiveExpmap selectiveExpmap(delta_, ordering_, markedRelinMask);
#endif
    theta_.apply(selectiveExpmap);
//    theta_ = theta_.expmap(deltaMarked);
  }
  toc(6,"expmap");

#ifndef NDEBUG
  lastRelinVariables_ = markedRelinMask;
#endif

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
  if(markedKeys.size() > 0 || newKeys.size() > 0)
    replacedKeys = recalculate(markedKeys, structuralKeys, newKeys, linearFactors);
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

//#ifndef NDEBUG
//    FactorGraph<JacobianFactor> linearfullJ = *nonlinearFactors_.linearize(theta_, ordering_);
//    VectorValues deltafullJ = optimize(*GenericSequentialSolver<JacobianFactor>(linearfullJ).eliminate());
//    FactorGraph<HessianFactor> linearfullH =
//        *nonlinearFactors_.linearize(theta_, ordering_)->template convertCastFactors<FactorGraph<HessianFactor> >();
//    VectorValues deltafullH = optimize(*GenericSequentialSolver<HessianFactor>(linearfullH).eliminate());
//    if(!assert_equal(deltafullJ, newDelta, 1e-2))
//      throw runtime_error("iSAM2 does not agree with full Jacobian solver");
//    if(!assert_equal(deltafullH, newDelta, 1e-2))
//      throw runtime_error("iSAM2 does not agree with full Hessian solver");
    //#endif

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
}

/* ************************************************************************* */
template<class Conditional, class Values>
Values ISAM2<Conditional, Values>::calculateEstimate() const {
  Values ret(theta_);
  vector<bool> mask(ordering_.nVars(), true);
  _SelectiveExpmap selectiveExpmap(delta_, ordering_, mask);
  ret.apply(selectiveExpmap);
  return ret;
}

/* ************************************************************************* */
template<class Conditional, class Values>
Values ISAM2<Conditional, Values>::calculateBestEstimate() const {
  VectorValues delta(theta_.dims(ordering_));
  optimize2(this->root(), delta);
  return theta_.expmap(delta, ordering_);
}

}
/// namespace gtsam
