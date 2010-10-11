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
#include <gtsam/nonlinear/NonlinearFactorGraph-inl.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/GaussianJunctionTree.h>

#include <gtsam/inference/Conditional.h>
#include <gtsam/inference/BayesTree-inl.h>
#include <gtsam/inference/ISAM2.h>

// for WAFR paper, separate update and relinearization steps if defined
//#define SEPARATE_STEPS


namespace gtsam {

using namespace std;

static const bool disableReordering = false;

/** Create an empty Bayes Tree */
template<class Conditional, class Values>
ISAM2<Conditional, Values>::ISAM2() : BayesTree<Conditional>(), delta_(Permutation(), deltaUnpermuted_) {}

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
list<size_t> ISAM2<Conditional, Values>::getAffectedFactors(const list<Index>& keys) const {
  static const bool debug = false;
  if(debug) cout << "Getting affected factors for ";
  if(debug) { BOOST_FOREACH(const Index key, keys) { cout << key << " "; } }
  if(debug) cout << endl;

  FactorGraph<NonlinearFactor<Values> > allAffected;
  list<size_t> indices;
  BOOST_FOREACH(const Index key, keys) {
//    const list<size_t> l = nonlinearFactors_.factors(key);
//    indices.insert(indices.begin(), l.begin(), l.end());
    const VariableIndexType::mapped_type& factors(variableIndex_[key]);
    BOOST_FOREACH(const VariableIndexType::mapped_factor_type& factor, factors) {
      if(debug) cout << "Variable " << key << " affects factor " << factor.factorIndex << endl;
      indices.push_back(factor.factorIndex);
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
boost::shared_ptr<GaussianFactorGraph> ISAM2<Conditional, Values>::relinearizeAffectedFactors
(const list<Index>& affectedKeys) const {

  tic("8.2.2.1 getAffectedFactors");
  list<size_t> candidates = getAffectedFactors(affectedKeys);
  toc("8.2.2.1 getAffectedFactors");

  NonlinearFactorGraph<Values> nonlinearAffectedFactors;

  tic("8.2.2.2 affectedKeysSet");
  // for fast lookup below
  set<Index> affectedKeysSet;
  affectedKeysSet.insert(affectedKeys.begin(), affectedKeys.end());
  toc("8.2.2.2 affectedKeysSet");

  tic("8.2.2.3 check candidates");
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
  toc("8.2.2.3 check candidates");

  return nonlinearAffectedFactors.linearize(theta_, ordering_);
}

/* ************************************************************************* */
// find intermediate (linearized) factors from cache that are passed into the affected area
template<class Conditional, class Values>
GaussianFactorGraph ISAM2<Conditional, Values>::getCachedBoundaryFactors(Cliques& orphans) {

  static const bool debug = false;

  GaussianFactorGraph cachedBoundary;

  BOOST_FOREACH(sharedClique orphan, orphans) {
    // find the last variable that was eliminated
    Index key = orphan->ordering().back();
#ifndef NDEBUG
//    typename BayesNet<Conditional>::const_iterator it = orphan->end();
//    const Conditional& lastConditional = **(--it);
//    typename Conditional::const_iterator keyit = lastConditional.endParents();
//    const Index lastKey = *(--keyit);
//    assert(key == lastKey);
#endif
    // retrieve the cached factor and add to boundary
    cachedBoundary.push_back(orphan->cachedFactor());
    if(debug) { cout << "Cached factor for variable " << key; orphan->cachedFactor()->print(""); }
  }

  return cachedBoundary;
}

/* ************************************************************************* */
template<class Conditional,class Values>
void reinsertCache(const typename ISAM2<Conditional,Values>::sharedClique& root, vector<GaussianFactor::shared_ptr>& cache, const Permutation& selector, const Permutation& selectorInverse) {
  static const bool debug = false;
  if(root) {
    if(root->size() > 0) {
      typename Conditional::shared_ptr& lastConditional = root->back();
      GaussianFactor::shared_ptr& cachedFactor = cache[selectorInverse[lastConditional->key()]];
      assert(cachedFactor);
      cachedFactor->permuteWithInverse(selector);
      if(debug) {
        cout << "Conditional, " << lastConditional->endParents()-lastConditional->beginParents() << " parents: ";
        for(typename Conditional::const_iterator key=lastConditional->beginParents(); key!=lastConditional->endParents(); ++key)
          cout << *key << " ";
        cout << endl;
        lastConditional->print("lastConditional: ");
        cout << "For key " << lastConditional->key() << " (" << selectorInverse[lastConditional->key()] << " selected) ";
        cachedFactor->print("cachedFactor: ");
      }
      assert((lastConditional->beginParents()==lastConditional->endParents() && cachedFactor->begin()==cachedFactor->end()) ||
          std::equal(lastConditional->beginParents(), lastConditional->endParents(), cachedFactor->begin()));
      assert(!root->cachedFactor());
      root->cachedFactor() = cachedFactor;
    }
    typedef ISAM2<Conditional,Values> This;
    BOOST_FOREACH(typename This::sharedClique& child, root->children()) {
      reinsertCache<Conditional,Values>(child, cache, selector, selectorInverse);
    }
  }
}

template<class Conditional, class Values>
boost::shared_ptr<set<Index> > ISAM2<Conditional, Values>::recalculate(const set<Index>& markedKeys, const vector<Index>& newKeys, const GaussianFactorGraph* newFactors) {

  static const bool debug = false;
  static const bool useMultiFrontal = true;

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

//  if(debug) newFactors->print("Recalculating factors: ");
  if(debug) {
    cout << "markedKeys: ";
    BOOST_FOREACH(const Index key, markedKeys) { cout << key << " "; }
    cout << endl;
  }

  // 1. Remove top of Bayes tree and convert to a factor graph:
  // (a) For each affected variable, remove the corresponding clique and all parents up to the root.
  // (b) Store orphaned sub-trees \BayesTree_{O} of removed cliques.
  tic("8.1 re-removetop");
  Cliques orphans;
  BayesNet<GaussianConditional> affectedBayesNet;
  this->removeTop(markedKeys, affectedBayesNet, orphans);
  toc("8.1 re-removetop");

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

  tic("8.2 re-lookup");
  // ordering provides all keys in conditionals, there cannot be others because path to root included
  tic("8.2.1 re-lookup: affectedKeys");
  list<Index> affectedKeys = affectedBayesNet.ordering();
  toc("8.2.1 re-lookup: affectedKeys");
//#ifndef NDEBUG
//  Index lastKey;
//  for(list<Index>::const_iterator key=affectedKeys.begin(); key!=affectedKeys.end(); ++key) {
//    if(key != affectedKeys.begin())
//      assert(*key > lastKey);
//    lastKey = *key;
//  }
//#endif
  list<Index> affectedAndNewKeys;
  affectedAndNewKeys.insert(affectedAndNewKeys.end(), affectedKeys.begin(), affectedKeys.end());
  affectedAndNewKeys.insert(affectedAndNewKeys.end(), newKeys.begin(), newKeys.end());
  tic("8.2.2 re-lookup: relinearizeAffected");
  GaussianFactorGraph factors(*relinearizeAffectedFactors(affectedAndNewKeys));
  toc("8.2.2 re-lookup: relinearizeAffected");

#ifndef NDEBUG
#ifndef SEPARATE_STEPS
  // The relinearized variables should not appear anywhere in the orphans
  BOOST_FOREACH(boost::shared_ptr<const typename BayesTree<Conditional>::Clique> clique, orphans) {
    BOOST_FOREACH(const typename GaussianConditional::shared_ptr& cond, *clique) {
      BOOST_FOREACH(const Index key, cond->keys()) {
        assert(lastRelinVariables_[key] == false);
      }
    }
  }
#endif
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

  toc("8.2 re-lookup");

//#ifndef NDEBUG
//  for(Index var=0; var<cached_.size(); ++var) {
//    if(find(affectedKeys.begin(), affectedKeys.end(), var) == affectedKeys.end() ||
//        lastRelinVariables_[var] == true) {
//      assert(!cached_[var] || find(cached_[var]->begin(), cached_[var]->end(), var) == cached_[var]->end());
//    }
//  }
//#endif

  tic("8.3 re-cached");
  // add the cached intermediate results from the boundary of the orphans ...
  GaussianFactorGraph cachedBoundary = getCachedBoundaryFactors(orphans);
  if(debug) cachedBoundary.print("Boundary factors: ");
  factors.reserve(factors.size() + cachedBoundary.size());
  // Copy so that we can later permute factors
  BOOST_FOREACH(const GaussianFactor::shared_ptr& cached, cachedBoundary) {
#ifndef NDEBUG
#ifndef SEPARATE_STEPS
    BOOST_FOREACH(const Index key, *cached) {
      assert(lastRelinVariables_[key] == false);
    }
#endif
#endif
    factors.push_back(GaussianFactor::shared_ptr(new GaussianFactor(*cached)));
  }
//  factors.push_back(cachedBoundary);
  toc("8.3 re-cached");

  // END OF COPIED CODE


  // 2. Add the new factors \Factors' into the resulting factor graph
  tic("8.4 re-newfactors");
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
  toc("8.4 re-newfactors");

  // 3. Re-order and eliminate the factor graph into a Bayes net (Algorithm [alg:eliminate]), and re-assemble into a new Bayes tree (Algorithm [alg:BayesTree])

  tic("8.5 re-order");

//#define PRESORT_ALPHA

  tic("8.5.1 re-order: select affected variables");
  // create a partial reordering for the new and contaminated factors
  // markedKeys are passed in: those variables will be forced to the end in the ordering
  boost::shared_ptr<set<Index> > affectedKeysSet(new set<Index>(markedKeys));
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
  if(disableReordering) { assert(affectedKeysSelector.equals(Permutation::Identity(ordering_.nVars()))); assert(affectedKeysSelectorInverse.equals(Permutation::Identity(ordering_.nVars()))); }
  if(debug) affectedKeysSelector.print("affectedKeysSelector: ");
  if(debug) affectedKeysSelectorInverse.print("affectedKeysSelectorInverse: ");
#ifndef NDEBUG
  GaussianVariableIndex<> beforePermutationIndex(factors);
#endif
  factors.permuteWithInverse(affectedKeysSelectorInverse);
  if(debug) factors.print("Factors to reorder/re-eliminate: ");
  toc("8.5.1 re-order: select affected variables");
  tic("8.5.2 re-order: variable index");
  GaussianVariableIndex<> affectedFactorsIndex(factors); // Create a variable index for the factors to be re-eliminated
#ifndef NDEBUG
//  beforePermutationIndex.permute(affectedKeysSelector);
//  assert(assert_equal(affectedFactorsIndex, beforePermutationIndex));
#endif
  if(debug) affectedFactorsIndex.print("affectedFactorsIndex: ");
  toc("8.5.2 re-order: variable index");
  tic("8.5.3 re-order: constrained colamd");
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
  vector<Index> newKeysSelected; newKeysSelected.reserve(newKeys.size());
  BOOST_FOREACH(Index var, newKeys) { newKeysSelected.push_back(affectedKeysSelectorInverse[var]); }
  Permutation::shared_ptr affectedColamd(Inference::PermutationCOLAMD(affectedFactorsIndex, newKeysSelected));
  if(disableReordering) {
    affectedColamd.reset(new Permutation(Permutation::Identity(affectedKeysSelector.size())));
    assert(affectedColamd->equals(Permutation::Identity(ordering_.nVars())));
  }
#endif
  toc("8.5.3 re-order: constrained colamd");
  tic("8.5.4 re-order: create ccolamd permutations");
  Permutation::shared_ptr affectedColamdInverse(affectedColamd->inverse());
  if(disableReordering) assert(affectedColamdInverse->equals(Permutation::Identity(ordering_.nVars())));
  if(debug) affectedColamd->print("affectedColamd: ");
  if(debug) affectedColamdInverse->print("affectedColamdInverse: ");
  Permutation::shared_ptr partialReordering(
      Permutation::Identity(ordering_.nVars()).partialPermutation(affectedKeysSelector, *affectedColamd));
  Permutation::shared_ptr partialReorderingInverse(
      Permutation::Identity(ordering_.nVars()).partialPermutation(affectedKeysSelector, *affectedColamdInverse));
  if(disableReordering) { assert(partialReordering->equals(Permutation::Identity(ordering_.nVars()))); assert(partialReorderingInverse->equals(Permutation::Identity(ordering_.nVars()))); }
  if(debug) partialReordering->print("partialReordering: ");
  toc("8.5.4 re-order: create ccolamd permutations");

  // We now need to permute everything according this partial reordering: the
  // delta vector, the global ordering, and the factors we're about to
  // re-eliminate.  The reordered variables are also mentioned in the
  // orphans and the leftover cached factors.
  // NOTE: We have shared_ptr's to cached factors that we permute here, thus we
  // undo this permutation after elimination.
  tic("8.5.5 re-order: ccolamd permute global variable index");
  variableIndex_.permute(*partialReordering);
  toc("8.5.5 re-order: ccolamd permute global variable index");
  tic("8.5.6 re-order: ccolamd permute affected variable index");
  affectedFactorsIndex.permute(*affectedColamd);
  toc("8.5.6 re-order: ccolamd permute affected variable index");
  tic("8.5.7 re-order: ccolamd permute delta");
  delta_.permute(*partialReordering);
  toc("8.5.7 re-order: ccolamd permute delta");
  tic("8.5.8 re-order: ccolamd permute ordering");
  ordering_.permuteWithInverse(*partialReorderingInverse);
  toc("8.5.8 re-order: ccolamd permute ordering");
  tic("8.5.9 re-order: ccolamd permute affected factors");
  factors.permuteWithInverse(*affectedColamdInverse);
  toc("8.5.9 re-order: ccolamd permute affected factors");

  if(debug) factors.print("Colamd-ordered affected factors: ");

#ifndef NDEBUG
  GaussianVariableIndex<> fromScratchIndex(factors);
  assert(assert_equal(fromScratchIndex, affectedFactorsIndex));
//  beforePermutationIndex.permute(*affectedColamd);
//  assert(assert_equal(fromScratchIndex, beforePermutationIndex));
#endif

//  Permutation::shared_ptr reorderedSelectorInverse(affectedKeysSelector.permute(*affectedColamd));
//  reorderedSelectorInverse->print("reorderedSelectorInverse: ");
  toc("8.5 re-order");

  // eliminate into a Bayes net
  if(useMultiFrontal) {
    tic("8.6 eliminate");
    GaussianJunctionTree jt(factors);
    sharedClique newRoot = jt.eliminate();
    if(debug && newRoot) cout << "Re-eliminated BT:\n";
    if(debug && newRoot) newRoot->printTree("");
    toc("8.6 eliminate");

    tic("8.7 re-assemble");
    tic("8.7.1 permute eliminated");
    if(newRoot) newRoot->permuteWithInverse(affectedKeysSelector);
    if(debug && newRoot) cout << "Full var-ordered eliminated BT:\n";
    if(debug && newRoot) newRoot->printTree("");
    toc("8.7.1 permute eliminated");
    tic("8.7.2 insert");
    if(newRoot) {
      assert(!this->root_);
      this->insert(newRoot);
    }
    toc("8.7.2 insert");
    toc("8.7 re-assemble");
  } else {
    tic("8.6 eliminate");
    boost::shared_ptr<GaussianBayesNet> bayesNet(new GaussianBayesNet());
    vector<GaussianFactor::shared_ptr> newlyCached(affectedKeysSelector.size());
    for(Index var=0; var<affectedKeysSelector.size(); ++var) {
      GaussianConditional::shared_ptr conditional = Inference::EliminateOne(factors, affectedFactorsIndex, var);
      //    assert(partialReordering[affectedKeysSelector[var]] == affectedKeysSelectorInverse[affectedColamd[var]]);
      //    assert(reorderedSelectorInverse[var] == partialReordering[affectedKeysSelector[var]]);
      if(conditional != NULL) {
        //      if(debug) cout << var << "th colamd variable becomes variable " << affectedKeysSelector[var] << endl;
        if(debug) cout << "Caching for variable " << var << "->" << affectedKeysSelector[var] << " factor ";
        if(debug) factors.back()->print("");
        newlyCached[var] = factors.back();
        bayesNet->push_back(conditional);
      }
    }
    toc("8.6 eliminate");

    tic("8.7 re-assemble");

    if(debug) bayesNet->print("Re-eliminated portion: ");
    // permute the BayesNet up to the full variable space
    tic("8.7.1 re-assemble: permute eliminated");
    bayesNet->permuteWithInverse(affectedKeysSelector);
    toc("8.7.1 re-assemble: permute eliminated");
    if(debug) bayesNet->print("Ready to re-insert (permuted): ");

    // insert conditionals back in, straight into the topless bayesTree
    tic("8.7.2 re-assemble: insert");
    typename BayesNet<Conditional>::const_reverse_iterator rit;
    for ( rit=bayesNet->rbegin(); rit != bayesNet->rend(); ++rit ) {
      this->insert(*rit);
    }
    toc("8.7.2 re-assemble: insert");

    tic("8.7.3 re-assemble: insert cache");
    if(bayesNet->size() == 0)
      assert(newlyCached.size() == 0);
    else
      reinsertCache<Conditional,Values>(this->root(), newlyCached, affectedKeysSelector, affectedKeysSelectorInverse);
    toc("8.7.3 re-assemble: insert cache");

    lastNnzTop = 0; //calculate_nnz(this->root());

    // Save number of affectedCliques
    lastAffectedCliqueCount = this->size();
    toc("8.7 re-assemble");
  }

  // 4. Insert the orphans back into the new Bayes tree.

  tic("8.8 re-orphan");
  tic("8.8.1 re-orphan: permute");
  BOOST_FOREACH(sharedClique orphan, orphans) {
    (void)orphan->permuteSeparatorWithInverse(*partialReorderingInverse);
  }
  toc("8.8.1 re-orphan: permute");
  tic("8.8.2 re-orphan: insert");
  // add orphans to the bottom of the new tree
  BOOST_FOREACH(sharedClique orphan, orphans) {
    // Because the affectedKeysSelector is sorted, the orphan separator keys
    // will be sorted correctly according to the new elimination order after
    // applying the permutation, so findParentClique, which looks for the
    // lowest-ordered parent, will still work.
    Index parentRepresentative = findParentClique(orphan->separator_);
    sharedClique parent = (*this)[parentRepresentative];
    parent->children_ += orphan;
    orphan->parent_ = parent; // set new parent!
  }
  toc("8.8.2 re-orphan: insert");
  toc("8.8 re-orphan");

  // Output: BayesTree(this)

//  boost::shared_ptr<set<Index> > affectedKeysSet(new set<Index>());
//  affectedKeysSet->insert(affectedKeys.begin(), affectedKeys.end());
  return affectedKeysSet;
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
void ISAM2<Conditional, Values>::find_all(sharedClique clique, set<Index>& keys, const vector<bool>& markedMask) {
  // does the separator contain any of the variables?
  bool found = false;
  BOOST_FOREACH(const Index& key, clique->separator_) {
    if (markedMask[key])
      found = true;
  }
  if (found) {
    // then add this clique
    assert(clique->keys().front() == (*clique->begin())->key());
    keys.insert(clique->keys().front());
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
    assert(delta[var].size() == it_x->second.dim());
    if(mask[var]) it_x->second = it_x->second.expmap(delta[var]); }
};
#ifndef NDEBUG
struct _SelectiveExpmapAndClear {
  Permuted<VectorValues>& delta;
  const Ordering& ordering;
  const vector<bool>& mask;
  _SelectiveExpmapAndClear(Permuted<VectorValues>& _delta, const Ordering& _ordering, const vector<bool>& _mask) :
    delta(_delta), ordering(_ordering), mask(_mask) {}
  template<typename I>
  void operator()(I it_x) {
    Index var = ordering[it_x->first];
    assert(delta[var].size() == it_x->second.dim());
    BOOST_FOREACH(double v, delta[var]) assert(isfinite(v));
    if(disableReordering) {
      assert(mask[var]);
      assert(it_x->first.index() == var);
      //assert(equal(delta[var], delta.container()[var]));
      assert(delta.permutation()[var] == var);
    }
    if(mask[var]) it_x->second = it_x->second.expmap(delta[var]);
    fill(delta[var].begin(), delta[var].end(), numeric_limits<double>::infinity());
  }
};
#endif
struct _VariableAdder {
  Ordering& ordering;
  Permuted<VectorValues>& vconfig;
  _VariableAdder(Ordering& _ordering, Permuted<VectorValues>& _vconfig) : ordering(_ordering), vconfig(_vconfig) {}
  template<typename I>
  void operator()(I xIt) {
    static const bool debug = false;
    Index var = vconfig->push_back_preallocated(zero(xIt->second.dim()));
    vconfig.permutation()[var] = var;
    ordering.insert(xIt->first, var);
    if(debug) cout << "Adding variable " << (string)xIt->first << " with order " << var << endl;
  }
};
template<class Conditional, class Values>
void ISAM2<Conditional, Values>::update(
    const NonlinearFactorGraph<Values>& newFactors, const Values& newTheta,
    double wildfire_threshold, double relinearize_threshold, bool relinearize) {

  static const bool debug = false;
  if(disableReordering) { wildfire_threshold = 0.0; relinearize_threshold = -1.0; }

  static int count = 0;
  count++;

  lastAffectedVariableCount = 0;
  lastAffectedFactorCount = 0;
  lastAffectedCliqueCount = 0;
  lastAffectedMarkedCount = 0;
  lastBacksubVariableCount = 0;
  lastNnzTop = 0;

  tic("all");

  tic("1 step1");
  // 1. Add any new factors \Factors:=\Factors\cup\Factors'.
  nonlinearFactors_.push_back(newFactors);
  toc("1 step1");

  tic("2 step2");
  // 2. Initialize any new variables \Theta_{new} and add \Theta:=\Theta\cup\Theta_{new}.
  theta_.insert(newTheta);
  if(debug) newTheta.print("The new variables are: ");
  // Add the new keys onto the ordering, add zeros to the delta for the new variables
  vector<Index> dims(newTheta.dims(*newTheta.orderingArbitrary(ordering_.nVars())));
  if(debug) cout << "New variables have total dimensionality " << accumulate(dims.begin(), dims.end(), 0) << endl;
  delta_.container().reserve(delta_->size() + newTheta.size(), delta_->dim() + accumulate(dims.begin(), dims.end(), 0));
  delta_.permutation().resize(delta_->size() + newTheta.size());
  {
    _VariableAdder vadder(ordering_, delta_);
    newTheta.apply(vadder);
    assert(delta_.permutation().size() == delta_.container().size());
    assert(delta_.container().dim() == delta_.container().dimCapacity());
    assert(ordering_.nVars() == delta_.size());
    assert(ordering_.size() == delta_.size());
  }
  assert(ordering_.nVars() >= this->nodes_.size());
  this->nodes_.resize(ordering_.nVars());
//  assert(ordering_.nVars() >= cached_.size());
//  cached_.resize(ordering_.nVars());
  toc("2 step2");

  tic("3 step3");
  // 3. Mark linear update
  set<Index> markedKeys;
  vector<Index> newKeys; newKeys.reserve(newFactors.size() * 6);
  BOOST_FOREACH(const typename NonlinearFactor<Values>::shared_ptr& factor, newFactors) {
    BOOST_FOREACH(const Symbol& key, factor->keys()) {
      markedKeys.insert(ordering_[key]);
      newKeys.push_back(ordering_[key]);
    }
  }
//  list<Index> markedKeys = newFactors.keys();
  toc("3 step3");

#ifdef SEPARATE_STEPS // original algorithm from paper: separate relin and optimize

  // todo: kaess - don't need linear factors here, just to update variableIndex
  boost::shared_ptr<GaussianFactorGraph> linearFactors = newFactors.linearize(theta_, ordering_);
  variableIndex_.augment(*linearFactors);

  boost::shared_ptr<set<Index> > replacedKeys_todo = recalculate(markedKeys, newKeys, &(*linearFactors));
  markedKeys.clear();
  vector<bool> none(variableIndex_.size(), false);
  optimize2(this->root(), wildfire_threshold, none, delta_);
#endif

  vector<bool> markedRelinMask(ordering_.nVars(), false);
  bool relinAny = false;
  if (relinearize && count%10 == 0) { // todo: every n steps
    tic("4 step4");
    // 4. Mark keys in \Delta above threshold \beta: J=\{\Delta_{j}\in\Delta|\Delta_{j}\geq\beta\}.
    for(Index var=0; var<delta_.size(); ++var) {
      if (max(abs(delta_[var])) >= relinearize_threshold) {
        markedRelinMask[var] = true;
        markedKeys.insert(var);
        if(!relinAny) relinAny = true;
      }
    }
    toc("4 step4");

    tic("5 step5");
    // 5. Mark all cliques that involve marked variables \Theta_{J} and all their ancestors.
    if (relinAny) {
      // mark all cliques that involve marked variables
      tic("5.1 fluid-find_all");
      if(this->root())
        find_all(this->root(), markedKeys, markedRelinMask); // add other cliques that have the marked ones in the separator
      // richard commented these out since now using an array to mark keys
      //affectedKeys.sort(); // remove duplicates
      //affectedKeys.unique();
      // merge with markedKeys
      toc("5.1 fluid-find_all");
    }
    // richard commented these out since now using an array to mark keys
    //markedKeys.splice(markedKeys.begin(), affectedKeys, affectedKeys.begin(), affectedKeys.end());
    //markedKeys.sort(); // remove duplicates
    //markedKeys.unique();
//    BOOST_FOREACH(const Index var, affectedKeys) {
//      markedKeys.push_back(var);
//    }
    toc("5 step5");

  }

  tic("6 step6");
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
  toc("6 step6");

#ifndef NDEBUG
  lastRelinVariables_ = markedRelinMask;
#endif

#ifndef SEPARATE_STEPS
  tic("7 step7");
  // 7. Linearize new factors
  boost::shared_ptr<GaussianFactorGraph> linearFactors = newFactors.linearize(theta_, ordering_);
  toc("7 step7");

  tic("7.1 step7");
  // Augment the variable index with the new factors
//  tic("step7.5: newVarIndex");
//  cout << linearFactors->size() << "=" << newFactors.size() << " newFactors" << endl;
//  GaussianVariableIndex<> newVarIndex(*linearFactors);
//  toc("step7.5: newVarIndex");
//  tic("step7.5: rebase");
//  newVarIndex.rebaseFactors(newFactorsIndex);
//  toc("step7.5: rebase");
//  tic("step7.5: augment");
  variableIndex_.augment(*linearFactors);
//  toc("step7.5: augment");
  toc("7.1 step7");

  tic("8 step8");
  // 8. Redo top of Bayes tree
  boost::shared_ptr<set<Index> > replacedKeys = recalculate(markedKeys, newKeys, &(*linearFactors));
  toc("8 step8");
#else
  vector<Index> empty;
  boost::shared_ptr<set<Index> > replacedKeys = recalculate(markedKeys, empty);
#endif

  tic("9 step9");
  // 9. Solve
  if (wildfire_threshold<=0.) {
    VectorValues newDelta(variableIndex_.dims());
    optimize2(this->root(), newDelta);
    assert(newDelta.size() == delta_.size());
    delta_.permutation() = Permutation::Identity(delta_.size());
    delta_.container() = newDelta;
    lastBacksubVariableCount = theta_.size();

//    GaussianFactorGraph linearfull = *nonlinearFactors_.linearize(theta_, ordering_);
//    GaussianBayesNet gbn = *Inference::Eliminate(linearfull);
//    VectorValues deltafull = optimize(gbn);
//    assert(assert_equal(deltafull, newDelta, 1e-3));

  } else {
    vector<bool> replacedKeysMask(variableIndex_.size(), false);
    BOOST_FOREACH(const Index var, *replacedKeys) { replacedKeysMask[var] = true; }
    lastBacksubVariableCount = optimize2(this->root(), wildfire_threshold, replacedKeysMask, delta_); // modifies delta_
  }
  toc("9 step9");

  toc("all");
  tictoc_print(); // switch on/off at top of file (#if 1/#if 0)
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
  VectorValues delta(variableIndex_.dims());
  optimize2(this->root(), delta);
  return theta_.expmap(delta, ordering_);
}

}
/// namespace gtsam
