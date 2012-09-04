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
#include <boost/range/adaptors.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/algorithm/string.hpp>

#include <gtsam/base/timing.h>
#include <gtsam/base/debug.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/inference/BayesTree-inl.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/DoglegOptimizerImpl.h>


namespace gtsam {

using namespace std;

static const bool disableReordering = false;
static const double batchThreshold = 0.65;

/* ************************************************************************* */
std::string ISAM2DoglegParams::adaptationModeTranslator(const DoglegOptimizerImpl::TrustRegionAdaptationMode& adaptationMode) const {
  std::string s;
  switch (adaptationMode) {
  case DoglegOptimizerImpl::SEARCH_EACH_ITERATION:   s = "SEARCH_EACH_ITERATION";  break;
  case DoglegOptimizerImpl::ONE_STEP_PER_ITERATION:  s = "ONE_STEP_PER_ITERATION"; break;
  default:                                           s = "UNDEFINED";              break;
  }
  return s;
}

/* ************************************************************************* */
DoglegOptimizerImpl::TrustRegionAdaptationMode ISAM2DoglegParams::adaptationModeTranslator(const std::string& adaptationMode) const {
  std::string s = adaptationMode;  boost::algorithm::to_upper(s);
  if (s == "SEARCH_EACH_ITERATION")  return DoglegOptimizerImpl::SEARCH_EACH_ITERATION;
  if (s == "ONE_STEP_PER_ITERATION") return DoglegOptimizerImpl::ONE_STEP_PER_ITERATION;

  /* default is SEARCH_EACH_ITERATION */
  return DoglegOptimizerImpl::SEARCH_EACH_ITERATION;
}

/* ************************************************************************* */
ISAM2Params::Factorization ISAM2Params::factorizationTranslator(const std::string& str) const {
  std::string s = str;  boost::algorithm::to_upper(s);
  if (s == "QR") return ISAM2Params::QR;
  if (s == "CHOLESKY") return ISAM2Params::CHOLESKY;

  /* default is CHOLESKY */
  return ISAM2Params::CHOLESKY;
}

/* ************************************************************************* */
std::string ISAM2Params::factorizationTranslator(const ISAM2Params::Factorization& value) const {
  std::string s;
  switch (value) {
  case ISAM2Params::QR:         s = "QR"; break;
  case ISAM2Params::CHOLESKY:   s = "CHOLESKY"; break;
  default:                      s = "UNDEFINED"; break;
  }
  return s;
}

/* ************************************************************************* */
ISAM2::ISAM2(const ISAM2Params& params):
    deltaDoglegUptodate_(true), deltaUptodate_(true), params_(params) {
  if(params_.optimizationParams.type() == typeid(ISAM2DoglegParams))
    doglegDelta_ = boost::get<ISAM2DoglegParams>(params_.optimizationParams).initialDelta;
}

/* ************************************************************************* */
ISAM2::ISAM2():
    deltaDoglegUptodate_(true), deltaUptodate_(true) {
  if(params_.optimizationParams.type() == typeid(ISAM2DoglegParams))
    doglegDelta_ = boost::get<ISAM2DoglegParams>(params_.optimizationParams).initialDelta;
}

/* ************************************************************************* */
ISAM2::ISAM2(const ISAM2& other) {
  *this = other;
}

/* ************************************************************************* */
ISAM2& ISAM2::operator=(const ISAM2& rhs) {
  // Copy BayesTree
  this->Base::operator=(rhs);

  // Copy our variables
  // When we have Permuted<...>, it is only necessary to copy this permuted
  // view and not the original, because copying the permuted view automatically
  // copies the original.
  theta_ = rhs.theta_;
  variableIndex_ = rhs.variableIndex_;
  delta_ = rhs.delta_;
  deltaNewton_ = rhs.deltaNewton_;
  RgProd_ = rhs.RgProd_;
  deltaDoglegUptodate_ = rhs.deltaDoglegUptodate_;
  deltaUptodate_ = rhs.deltaUptodate_;
  deltaReplacedMask_ = rhs.deltaReplacedMask_;
  nonlinearFactors_ = rhs.nonlinearFactors_;

  linearFactors_ = GaussianFactorGraph();
  linearFactors_.reserve(rhs.linearFactors_.size());
  BOOST_FOREACH(const GaussianFactor::shared_ptr& linearFactor, rhs.linearFactors_) {
    linearFactors_.push_back(linearFactor->clone()); }

  ordering_ = rhs.ordering_;
  params_ = rhs.params_;
  doglegDelta_ = rhs.doglegDelta_;

  lastAffectedVariableCount = rhs.lastAffectedVariableCount;
  lastAffectedFactorCount = rhs.lastAffectedFactorCount;
  lastAffectedCliqueCount = rhs.lastAffectedCliqueCount;
  lastAffectedMarkedCount = rhs.lastAffectedMarkedCount;
  lastBacksubVariableCount = rhs.lastBacksubVariableCount;
  lastNnzTop = rhs.lastNnzTop;

  return *this;
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
ISAM2::relinearizeAffectedFactors(const FastList<Index>& affectedKeys, const FastSet<Index>& relinKeys) const {

  tic(1,"getAffectedFactors");
  FastList<size_t> candidates = getAffectedFactors(affectedKeys);
  toc(1,"getAffectedFactors");

  NonlinearFactorGraph nonlinearAffectedFactors;

  tic(2,"affectedKeysSet");
  // for fast lookup below
  FastSet<Index> affectedKeysSet;
  affectedKeysSet.insert(affectedKeys.begin(), affectedKeys.end());
  toc(2,"affectedKeysSet");

  tic(3,"check candidates and linearize");
  FactorGraph<GaussianFactor>::shared_ptr linearized = boost::make_shared<FactorGraph<GaussianFactor> >();
  BOOST_FOREACH(size_t idx, candidates) {
    bool inside = true;
    bool useCachedLinear = params_.cacheLinearizedFactors;
    BOOST_FOREACH(Key key, nonlinearFactors_[idx]->keys()) {
      Index var = ordering_[key];
      if(affectedKeysSet.find(var) == affectedKeysSet.end()) {
        inside = false;
        break;
      }
      if(useCachedLinear && relinKeys.find(var) != relinKeys.end())
        useCachedLinear = false;
    }
    if(inside) {
      if(useCachedLinear) {
        assert(linearFactors_[idx]);
        linearized->push_back(linearFactors_[idx]);
        assert(linearFactors_[idx]->keys() == nonlinearFactors_[idx]->symbolic(ordering_)->keys());
      } else {
        GaussianFactor::shared_ptr linearFactor = nonlinearFactors_[idx]->linearize(theta_, ordering_);
        linearized->push_back(linearFactor);
        if(params_.cacheLinearizedFactors) {
          assert(linearFactors_[idx]->keys() == linearFactor->keys());
          linearFactors_[idx] = linearFactor;
        }
      }
    }
  }
  toc(3,"check candidates and linearize");

  return linearized;
}

/* ************************************************************************* */
// find intermediate (linearized) factors from cache that are passed into the affected area
GaussianFactorGraph ISAM2::getCachedBoundaryFactors(Cliques& orphans) {

  static const bool debug = false;

  GaussianFactorGraph cachedBoundary;

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
    cachedBoundary.push_back(orphan->cachedFactor());
    if(debug) { cout << "Cached factor for variable " << key; orphan->cachedFactor()->print(""); }
  }

  return cachedBoundary;
}

boost::shared_ptr<FastSet<Index> > ISAM2::recalculate(const FastSet<Index>& markedKeys,
		const FastSet<Index>& relinKeys, const FastVector<Index>& observedKeys, const FastSet<Index>& unusedIndices,
    const boost::optional<FastMap<Index,int> >& constrainKeys, ISAM2Result& result) {

  // TODO:  new factors are linearized twice, the newFactors passed in are not used.

  const bool debug = ISDEBUG("ISAM2 recalculate");

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
    cout << "observedKeys: ";
    BOOST_FOREACH(const Index key, observedKeys) { cout << key << " "; }
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

  boost::shared_ptr<FastSet<Index> > affectedKeysSet(new FastSet<Index>()); // Will return this result

  if(affectedKeys.size() >= theta_.size() * batchThreshold) {

    tic(3,"batch");

    tic(0,"add keys");
    BOOST_FOREACH(const Ordering::value_type& key_index, ordering_) { affectedKeysSet->insert(key_index.second); }
    toc(0,"add keys");

    tic(1,"reorder");
    tic(1,"CCOLAMD");
    // Do a batch step - reorder and relinearize all variables
    vector<int> cmember(theta_.size(), 0);
    if(constrainKeys) {
      if(!constrainKeys->empty()) {
        typedef std::pair<const Index,int> Index_Group;
        if(theta_.size() > constrainKeys->size()) { // Only if some variables are unconstrained
          BOOST_FOREACH(const Index_Group& index_group, *constrainKeys) {
            cmember[index_group.first] = index_group.second; }
        } else {
          int minGroup = *boost::range::min_element(boost::adaptors::values(*constrainKeys));
          BOOST_FOREACH(const Index_Group& index_group, *constrainKeys) {
            cmember[index_group.first] = index_group.second - minGroup; }
        }
      }
    } else {
      if(theta_.size() > observedKeys.size()) { // Only if some variables are unconstrained
        BOOST_FOREACH(Index var, observedKeys) { cmember[var] = 1; }
      }
    }
    Permutation::shared_ptr colamd(inference::PermutationCOLAMD_(variableIndex_, cmember));
    Permutation::shared_ptr colamdInverse(colamd->inverse());
    toc(1,"CCOLAMD");

    // Reorder
    tic(2,"permute global variable index");
    variableIndex_.permuteInPlace(*colamd);
    toc(2,"permute global variable index");
    tic(3,"permute delta");
    delta_ = delta_.permute(*colamd);
    deltaNewton_ = deltaNewton_.permute(*colamd);
    RgProd_ = RgProd_.permute(*colamd);
    toc(3,"permute delta");
    tic(4,"permute ordering");
    ordering_.permuteWithInverse(*colamdInverse);
    toc(4,"permute ordering");
    toc(1,"reorder");

    tic(2,"linearize");
    GaussianFactorGraph linearized = *nonlinearFactors_.linearize(theta_, ordering_);
    if(params_.cacheLinearizedFactors)
      linearFactors_ = linearized;
    toc(2,"linearize");

    tic(5,"eliminate");
    JunctionTree<GaussianFactorGraph, Base::Clique> jt(linearized, variableIndex_);
    sharedClique newRoot;
    if(params_.factorization == ISAM2Params::CHOLESKY)
      newRoot = jt.eliminate(EliminatePreferCholesky);
    else if(params_.factorization == ISAM2Params::QR)
      newRoot = jt.eliminate(EliminateQR);
    else assert(false);
    if(debug) newRoot->print("Eliminated: ");
    toc(5,"eliminate");

    tic(6,"insert");
    this->clear();
    this->insert(newRoot);
    toc(6,"insert");

    result.variablesReeliminated = affectedKeysSet->size();

    lastAffectedMarkedCount = markedKeys.size();
    lastAffectedVariableCount = affectedKeysSet->size();
    lastAffectedFactorCount = linearized.size();

    // Reeliminated keys for detailed results
    if(params_.enableDetailedResults) {
      BOOST_FOREACH(Key key, theta_.keys()) {
        result.detail->variableStatus[key].isReeliminated = true;
      }
    }

    toc(3,"batch");

  } else {

    tic(4,"incremental");

    // 2. Add the new factors \Factors' into the resulting factor graph
    FastList<Index> affectedAndNewKeys;
    affectedAndNewKeys.insert(affectedAndNewKeys.end(), affectedKeys.begin(), affectedKeys.end());
    affectedAndNewKeys.insert(affectedAndNewKeys.end(), observedKeys.begin(), observedKeys.end());
    tic(1,"relinearizeAffected");
    GaussianFactorGraph factors(*relinearizeAffectedFactors(affectedAndNewKeys, relinKeys));
    if(debug) factors.print("Relinearized factors: ");
    toc(1,"relinearizeAffected");

    if(debug) { cout << "Affected keys: "; BOOST_FOREACH(const Index key, affectedKeys) { cout << key << " "; } cout << endl; }

    // Reeliminated keys for detailed results
    if(params_.enableDetailedResults) {
      BOOST_FOREACH(Index index, affectedAndNewKeys) {
        result.detail->variableStatus[inverseOrdering_->at(index)].isReeliminated = true;
      }
    }

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
    GaussianFactorGraph cachedBoundary = getCachedBoundaryFactors(orphans);
    if(debug) cachedBoundary.print("Boundary factors: ");
    factors.push_back(cachedBoundary);
    toc(2,"cached");

    // END OF COPIED CODE

    // 3. Re-order and eliminate the factor graph into a Bayes net (Algorithm [alg:eliminate]), and re-assemble into a new Bayes tree (Algorithm [alg:BayesTree])

    tic(4,"reorder and eliminate");

    tic(1,"list to set");
    // create a partial reordering for the new and contaminated factors
    // markedKeys are passed in: those variables will be forced to the end in the ordering
    affectedKeysSet->insert(markedKeys.begin(), markedKeys.end());
    affectedKeysSet->insert(affectedKeys.begin(), affectedKeys.end());
    toc(1,"list to set");

    tic(2,"PartialSolve");
    Impl::ReorderingMode reorderingMode;
    reorderingMode.nFullSystemVars = ordering_.nVars();
    reorderingMode.algorithm = Impl::ReorderingMode::COLAMD;
    reorderingMode.constrain = Impl::ReorderingMode::CONSTRAIN_LAST;
    if(constrainKeys) {
      reorderingMode.constrainedKeys = *constrainKeys;
    } else {
      reorderingMode.constrainedKeys = FastMap<Index,int>();
      BOOST_FOREACH(Index var, observedKeys) { reorderingMode.constrainedKeys->insert(make_pair(var, 1)); }
    }
		FastSet<Index> affectedUsedKeys = *affectedKeysSet; // Remove unused keys from the set we pass to PartialSolve
		BOOST_FOREACH(Index unused, unusedIndices) {
			affectedUsedKeys.erase(unused);
		}
    Impl::PartialSolveResult partialSolveResult =
        Impl::PartialSolve(factors, affectedUsedKeys, reorderingMode, (params_.factorization == ISAM2Params::QR));
    toc(2,"PartialSolve");

    // We now need to permute everything according this partial reordering: the
    // delta vector, the global ordering, and the factors we're about to
    // re-eliminate.  The reordered variables are also mentioned in the
    // orphans and the leftover cached factors.
    tic(3,"permute global variable index");
    variableIndex_.permuteInPlace(partialSolveResult.fullReordering);
    toc(3,"permute global variable index");
    tic(4,"permute delta");
    delta_ = delta_.permute(partialSolveResult.fullReordering);
    deltaNewton_ = deltaNewton_.permute(partialSolveResult.fullReordering);
    RgProd_ = RgProd_.permute(partialSolveResult.fullReordering);
    toc(4,"permute delta");
    tic(5,"permute ordering");
    ordering_.permuteWithInverse(partialSolveResult.fullReorderingInverse);
    toc(5,"permute ordering");
    if(params_.cacheLinearizedFactors) {
      tic(6,"permute cached linear");
      //linearFactors_.permuteWithInverse(partialSolveResult.fullReorderingInverse);
      FastList<size_t> permuteLinearIndices = getAffectedFactors(affectedAndNewKeys);
      BOOST_FOREACH(size_t idx, permuteLinearIndices) {
        linearFactors_[idx]->permuteWithInverse(partialSolveResult.fullReorderingInverse);
      }
      toc(6,"permute cached linear");
    }

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
  }

  // Root clique variables for detailed results
  if(params_.enableDetailedResults) {
    BOOST_FOREACH(Index index, this->root()->conditional()->frontals()) {
      result.detail->variableStatus[inverseOrdering_->at(index)].inRootClique = true;
    }
  }

  return affectedKeysSet;
}

/* ************************************************************************* */
ISAM2Result ISAM2::update(
    const NonlinearFactorGraph& newFactors, const Values& newTheta, const FastVector<size_t>& removeFactorIndices,
    const boost::optional<FastMap<Key,int> >& constrainedKeys, bool force_relinearize) {

  const bool debug = ISDEBUG("ISAM2 update");
  const bool verbose = ISDEBUG("ISAM2 update verbose");

  static int count = 0;
  count++;

  lastAffectedVariableCount = 0;
  lastAffectedFactorCount = 0;
  lastAffectedCliqueCount = 0;
  lastAffectedMarkedCount = 0;
  lastBacksubVariableCount = 0;
  lastNnzTop = 0;
  ISAM2Result result;
  if(params_.enableDetailedResults)
    result.detail = ISAM2Result::DetailedResults();
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
		if(params_.cacheLinearizedFactors)
			linearFactors_.remove(index);
  }

  // Remove removed factors from the variable index so we do not attempt to relinearize them
  variableIndex_.remove(removeFactorIndices, *removeFactors.symbolic(ordering_));

	// Compute unused keys and indices
	FastSet<Key> unusedKeys;
	FastSet<Index> unusedIndices;
	{
		// Get keys from removed factors and new factors, and compute unused keys,
		// i.e., keys that are empty now and do not appear in the new factors.
		FastSet<Key> removedAndEmpty;
		BOOST_FOREACH(Key key, removeFactors.keys()) {
			if(variableIndex_[ordering_[key]].empty())
				removedAndEmpty.insert(removedAndEmpty.end(), key);
		}
		FastSet<Key> newFactorSymbKeys = newFactors.keys();
		std::set_difference(removedAndEmpty.begin(), removedAndEmpty.end(),
			newFactorSymbKeys.begin(), newFactorSymbKeys.end(), std::inserter(unusedKeys, unusedKeys.end()));

		// Get indices for unused keys
		BOOST_FOREACH(Key key, unusedKeys) {
			unusedIndices.insert(unusedIndices.end(), ordering_[key]);
		}
	}
  toc(1,"push_back factors");

  tic(2,"add new variables");
  // 2. Initialize any new variables \Theta_{new} and add \Theta:=\Theta\cup\Theta_{new}.
  Impl::AddVariables(newTheta, theta_, delta_, deltaNewton_, RgProd_, deltaReplacedMask_, ordering_);
  // New keys for detailed results
  if(params_.enableDetailedResults) {
    inverseOrdering_ = ordering_.invert();
    BOOST_FOREACH(Key key, newTheta.keys()) { result.detail->variableStatus[key].isNew = true; } }
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

	// Observed keys for detailed results
  if(params_.enableDetailedResults) {
    BOOST_FOREACH(Index index, markedKeys) {
      result.detail->variableStatus[inverseOrdering_->at(index)].isObserved = true;
    }
  }
  // NOTE: we use assign instead of the iterator constructor here because this
  // is a vector of size_t, so the constructor unintentionally resolves to
  // vector(size_t count, Index value) instead of the iterator constructor.
  FastVector<Index> observedKeys;  observedKeys.reserve(markedKeys.size());
	BOOST_FOREACH(Index index, markedKeys) {
		if(unusedIndices.find(index) == unusedIndices.end()) // Only add if not unused
			observedKeys.push_back(index); // Make a copy of these, as we'll soon add to them
	}
  toc(4,"gather involved keys");

  // Check relinearization if we're at the nth step, or we are using a looser loop relin threshold
  FastSet<Index> relinKeys;
  if (relinearizeThisStep) {
    tic(5,"gather relinearize keys");
    vector<bool> markedRelinMask(ordering_.nVars(), false);
    // 4. Mark keys in \Delta above threshold \beta: J=\{\Delta_{j}\in\Delta|\Delta_{j}\geq\beta\}.
    if(params_.enablePartialRelinearizationCheck)
      relinKeys = Impl::CheckRelinearizationPartial(root_, delta_, ordering_, params_.relinearizeThreshold);
    else
      relinKeys = Impl::CheckRelinearizationFull(delta_, ordering_, params_.relinearizeThreshold);
    if(disableReordering) relinKeys = Impl::CheckRelinearizationFull(delta_, ordering_, 0.0); // This is used for debugging

    // Above relin threshold keys for detailed results
    if(params_.enableDetailedResults) {
      BOOST_FOREACH(Index index, relinKeys) {
        result.detail->variableStatus[inverseOrdering_->at(index)].isAboveRelinThreshold = true;
        result.detail->variableStatus[inverseOrdering_->at(index)].isRelinearized = true; } }

    // Add the variables being relinearized to the marked keys
    BOOST_FOREACH(const Index j, relinKeys) { markedRelinMask[j] = true; }
    markedKeys.insert(relinKeys.begin(), relinKeys.end());
    toc(5,"gather relinearize keys");

    tic(6,"fluid find_all");
    // 5. Mark all cliques that involve marked variables \Theta_{J} and all their ancestors.
    if (!relinKeys.empty() && this->root()) {
      // add other cliques that have the marked ones in the separator
      Impl::FindAll(this->root(), markedKeys, markedRelinMask);

      // Relin involved keys for detailed results
      if(params_.enableDetailedResults) {
        FastSet<Index> involvedRelinKeys;
        Impl::FindAll(this->root(), involvedRelinKeys, markedRelinMask);
        BOOST_FOREACH(Index index, involvedRelinKeys) {
          if(!result.detail->variableStatus[inverseOrdering_->at(index)].isAboveRelinThreshold) {
            result.detail->variableStatus[inverseOrdering_->at(index)].isRelinearizeInvolved = true;
            result.detail->variableStatus[inverseOrdering_->at(index)].isRelinearized = true; } }
      }
    }
    toc(6,"fluid find_all");

    tic(7,"expmap");
    // 6. Update linearization point for marked variables: \Theta_{J}:=\Theta_{J}+\Delta_{J}.
    if (!relinKeys.empty())
      Impl::ExpmapMasked(theta_, delta_, ordering_, markedRelinMask, delta_);
    toc(7,"expmap");

    result.variablesRelinearized = markedKeys.size();
  } else {
    result.variablesRelinearized = 0;
  }

  tic(8,"linearize new");
  // 7. Linearize new factors
  if(params_.cacheLinearizedFactors) {
    tic(1,"linearize");
    FactorGraph<GaussianFactor>::shared_ptr linearFactors = newFactors.linearize(theta_, ordering_);
    linearFactors_.push_back(*linearFactors);
    assert(nonlinearFactors_.size() == linearFactors_.size());
    toc(1,"linearize");

    tic(2,"augment VI");
    // Augment the variable index with the new factors
    variableIndex_.augment(*linearFactors);
    toc(2,"augment VI");
  } else {
    variableIndex_.augment(*newFactors.symbolic(ordering_));
  }
  toc(8,"linearize new");

  tic(9,"recalculate");
  // 8. Redo top of Bayes tree
  // Convert constrained symbols to indices
  boost::optional<FastMap<Index,int> > constrainedIndices;
  if(constrainedKeys) {
    constrainedIndices = FastMap<Index,int>();
    typedef pair<const Key, int> Key_Group;
    BOOST_FOREACH(Key_Group key_group, *constrainedKeys) {
      constrainedIndices->insert(make_pair(ordering_[key_group.first], key_group.second));
    }
  }
  boost::shared_ptr<FastSet<Index> > replacedKeys;
  if(!markedKeys.empty() || !observedKeys.empty())
    replacedKeys = recalculate(markedKeys, relinKeys, observedKeys, unusedIndices, constrainedIndices, result);

  // Update replaced keys mask (accumulates until back-substitution takes place)
  if(replacedKeys) {
    BOOST_FOREACH(const Index var, *replacedKeys) {
      deltaReplacedMask_[var] = true; } }
  toc(9,"recalculate");

	// After the top of the tree has been redone and may have index gaps from
	// unused keys, condense the indices to remove gaps by rearranging indices
	// in all data structures.
  if(!unusedKeys.empty()) {
    tic(10,"remove variables");
    Impl::RemoveVariables(unusedKeys, root_, theta_, variableIndex_, delta_, deltaNewton_, RgProd_,
        deltaReplacedMask_, ordering_, Base::nodes_, linearFactors_);
    toc(10,"remove variables");
  }
  result.cliques = this->nodes().size();
  deltaDoglegUptodate_ = false;
  deltaUptodate_ = false;

  tic(11,"evaluate error after");
  if(params_.evaluateNonlinearError)
    result.errorAfter.reset(nonlinearFactors_.error(calculateEstimate()));
  toc(11,"evaluate error after");

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
    DoglegOptimizerImpl::IterationResult doglegResult(DoglegOptimizerImpl::Iterate(
        *doglegDelta_, doglegParams.adaptationMode, *this, nonlinearFactors_, theta_, ordering_, nonlinearFactors_.error(theta_), doglegParams.verbose));
    toc(1, "Dogleg Iterate");

    tic(2, "Copy dx_d");
    // Update Delta and linear step
    doglegDelta_ = doglegResult.Delta;
    delta_ = doglegResult.dx_d; // Copy the VectorValues containing with the linear solution
    toc(2, "Copy dx_d");
  }

  deltaUptodate_ = true;
}

/* ************************************************************************* */
Values ISAM2::calculateEstimate() const {
  // We use ExpmapMasked here instead of regular expmap because the former
  // handles Permuted<VectorValues>
  tic(1, "Copy Values");
  Values ret(theta_);
  toc(1, "Copy Values");
  tic(2, "getDelta");
  const VectorValues& delta(getDelta());
  toc(2, "getDelta");
  tic(3, "Expmap");
  vector<bool> mask(ordering_.nVars(), true);
  Impl::ExpmapMasked(ret, delta, ordering_, mask);
  toc(3, "Expmap");
  return ret;
}

/* ************************************************************************* */
Values ISAM2::calculateBestEstimate() const {
  VectorValues delta(theta_.dims(ordering_));
  internal::optimizeInPlace<Base>(this->root(), delta);
  return theta_.retract(delta, ordering_);
}

/* ************************************************************************* */
const VectorValues& ISAM2::getDelta() const {
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
    double wildfireThreshold = 0.0;
    if(isam.params().optimizationParams.type() == typeid(ISAM2GaussNewtonParams))
      wildfireThreshold = boost::get<ISAM2GaussNewtonParams>(isam.params().optimizationParams).wildfireThreshold;
    else if(isam.params().optimizationParams.type() == typeid(ISAM2DoglegParams))
      wildfireThreshold = boost::get<ISAM2DoglegParams>(isam.params().optimizationParams).wildfireThreshold;
    else
      assert(false);
    ISAM2::Impl::UpdateDoglegDeltas(isam, wildfireThreshold, isam.deltaReplacedMask_, isam.deltaNewton_, isam.RgProd_);
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
    double wildfireThreshold = 0.0;
    if(isam.params().optimizationParams.type() == typeid(ISAM2GaussNewtonParams))
      wildfireThreshold = boost::get<ISAM2GaussNewtonParams>(isam.params().optimizationParams).wildfireThreshold;
    else if(isam.params().optimizationParams.type() == typeid(ISAM2DoglegParams))
      wildfireThreshold = boost::get<ISAM2DoglegParams>(isam.params().optimizationParams).wildfireThreshold;
    else
      assert(false);
    ISAM2::Impl::UpdateDoglegDeltas(isam, wildfireThreshold, isam.deltaReplacedMask_, isam.deltaNewton_, isam.RgProd_);
    isam.deltaDoglegUptodate_ = true;
    toc(1, "UpdateDoglegDeltas");
  }

  tic(2, "Compute Gradient");
  // Compute gradient (call gradientAtZero function, which is defined for various linear systems)
  gradientAtZero(isam, grad);
  double gradientSqNorm = grad.dot(grad);
  toc(2, "Compute Gradient");

  tic(3, "Compute minimizing step size");
  // Compute minimizing step size
  double RgNormSq = isam.RgProd_.vector().squaredNorm();
  double step = -gradientSqNorm / RgNormSq;
  toc(3, "Compute minimizing step size");

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
  if(bayesTree.root())
    gradientAtZeroTreeAdder(bayesTree.root(), g);
}

}
/// namespace gtsam
