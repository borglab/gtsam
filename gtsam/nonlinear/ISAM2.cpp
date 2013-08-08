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
namespace br { using namespace boost::range; using namespace boost::adaptors; }

#include <gtsam/base/timing.h>
#include <gtsam/base/debug.h>
#include <gtsam/inference/BayesTree-inst.h>
#include <gtsam/inference/JunctionTree-inst.h> // We need the inst file because we'll make a special JT templated on ISAM2
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianEliminationTree.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/DoglegOptimizerImpl.h>
#include <gtsam/nonlinear/nonlinearExceptions.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

namespace gtsam {

using namespace std;

static const bool disableReordering = false;
static const double batchThreshold = 0.65;

/* ************************************************************************* */
// Special BayesTree class that uses ISAM2 cliques - this is the result of reeliminating ISAM2
// subtrees.
class ISAM2BayesTree : public ISAM2::Base
{
public:
  typedef ISAM2::Base Base;
  typedef ISAM2BayesTree This;
  typedef boost::shared_ptr<This> shared_ptr;

  ISAM2BayesTree() {}
};

/* ************************************************************************* */
// Special JunctionTree class that produces ISAM2 BayesTree cliques, used for reeliminating ISAM2
// subtrees.
class ISAM2JunctionTree : public JunctionTree<ISAM2BayesTree, GaussianFactorGraph>
{
public:
  typedef JunctionTree<ISAM2BayesTree, GaussianFactorGraph> Base;
  typedef ISAM2JunctionTree This;
  typedef boost::shared_ptr<This> shared_ptr;

  ISAM2JunctionTree(const GaussianEliminationTree& eliminationTree) :
    Base(Base::FromEliminationTree(eliminationTree)) {}
};

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
void ISAM2Clique::setEliminationResult(const typename FactorGraphType::EliminationResult& eliminationResult)
{
  conditional_ = eliminationResult.first;
  cachedFactor_ = eliminationResult.second;
  // Compute gradient contribution
  gradientContribution_.resize(conditional_->cols() - 1);
  // Rewrite -(R * P')'*d   as   -(d' * R * P')'   for computational speed reasons
  gradientContribution_ = -conditional_->get_R().transpose() * conditional_->get_d(),
    -conditional_->get_S().transpose() * conditional_->get_d();
}

/* ************************************************************************* */
bool ISAM2Clique::equals(const This& other, double tol) const {
  return Base::equals(other) &&
    ((!cachedFactor_ && !other.cachedFactor_)
    || (cachedFactor_ && other.cachedFactor_
    && cachedFactor_->equals(*other.cachedFactor_, tol)));
}

/* ************************************************************************* */
void ISAM2Clique::print(const std::string& s, const KeyFormatter& formatter) const
{
  Base::print(s,formatter);
  if(cachedFactor_)
    cachedFactor_->print(s + "Cached: ", formatter);
  else
    std::cout << s << "Cached empty" << std::endl;
  if(gradientContribution_.rows() != 0)
    gtsam::print(gradientContribution_, "Gradient contribution: ");
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
ISAM2& ISAM2::operator=(const ISAM2& rhs)
{
  // Copy BayesTree
  this->Base::operator=(rhs);

  // Copy our variables
  theta_ = rhs.theta_;
  variableIndex_ = rhs.variableIndex_;
  delta_ = rhs.delta_;
  deltaNewton_ = rhs.deltaNewton_;
  RgProd_ = rhs.RgProd_;
  deltaDoglegUptodate_ = rhs.deltaDoglegUptodate_;
  deltaUptodate_ = rhs.deltaUptodate_;
  deltaReplacedMask_ = rhs.deltaReplacedMask_;
  nonlinearFactors_ = rhs.nonlinearFactors_;
  linearFactors_ = rhs.linearFactors_;
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
FastList<size_t> ISAM2::getAffectedFactors(const FastList<Key>& keys) const {
  static const bool debug = false;
  if(debug) cout << "Getting affected factors for ";
  if(debug) { BOOST_FOREACH(const Key key, keys) { cout << key << " "; } }
  if(debug) cout << endl;

  NonlinearFactorGraph allAffected;
  FastList<size_t> indices;
  BOOST_FOREACH(const Key key, keys) {
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

GaussianFactorGraph::shared_ptr
ISAM2::relinearizeAffectedFactors(const FastList<Key>& affectedKeys, const FastSet<Key>& relinKeys) const
{
  gttic(getAffectedFactors);
  FastList<size_t> candidates = getAffectedFactors(affectedKeys);
  gttoc(getAffectedFactors);

  NonlinearFactorGraph nonlinearAffectedFactors;

  gttic(affectedKeysSet);
  // for fast lookup below
  FastSet<Key> affectedKeysSet;
  affectedKeysSet.insert(affectedKeys.begin(), affectedKeys.end());
  gttoc(affectedKeysSet);

  gttic(check_candidates_and_linearize);
  GaussianFactorGraph::shared_ptr linearized = boost::make_shared<GaussianFactorGraph>();
  BOOST_FOREACH(size_t idx, candidates) {
    bool inside = true;
    bool useCachedLinear = params_.cacheLinearizedFactors;
    BOOST_FOREACH(Key key, nonlinearFactors_[idx]->keys()) {
      if(affectedKeysSet.find(key) == affectedKeysSet.end()) {
        inside = false;
        break;
      }
      if(useCachedLinear && relinKeys.find(key) != relinKeys.end())
        useCachedLinear = false;
    }
    if(inside) {
      if(useCachedLinear) {
#ifdef GTSAM_EXTRA_CONSISTENCY_CHECKS
        assert(linearFactors_[idx]);
        assert(linearFactors_[idx]->keys() == nonlinearFactors_[idx]->keys());
#endif
        linearized->push_back(linearFactors_[idx]);
      } else {
        GaussianFactor::shared_ptr linearFactor = nonlinearFactors_[idx]->linearize(theta_);
        linearized->push_back(linearFactor);
        if(params_.cacheLinearizedFactors) {
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
// find intermediate (linearized) factors from cache that are passed into the affected area

GaussianFactorGraph ISAM2::getCachedBoundaryFactors(Cliques& orphans) {

  static const bool debug = false;

  GaussianFactorGraph cachedBoundary;

  BOOST_FOREACH(sharedClique orphan, orphans) {
    // retrieve the cached factor and add to boundary
    cachedBoundary.push_back(orphan->cachedFactor());
  }

  return cachedBoundary;
}

/* ************************************************************************* */
boost::shared_ptr<FastSet<Key> > ISAM2::recalculate(const FastSet<Key>& markedKeys, const FastSet<Key>& relinKeys,
                                                    const FastVector<Key>& observedKeys,
                                                    const FastSet<Key>& unusedIndices,
                                                    const boost::optional<FastMap<Key,int> >& constrainKeys,
                                                    ISAM2Result& result)
{
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
    BOOST_FOREACH(const Key key, markedKeys) { cout << key << " "; }
    cout << endl;
    cout << "observedKeys: ";
    BOOST_FOREACH(const Key key, observedKeys) { cout << key << " "; }
    cout << endl;
  }

  // 1. Remove top of Bayes tree and convert to a factor graph:
  // (a) For each affected variable, remove the corresponding clique and all parents up to the root.
  // (b) Store orphaned sub-trees \BayesTree_{O} of removed cliques.
  gttic(removetop);
  Cliques orphans;
  GaussianBayesNet affectedBayesNet;
  this->removeTop(markedKeys, affectedBayesNet, orphans);
  gttoc(removetop);

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
  gttic(affectedKeys);
  FastList<Key> affectedKeys;
  BOOST_FOREACH(const ConditionalType::shared_ptr& conditional, affectedBayesNet)
    affectedKeys.insert(affectedKeys.end(), conditional->beginFrontals(), conditional->endFrontals());
  gttoc(affectedKeys);

  boost::shared_ptr<FastSet<Index> > affectedKeysSet(new FastSet<Key>()); // Will return this result

  if(affectedKeys.size() >= theta_.size() * batchThreshold)
  {
    // Do a batch step - reorder and relinearize all variables
    gttic(batch);

    gttic(add_keys);
    br::copy(variableIndex_ | br::map_keys, std::inserter(*affectedKeysSet, affectedKeysSet->end()));
    gttoc(add_keys);

    gttic(ordering);
    Ordering order;
    if(constrainKeys)
    {
      order = Ordering::COLAMDConstrained(variableIndex_, *constrainKeys);
    }
    else
    {
      if(theta_.size() > observedKeys.size())
      {
        // Only if some variables are unconstrained
        FastMap<Key, int> constraintGroups;
        BOOST_FOREACH(Key var, observedKeys)
          constraintGroups[var] = 1;
        order = Ordering::COLAMDConstrained(variableIndex_, constraintGroups);
      }
      else
      {
        order = Ordering::COLAMD(variableIndex_);
      }
    }

    gttic(linearize);
    GaussianFactorGraph linearized = *nonlinearFactors_.linearize(theta_);
    if(params_.cacheLinearizedFactors)
      linearFactors_ = linearized;
    gttoc(linearize);

    gttic(eliminate);
    ISAM2BayesTree bayesTree = *ISAM2JunctionTree(GaussianEliminationTree(linearized, variableIndex_, order))
      .eliminate(params_.getEliminationFunction()).first;
    gttoc(eliminate);

    gttic(insert);
    this->clear();
    BOOST_FOREACH(const sharedNode& root, bayesTree.roots())
      this->insertRoot(root);
    gttoc(insert);

    result.variablesReeliminated = affectedKeysSet->size();
    result.factorsRecalculated = nonlinearFactors_.size();

    lastAffectedMarkedCount = markedKeys.size();
    lastAffectedVariableCount = affectedKeysSet->size();
    lastAffectedFactorCount = linearized.size();

    // Reeliminated keys for detailed results
    if(params_.enableDetailedResults) {
      BOOST_FOREACH(Key key, theta_.keys()) {
        result.detail->variableStatus[key].isReeliminated = true;
      }
    }

    gttoc(batch);

  } else {

    gttic(incremental);

    // 2. Add the new factors \Factors' into the resulting factor graph
    FastList<Key> affectedAndNewKeys;
    affectedAndNewKeys.insert(affectedAndNewKeys.end(), affectedKeys.begin(), affectedKeys.end());
    affectedAndNewKeys.insert(affectedAndNewKeys.end(), observedKeys.begin(), observedKeys.end());
    gttic(relinearizeAffected);
    GaussianFactorGraph factors(*relinearizeAffectedFactors(affectedAndNewKeys, relinKeys));
    if(debug) factors.print("Relinearized factors: ");
    gttoc(relinearizeAffected);

    if(debug) { cout << "Affected keys: "; BOOST_FOREACH(const Index key, affectedKeys) { cout << key << " "; } cout << endl; }

    // Reeliminated keys for detailed results
    if(params_.enableDetailedResults) {
      BOOST_FOREACH(Index key, affectedAndNewKeys) {
        result.detail->variableStatus[key].isReeliminated = true;
      }
    }

    result.variablesReeliminated = affectedAndNewKeys.size();
    result.factorsRecalculated = factors.size();
    lastAffectedMarkedCount = markedKeys.size();
    lastAffectedVariableCount = affectedKeys.size();
    lastAffectedFactorCount = factors.size();

#ifdef PRINT_STATS
    // output for generating figures
    cout << "linear: #markedKeys: " << markedKeys.size() << " #affectedVariables: " << affectedKeys.size()
              << " #affectedFactors: " << factors.size() << " maxCliqueSize: " << maxClique
              << " avgCliqueSize: " << avgClique << " #Cliques: " << numCliques << " nnzR: " << nnzR << endl;
#endif

    gttic(cached);
    // add the cached intermediate results from the boundary of the orphans ...
    GaussianFactorGraph cachedBoundary = getCachedBoundaryFactors(orphans);
    if(debug) cachedBoundary.print("Boundary factors: ");
    factors.push_back(cachedBoundary);
    gttoc(cached);

    // END OF COPIED CODE

    // 3. Re-order and eliminate the factor graph into a Bayes net (Algorithm [alg:eliminate]), and re-assemble into a new Bayes tree (Algorithm [alg:BayesTree])

    gttic(reorder_and_eliminate);

    gttic(list_to_set);
    // create a partial reordering for the new and contaminated factors
    // markedKeys are passed in: those variables will be forced to the end in the ordering
    affectedKeysSet->insert(markedKeys.begin(), markedKeys.end());
    affectedKeysSet->insert(affectedKeys.begin(), affectedKeys.end());
    gttoc(list_to_set);

    gttic(PartialSolve);
    Impl::ReorderingMode reorderingMode;
    reorderingMode.nFullSystemVars = variableIndex_.size();
    reorderingMode.algorithm = Impl::ReorderingMode::COLAMD;
    reorderingMode.constrain = Impl::ReorderingMode::CONSTRAIN_LAST;
    if(constrainKeys) {
      reorderingMode.constrainedKeys = *constrainKeys;
    } else {
      reorderingMode.constrainedKeys = FastMap<Key,int>();
      BOOST_FOREACH(Key var, observedKeys)
        reorderingMode.constrainedKeys->insert(make_pair(var, 1));
    }
    FastSet<Key> affectedUsedKeys = *affectedKeysSet; // Remove unused keys from the set we pass to PartialSolve
    BOOST_FOREACH(Key unused, unusedIndices)
      affectedUsedKeys.erase(unused);
    // Remove unaffected keys from the constraints
    FastMap<Key,int>::iterator iter = reorderingMode.constrainedKeys->begin();
    while(iter != reorderingMode.constrainedKeys->end())
      if(affectedUsedKeys.find(iter->first) == affectedUsedKeys.end())
        reorderingMode.constrainedKeys->erase(iter++);
      else
        ++iter;
    Impl::PartialSolveResult partialSolveResult =
        Impl::PartialSolve(factors, affectedUsedKeys, reorderingMode, (params_.factorization == ISAM2Params::QR));
    gttoc(PartialSolve);

    gttoc(reorder_and_eliminate);

    gttic(reassemble);
    if(partialSolveResult.bayesTree)
      BOOST_FOREACH(const sharedNode& root, *partialSolveResult.bayesTree.roots())
        this->insertRoot(root);
    gttoc(reassemble);

    // 4. The orphans have already been inserted during elimination

    gttoc(incremental);
  }

  // Root clique variables for detailed results
  if(params_.enableDetailedResults)
    BOOST_FOREACH(const sharedNode& root, this->roots())
      BOOST_FOREACH(Key var, *root)
        result.detail->variableStatus[var].inRootClique = true;

  return affectedKeysSet;
}

/* ************************************************************************* */
ISAM2Result ISAM2::update(
    const NonlinearFactorGraph& newFactors, const Values& newTheta, const FastVector<size_t>& removeFactorIndices,
    const boost::optional<FastMap<Key,int> >& constrainedKeys, const boost::optional<FastList<Key> >& noRelinKeys,
    const boost::optional<FastList<Key> >& extraReelimKeys, bool force_relinearize) {

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
    gttic(updateDelta);
    updateDelta(disableReordering);
    gttoc(updateDelta);
  }

  gttic(push_back_factors);
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
  variableIndex_.remove(removeFactorIndices, removeFactors);

  // Compute unused keys and indices
  FastSet<Key> unusedKeys;
  FastSet<Index> unusedIndices;
  {
    // Get keys from removed factors and new factors, and compute unused keys,
    // i.e., keys that are empty now and do not appear in the new factors.
    FastSet<Key> removedAndEmpty;
    BOOST_FOREACH(Key key, removeFactors.keys()) {
      if(variableIndex_[key].empty())
        removedAndEmpty.insert(removedAndEmpty.end(), key);
    }
    FastSet<Key> newFactorSymbKeys = newFactors.keys();
    std::set_difference(removedAndEmpty.begin(), removedAndEmpty.end(),
      newFactorSymbKeys.begin(), newFactorSymbKeys.end(), std::inserter(unusedKeys, unusedKeys.end()));

    // Get indices for unused keys
    BOOST_FOREACH(Key key, unusedKeys) {
      unusedIndices.insert(unusedIndices.end(), key);
    }
  }
  gttoc(push_back_factors);

  gttic(add_new_variables);
  // 2. Initialize any new variables \Theta_{new} and add \Theta:=\Theta\cup\Theta_{new}.
  Impl::AddVariables(newTheta, theta_, delta_, deltaNewton_, RgProd_, deltaReplacedMask_);
  // New keys for detailed results
  if(params_.enableDetailedResults) {
    BOOST_FOREACH(Key key, newTheta.keys()) { result.detail->variableStatus[key].isNew = true; } }
  gttoc(add_new_variables);

  gttic(evaluate_error_before);
  if(params_.evaluateNonlinearError)
    result.errorBefore.reset(nonlinearFactors_.error(calculateEstimate()));
  gttoc(evaluate_error_before);

  gttic(gather_involved_keys);
  // 3. Mark linear update
  FastSet<Key> markedKeys = newFactors.keys(); // Get keys from new factors
  // Also mark keys involved in removed factors
  {
    FastSet<Index> markedRemoveKeys = removeFactors.keys(); // Get keys involved in removed factors
    markedKeys.insert(markedRemoveKeys.begin(), markedRemoveKeys.end()); // Add to the overall set of marked keys
  }
  // Also mark any provided extra re-eliminate keys
  if(extraReelimKeys) {
    BOOST_FOREACH(Key key, *extraReelimKeys) {
      markedKeys.insert(key);
    }
  }

  // Observed keys for detailed results
  if(params_.enableDetailedResults) {
    BOOST_FOREACH(Key key, markedKeys) {
      result.detail->variableStatus[key].isObserved = true;
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
  gttoc(gather_involved_keys);

  // Check relinearization if we're at the nth step, or we are using a looser loop relin threshold
  FastSet<Index> relinKeys;
  if (relinearizeThisStep) {
    gttic(gather_relinearize_keys);
    // 4. Mark keys in \Delta above threshold \beta: J=\{\Delta_{j}\in\Delta|\Delta_{j}\geq\beta\}.
    if(params_.enablePartialRelinearizationCheck)
      relinKeys = Impl::CheckRelinearizationPartial(roots_, delta_, params_.relinearizeThreshold);
    else
      relinKeys = Impl::CheckRelinearizationFull(delta_, params_.relinearizeThreshold);
    if(disableReordering) relinKeys = Impl::CheckRelinearizationFull(delta_, 0.0); // This is used for debugging

    // Remove from relinKeys any keys whose linearization points are fixed
    BOOST_FOREACH(Key key, fixedVariables_) {
      relinKeys.erase(key);
    }
    if(noRelinKeys) {
      BOOST_FOREACH(Key key, *noRelinKeys) {
        relinKeys.erase(key);
      }
    }

    // Above relin threshold keys for detailed results
    if(params_.enableDetailedResults) {
      BOOST_FOREACH(Key key, relinKeys) {
        result.detail->variableStatus[key].isAboveRelinThreshold = true;
        result.detail->variableStatus[key].isRelinearized = true; } }

    // Add the variables being relinearized to the marked keys
    FastSet<Key> markedRelinMask;
    BOOST_FOREACH(const Key key, relinKeys)
      markedRelinMask.insert(key);
    markedKeys.insert(relinKeys.begin(), relinKeys.end());
    gttoc(gather_relinearize_keys);

    gttic(fluid_find_all);
    // 5. Mark all cliques that involve marked variables \Theta_{J} and all their ancestors.
    if (!relinKeys.empty()) {
      BOOST_FOREACH(const sharedClique& root, roots_)
        // add other cliques that have the marked ones in the separator
        Impl::FindAll(root, markedKeys, markedRelinMask);

      // Relin involved keys for detailed results
      if(params_.enableDetailedResults) {
        FastSet<Index> involvedRelinKeys;
        BOOST_FOREACH(const sharedClique& root, roots_)
          Impl::FindAll(root, involvedRelinKeys, markedRelinMask);
        BOOST_FOREACH(Key key, involvedRelinKeys) {
          if(!result.detail->variableStatus[key].isAboveRelinThreshold) {
            result.detail->variableStatus[key].isRelinearizeInvolved = true;
            result.detail->variableStatus[key].isRelinearized = true; } }
      }
    }
    gttoc(fluid_find_all);

    gttic(expmap);
    // 6. Update linearization point for marked variables: \Theta_{J}:=\Theta_{J}+\Delta_{J}.
    if (!relinKeys.empty())
      Impl::ExpmapMasked(theta_, delta_, markedRelinMask, delta_);
    gttoc(expmap);

    result.variablesRelinearized = markedKeys.size();
  } else {
    result.variablesRelinearized = 0;
  }

  gttic(linearize_new);
  // 7. Linearize new factors
  if(params_.cacheLinearizedFactors) {
    gttic(linearize);
    GaussianFactorGraph::shared_ptr linearFactors = newFactors.linearize(theta_);
    linearFactors_.push_back(*linearFactors);
    assert(nonlinearFactors_.size() == linearFactors_.size());
    gttoc(linearize);

    gttic(augment_VI);
    // Augment the variable index with the new factors
    variableIndex_.augment(*linearFactors); // TODO: move this to a better place now
    gttoc(augment_VI);
  } else {
    variableIndex_.augment(newFactors);
  }
  gttoc(linearize_new);

  gttic(recalculate);
  // 8. Redo top of Bayes tree
  boost::shared_ptr<FastSet<Key> > replacedKeys;
  if(!markedKeys.empty() || !observedKeys.empty())
    replacedKeys = recalculate(markedKeys, relinKeys, observedKeys, unusedIndices, constrainedKeys, result);

  // Update replaced keys mask (accumulates until back-substitution takes place)
  if(replacedKeys) {
    BOOST_FOREACH(const Key var, *replacedKeys) {
      deltaReplacedMask_[var] = true; } }
  gttoc(recalculate);

  // After the top of the tree has been redone and may have index gaps from
  // unused keys, condense the indices to remove gaps by rearranging indices
  // in all data structures.
  if(!unusedKeys.empty()) {
    gttic(remove_variables);
    Impl::RemoveVariables(unusedKeys, root_, theta_, variableIndex_, delta_, deltaNewton_, RgProd_,
        deltaReplacedMask_, Base::nodes_, linearFactors_, fixedVariables_);
    gttoc(remove_variables);
  }
  result.cliques = this->nodes().size();
  deltaDoglegUptodate_ = false;
  deltaUptodate_ = false;

  gttic(evaluate_error_after);
  if(params_.evaluateNonlinearError)
    result.errorAfter.reset(nonlinearFactors_.error(calculateEstimate()));
  gttoc(evaluate_error_after);

  return result;
}

/* ************************************************************************* */
void ISAM2::marginalizeLeaves(const FastList<Key>& leafKeysList)
{
  // Convert to ordered set
  FastSet<Key> leafKeys(leafKeysList.begin(), leafKeysList.end());

  // Keep track of marginal factors - map from clique to the marginal factors
  // that should be incorporated into it, passed up from it's children.
  multimap<sharedClique, GaussianFactor::shared_ptr> marginalFactors;

  // Remove each variable and its subtrees
  BOOST_REVERSE_FOREACH(Key j, leafKeys) {
    if(nodes_.exists(j)) { // If the index was not already removed by removing another subtree
      sharedClique clique = nodes_[j];

      // See if we should remove the whole clique
      bool marginalizeEntireClique = true;
      BOOST_FOREACH(Key frontal, clique->conditional()->frontals()) {
        if(!leafKeys.exists(frontal)) {
          marginalizeEntireClique = false;
          break; } }

      // Remove either the whole clique or part of it
      if(marginalizeEntireClique) {
        // Remove the whole clique and its subtree, and keep the marginal factor.
        GaussianFactor::shared_ptr marginalFactor = clique->cachedFactor();
        // We do not need the marginal factors associated with this clique
        // because their information is already incorporated in the new
        // marginal factor.  So, now associate this marginal factor with the
        // parent of this clique.
        marginalFactors.insert(make_pair(clique->parent(), marginalFactor));
        // Now remove this clique and its subtree - all of its marginal
        // information has been stored in marginalFactors.
        const Cliques removedCliques = this->removeSubtree(clique); // Remove the subtree and throw away the cliques
        BOOST_FOREACH(const sharedClique& removedClique, removedCliques) {
          marginalFactors.erase(removedClique);
          BOOST_FOREACH(Key frontal, removedClique->conditional()->frontals()) {
            if(!leafKeys.exists(frontal))
              throw runtime_error("Requesting to marginalize variables that are not leaves, the ISAM2 object is now in an inconsistent state so should no longer be used."); }
        }
      }
      else {
        // Reeliminate the current clique and the marginals from its children,
        // then keep only the marginal on the non-marginalized variables.  We
        // get the childrens' marginals from any existing children, plus
        // the marginals from the marginalFactors multimap, which come from any
        // subtrees already marginalized out.
        
        // Add child marginals and remove marginalized subtrees
        GaussianFactorGraph graph;
        FastSet<size_t> factorsInSubtreeRoot;
        Cliques subtreesToRemove;
        BOOST_FOREACH(const sharedClique& child, clique->children) {
          // Remove subtree if child depends on any marginalized keys
          BOOST_FOREACH(Key parent, child->conditional()->parents()) {
            if(leafKeys.exists(parent)) {
              subtreesToRemove.push_back(child);
              graph.push_back(child->cachedFactor()); // Add child marginal
              break;
            }
          }
        }
        Cliques childrenRemoved;
        BOOST_FOREACH(const sharedClique& childToRemove, subtreesToRemove) {
          const Cliques removedCliques = this->removeSubtree(childToRemove); // Remove the subtree and throw away the cliques
          childrenRemoved.insert(childrenRemoved.end(), removedCliques.begin(), removedCliques.end());
          BOOST_FOREACH(const sharedClique& removedClique, removedCliques) {
            marginalFactors.erase(removedClique);
            BOOST_FOREACH(Key frontal, removedClique->conditional()->frontals()) {
              if(!leafKeys.exists(frontal))
                throw runtime_error("Requesting to marginalize variables that are not leaves, the ISAM2 object is now in an inconsistent state so should no longer be used."); }
          }
        }

        // Gather remaining children after we removed marginalized subtrees
        vector<sharedClique> orphans(clique->children.begin(), clique->children.end());

        // Add the factors that are pulled into the current clique by the marginalized variables.
        // These are the factors that involve *marginalized* frontal variables in this clique
        // but do not involve frontal variables of any of its children.
        FastSet<size_t> factorsFromMarginalizedInClique;
        BOOST_FOREACH(Key frontal, clique->conditional()->frontals()) {
          if(leafKeys.exists(frontal))
            factorsFromMarginalizedInClique.insert(variableIndex_[frontal].begin(), variableIndex_[frontal].end()); }
        BOOST_FOREACH(const sharedClique& removedChild, childrenRemoved) {
          BOOST_FOREACH(Index indexInClique, removedChild->conditional()->frontals()) {
            BOOST_FOREACH(size_t factorInvolving, variableIndex_[indexInClique]) {
              factorsFromMarginalizedInClique.erase(factorInvolving); } } }
        BOOST_FOREACH(size_t i, factorsFromMarginalizedInClique) {
          graph.push_back(nonlinearFactors_[i]->linearize(theta_)); }

        // Remove the current clique
        sharedClique parent = clique->parent();
        this->removeClique(clique);

        // Reeliminate the linear graph to get the marginal and discard the conditional
        const FastSet<Index> cliqueFrontals(clique->conditional()->beginFrontals(), clique->conditional()->endFrontals());
        FastSet<Index> cliqueFrontalsToEliminate;
        std::set_intersection(cliqueFrontals.begin(), cliqueFrontals.end(), leafKeys.begin(), leafKeys.end(),
          std::inserter(cliqueFrontalsToEliminate, cliqueFrontalsToEliminate.end()));
        vector<Index> cliqueFrontalsToEliminateV(cliqueFrontalsToEliminate.begin(), cliqueFrontalsToEliminate.end());
        pair<GaussianConditional::shared_ptr, GaussianFactor::shared_ptr> eliminationResult1 =
          params_.getEliminationFunction()(graph, Ordering(cliqueFrontalsToEliminateV));

        // Add the resulting marginal
        if(eliminationResult1.second)
          marginalFactors.insert(make_pair(clique, eliminationResult1.second));

        // Recover the conditional on the remaining subset of frontal variables
        // of this clique being partially marginalized.
        GaussianConditional::iterator jPosition = std::find(
          clique->conditional()->beginFrontals(), clique->conditional()->endFrontals(), j);
        GaussianFactorGraph graph2;
        graph2.push_back(clique->conditional());
        GaussianFactorGraph::EliminationResult eliminationResult2 = 
          params_.getEliminationFunction()(graph2, Ordering(
          clique->conditional()->beginFrontals(), jPosition));
        GaussianFactorGraph graph3;
        graph3.push_back(eliminationResult2.second);
        GaussianFactorGraph::EliminationResult eliminationResult3 = 
          params_.getEliminationFunction()(graph3, Ordering(jPosition, clique->conditional()->endFrontals()));
        sharedClique newClique = boost::make_shared<Clique>(make_pair(eliminationResult3.first, clique->cachedFactor()));

        // Add the marginalized clique to the BayesTree
        this->addClique(newClique, parent);

        // Add the orphans
        BOOST_FOREACH(const sharedClique& orphan, orphans) {
          this->addClique(orphan, newClique); }
      }
    }
  }

  // At this point we have updated the BayesTree, now update the remaining iSAM2 data structures

  // Gather factors to add - the new marginal factors
  GaussianFactorGraph factorsToAdd;
  typedef pair<sharedClique, GaussianFactor::shared_ptr> Clique_Factor;
  BOOST_FOREACH(const Clique_Factor& clique_factor, marginalFactors) {
    if(clique_factor.second)
      factorsToAdd.push_back(clique_factor.second);
    nonlinearFactors_.push_back(boost::make_shared<LinearContainerFactor>(
      clique_factor.second, ordering_));
    if(params_.cacheLinearizedFactors)
      linearFactors_.push_back(clique_factor.second);
    BOOST_FOREACH(Index factorIndex, *clique_factor.second) {
      fixedVariables_.insert(ordering_.key(factorIndex)); }
  }
  variableIndex_.augment(factorsToAdd); // Augment the variable index

  // Remove the factors to remove that have been summarized in the newly-added marginal factors
  FastSet<size_t> factorIndicesToRemove;
  BOOST_FOREACH(Index j, indices) {
    factorIndicesToRemove.insert(variableIndex_[j].begin(), variableIndex_[j].end()); }
  vector<size_t> removedFactorIndices;
  SymbolicFactorGraph removedFactors;
  BOOST_FOREACH(size_t i, factorIndicesToRemove) {
    removedFactorIndices.push_back(i);
    removedFactors.push_back(nonlinearFactors_[i]->symbolic(ordering_));
    nonlinearFactors_.remove(i);
    if(params_.cacheLinearizedFactors)
      linearFactors_.remove(i);
  }
  variableIndex_.remove(removedFactorIndices, removedFactors);

  // Remove the marginalized variables
  Impl::RemoveVariables(FastSet<Key>(leafKeys.begin(), leafKeys.end()), root_, theta_, variableIndex_, delta_, deltaNewton_, RgProd_,
    deltaReplacedMask_, ordering_, nodes_, linearFactors_, fixedVariables_);
}

/* ************************************************************************* */
void ISAM2::updateDelta(bool forceFullSolve) const {

  if(params_.optimizationParams.type() == typeid(ISAM2GaussNewtonParams)) {
    // If using Gauss-Newton, update with wildfireThreshold
    const ISAM2GaussNewtonParams& gaussNewtonParams =
        boost::get<ISAM2GaussNewtonParams>(params_.optimizationParams);
    const double effectiveWildfireThreshold = forceFullSolve ? 0.0 : gaussNewtonParams.wildfireThreshold;
    gttic(Wildfire_update);
    lastBacksubVariableCount = Impl::UpdateDelta(this->root(), deltaReplacedMask_, delta_, effectiveWildfireThreshold);
    gttoc(Wildfire_update);

  } else if(params_.optimizationParams.type() == typeid(ISAM2DoglegParams)) {
    // If using Dogleg, do a Dogleg step
    const ISAM2DoglegParams& doglegParams =
        boost::get<ISAM2DoglegParams>(params_.optimizationParams);

    // Do one Dogleg iteration
    gttic(Dogleg_Iterate);
    DoglegOptimizerImpl::IterationResult doglegResult(DoglegOptimizerImpl::Iterate(
        *doglegDelta_, doglegParams.adaptationMode, *this, nonlinearFactors_, theta_, ordering_, nonlinearFactors_.error(theta_), doglegParams.verbose));
    gttoc(Dogleg_Iterate);

    gttic(Copy_dx_d);
    // Update Delta and linear step
    doglegDelta_ = doglegResult.Delta;
    delta_ = doglegResult.dx_d; // Copy the VectorValues containing with the linear solution
    gttoc(Copy_dx_d);
  }

  deltaUptodate_ = true;
}

/* ************************************************************************* */
Values ISAM2::calculateEstimate() const {
  // We use ExpmapMasked here instead of regular expmap because the former
  // handles Permuted<VectorValues>
  gttic(Copy_Values);
  Values ret(theta_);
  gttoc(Copy_Values);
  gttic(getDelta);
  const VectorValues& delta(getDelta());
  gttoc(getDelta);
  gttic(Expmap);
  vector<bool> mask(ordering_.size(), true);
  Impl::ExpmapMasked(ret, delta, ordering_, mask);
  gttoc(Expmap);
  return ret;
}

/* ************************************************************************* */
const Value& ISAM2::calculateEstimate(Key key) const {
  const Index index = getOrdering()[key];
  const Vector& delta = getDelta()[index];
  return *theta_.at(key).retract_(delta);
}

/* ************************************************************************* */
Values ISAM2::calculateBestEstimate() const {
  VectorValues delta(theta_.dims(ordering_));
  internal::optimizeInPlace<Base>(this->root(), delta);
  return theta_.retract(delta, ordering_);
}

/* ************************************************************************* */
Matrix ISAM2::marginalCovariance(Index key) const {
  return marginalFactor(ordering_[key],
    params_.factorization == ISAM2Params::QR ? EliminateQR : EliminatePreferCholesky)
    ->information().inverse();
}

/* ************************************************************************* */
const VectorValues& ISAM2::getDelta() const {
  if(!deltaUptodate_)
    updateDelta();
  return delta_;
}

/* ************************************************************************* */

VectorValues optimize(const ISAM2& isam) {
  gttic(allocateVectorValues);
  VectorValues delta = *allocateVectorValues(isam);
  gttoc(allocateVectorValues);
  optimizeInPlace(isam, delta);
  return delta;
}

/* ************************************************************************* */
void optimizeInPlace(const ISAM2& isam, VectorValues& delta) {
  // We may need to update the solution calculations
  if(!isam.deltaDoglegUptodate_) {
    gttic(UpdateDoglegDeltas);
    double wildfireThreshold = 0.0;
    if(isam.params().optimizationParams.type() == typeid(ISAM2GaussNewtonParams))
      wildfireThreshold = boost::get<ISAM2GaussNewtonParams>(isam.params().optimizationParams).wildfireThreshold;
    else if(isam.params().optimizationParams.type() == typeid(ISAM2DoglegParams))
      wildfireThreshold = boost::get<ISAM2DoglegParams>(isam.params().optimizationParams).wildfireThreshold;
    else
      assert(false);
    ISAM2::Impl::UpdateDoglegDeltas(isam, wildfireThreshold, isam.deltaReplacedMask_, isam.deltaNewton_, isam.RgProd_);
    isam.deltaDoglegUptodate_ = true;
    gttoc(UpdateDoglegDeltas);
  }

  gttic(copy_delta);
  delta = isam.deltaNewton_;
  gttoc(copy_delta);
}

/* ************************************************************************* */

VectorValues optimizeGradientSearch(const ISAM2& isam) {
  gttic(Allocate_VectorValues);
  VectorValues grad = *allocateVectorValues(isam);
  gttoc(Allocate_VectorValues);

  optimizeGradientSearchInPlace(isam, grad);

  return grad;
}

/* ************************************************************************* */
void optimizeGradientSearchInPlace(const ISAM2& isam, VectorValues& grad) {
  // We may need to update the solution calcaulations
  if(!isam.deltaDoglegUptodate_) {
    gttic(UpdateDoglegDeltas);
    double wildfireThreshold = 0.0;
    if(isam.params().optimizationParams.type() == typeid(ISAM2GaussNewtonParams))
      wildfireThreshold = boost::get<ISAM2GaussNewtonParams>(isam.params().optimizationParams).wildfireThreshold;
    else if(isam.params().optimizationParams.type() == typeid(ISAM2DoglegParams))
      wildfireThreshold = boost::get<ISAM2DoglegParams>(isam.params().optimizationParams).wildfireThreshold;
    else
      assert(false);
    ISAM2::Impl::UpdateDoglegDeltas(isam, wildfireThreshold, isam.deltaReplacedMask_, isam.deltaNewton_, isam.RgProd_);
    isam.deltaDoglegUptodate_ = true;
    gttoc(UpdateDoglegDeltas);
  }

  gttic(Compute_Gradient);
  // Compute gradient (call gradientAtZero function, which is defined for various linear systems)
  gradientAtZero(isam, grad);
  double gradientSqNorm = grad.dot(grad);
  gttoc(Compute_Gradient);

  gttic(Compute_minimizing_step_size);
  // Compute minimizing step size
  double RgNormSq = isam.RgProd_.asVector().squaredNorm();
  double step = -gradientSqNorm / RgNormSq;
  gttoc(Compute_minimizing_step_size);

  gttic(Compute_point);
  // Compute steepest descent point
  scal(step, grad);
  gttoc(Compute_point);
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
