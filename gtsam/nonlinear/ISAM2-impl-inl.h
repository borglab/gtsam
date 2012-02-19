/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ISAM2-impl-inl.h
 * @brief   Incremental update functionality (ISAM2) for BayesTree, with fluid relinearization.
 * @author  Michael Kaess, Richard Roberts
 */

namespace gtsam {

using namespace std;

template<class CONDITIONAL, class GRAPH>
struct ISAM2<CONDITIONAL, GRAPH>::Impl {

  typedef ISAM2<CONDITIONAL, GRAPH> ISAM2Type;

  struct PartialSolveResult {
    typename ISAM2Type::sharedClique bayesTree;
    Permutation fullReordering;
    Permutation fullReorderingInverse;
  };

  struct ReorderingMode {
    size_t nFullSystemVars;
    enum { /*AS_ADDED,*/ COLAMD } algorithm;
    enum { NO_CONSTRAINT, CONSTRAIN_LAST } constrain;
    boost::optional<const FastSet<Index>&> constrainedKeys;
  };

  /**
   * Add new variables to the ISAM2 system.
   * @param newTheta Initial values for new variables
   * @param theta Current solution to be augmented with new initialization
   * @param delta Current linear delta to be augmented with zeros
   * @param ordering Current ordering to be augmented with new variables
   * @param nodes Current BayesTree::Nodes index to be augmented with slots for new variables
   */
  static void AddVariables(const Values& newTheta, Values& theta, Permuted<VectorValues>& delta, Ordering& ordering, typename Base::Nodes& nodes);

  /**
   * Extract the set of variable indices from a NonlinearFactorGraph.  For each Symbol
   * in each NonlinearFactor, obtains the index by calling ordering[symbol].
   * @param ordering The current ordering from which to obtain the variable indices
   * @param factors The factors from which to extract the variables
   * @return The set of variables indices from the factors
   */
  static FastSet<Index> IndicesFromFactors(const Ordering& ordering, const GRAPH& factors);

  /**
   * Find the set of variables to be relinearized according to relinearizeThreshold.
   * Any variables in the VectorValues delta whose vector magnitude is greater than
   * or equal to relinearizeThreshold are returned.
   * @param delta The linear delta to check against the threshold
   * @return The set of variable indices in delta whose magnitude is greater than or
   * equal to relinearizeThreshold
   */
  static FastSet<Index> CheckRelinearization(const Permuted<VectorValues>& delta, const Ordering& ordering, const ISAM2Params::RelinearizationThreshold& relinearizeThreshold);

  /**
   * Recursively search this clique and its children for marked keys appearing
   * in the separator, and add the *frontal* keys of any cliques whose
   * separator contains any marked keys to the set \c keys.  The purpose of
   * this is to discover the cliques that need to be redone due to information
   * propagating to them from cliques that directly contain factors being
   * relinearized.
   *
   * The original comment says this finds all variables directly connected to
   * the marked ones by measurements.  Is this true, because it seems like this
   * would also pull in variables indirectly connected through other frontal or
   * separator variables?
   *
   * Alternatively could we trace up towards the root for each variable here?
   */
  static void FindAll(typename ISAM2Clique<CONDITIONAL>::shared_ptr clique, FastSet<Index>& keys, const vector<bool>& markedMask);

  /**
   * Apply expmap to the given values, but only for indices appearing in
   * \c markedRelinMask.  Values are expmapped in-place.
   * \param [in][out] values The value to expmap in-place
   * \param delta The linear delta with which to expmap
   * \param ordering The ordering
   * \param mask Mask on linear indices, only \c true entries are expmapped
   * \param invalidateIfDebug If this is true, *and* NDEBUG is not defined,
   * expmapped deltas will be set to an invalid value (infinity) to catch bugs
   * where we might expmap something twice, or expmap it but then not
   * recalculate its delta.
   */
  static void ExpmapMasked(Values& values, const Permuted<VectorValues>& delta,
      const Ordering& ordering, const std::vector<bool>& mask,
      boost::optional<Permuted<VectorValues>&> invalidateIfDebug = boost::optional<Permuted<VectorValues>&>());

  /**
   * Reorder and eliminate factors.  These factors form a subset of the full
   * problem, so along with the BayesTree we get a partial reordering of the
   * problem that needs to be applied to the other data in ISAM2, which is the
   * VariableIndex, the delta, the ordering, and the orphans (including cached
   * factors).
   * \param factors The factors to eliminate, representing part of the full
   * problem.  This is permuted during use and so is cleared when this function
   * returns in order to invalidate it.
   * \param keys The set of indices used in \c factors.
   * \return The eliminated BayesTree and the permutation to be applied to the
   * rest of the ISAM2 data.
   */
  static PartialSolveResult PartialSolve(GaussianFactorGraph& factors, const FastSet<Index>& keys,
      const ReorderingMode& reorderingMode);

};

/* ************************************************************************* */
template<class CONDITIONAL, class GRAPH>
void ISAM2<CONDITIONAL,GRAPH>::Impl::AddVariables(
    const Values& newTheta, Values& theta, Permuted<VectorValues>& delta, Ordering& ordering, typename Base::Nodes& nodes) {
  const bool debug = ISDEBUG("ISAM2 AddVariables");

  theta.insert(newTheta);
  if(debug) newTheta.print("The new variables are: ");
  // Add the new keys onto the ordering, add zeros to the delta for the new variables
  std::vector<Index> dims(newTheta.dims(*newTheta.orderingArbitrary()));
  if(debug) cout << "New variables have total dimensionality " << accumulate(dims.begin(), dims.end(), 0) << endl;
  const size_t newDim = accumulate(dims.begin(), dims.end(), 0);
  const size_t originalDim = delta->dim();
  const size_t originalnVars = delta->size();
  delta.container().append(dims);
  delta.container().vector().segment(originalDim, newDim).operator=(Vector::Zero(newDim));
  delta.permutation().resize(originalnVars + newTheta.size());
  {
    Index nextVar = originalnVars;
    BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, newTheta) {
      delta.permutation()[nextVar] = nextVar;
      ordering.insert(key_value.first, nextVar);
      if(debug) cout << "Adding variable " << (string)key_value.first << " with order " << nextVar << endl;
      ++ nextVar;
    }
    assert(delta.permutation().size() == delta.container().size());
    assert(ordering.nVars() == delta.size());
    assert(ordering.size() == delta.size());
  }
  assert(ordering.nVars() >= nodes.size());
  nodes.resize(ordering.nVars());
}

/* ************************************************************************* */
template<class CONDITIONAL, class GRAPH>
FastSet<Index> ISAM2<CONDITIONAL,GRAPH>::Impl::IndicesFromFactors(const Ordering& ordering, const GRAPH& factors) {
  FastSet<Index> indices;
  BOOST_FOREACH(const typename NonlinearFactor::shared_ptr& factor, factors) {
    BOOST_FOREACH(Key key, factor->keys()) {
      indices.insert(ordering[key]);
    }
  }
  return indices;
}

/* ************************************************************************* */
template<class CONDITIONAL, class GRAPH>
FastSet<Index> ISAM2<CONDITIONAL,GRAPH>::Impl::CheckRelinearization(const Permuted<VectorValues>& delta, const Ordering& ordering, const ISAM2Params::RelinearizationThreshold& relinearizeThreshold) {
  FastSet<Index> relinKeys;

  if(relinearizeThreshold.type() == typeid(double)) {
    double threshold = boost::get<double>(relinearizeThreshold);
    for(Index var=0; var<delta.size(); ++var) {
      double maxDelta = delta[var].lpNorm<Eigen::Infinity>();
      if(maxDelta >= threshold) {
        relinKeys.insert(var);
      }
    }
  } else if(relinearizeThreshold.type() == typeid(FastMap<char,Vector>)) {
    const FastMap<char,Vector>& thresholds = boost::get<FastMap<char,Vector> >(relinearizeThreshold);
    BOOST_FOREACH(const Ordering::value_type& key_index, ordering) {
      const Vector& threshold = thresholds.find(key_index.first.chr())->second;
      Index j = key_index.second;
      if(threshold.rows() != delta[j].rows())
        throw std::invalid_argument("Relinearization threshold vector dimensionality passed into iSAM2 parameters does not match actual variable dimensionality");
      if((delta[j].array().abs() > threshold.array()).any())
        relinKeys.insert(j);
    }
  }

  return relinKeys;
}

/* ************************************************************************* */
template<class CONDITIONAL, class GRAPH>
void ISAM2<CONDITIONAL,GRAPH>::Impl::FindAll(typename ISAM2Clique<CONDITIONAL>::shared_ptr clique, FastSet<Index>& keys, const vector<bool>& markedMask) {
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
  BOOST_FOREACH(const typename ISAM2Clique<CONDITIONAL>::shared_ptr& child, clique->children_) {
    FindAll(child, keys, markedMask);
  }
}

/* ************************************************************************* */
template<class CONDITIONAL, class GRAPH>
void ISAM2<CONDITIONAL, GRAPH>::Impl::ExpmapMasked(Values& values, const Permuted<VectorValues>& delta,
    const Ordering& ordering, const vector<bool>& mask, boost::optional<Permuted<VectorValues>&> invalidateIfDebug) {
  // If debugging, invalidate if requested, otherwise do not invalidate.
  // Invalidating means setting expmapped entries to Inf, to trigger assertions
  // if we try to re-use them.
#ifdef NDEBUG
  invalidateIfDebug = boost::optional<Permuted<VectorValues>&>();
#endif

  assert(values.size() == ordering.size());
  Values::iterator key_value;
  Ordering::const_iterator key_index;
  for(key_value = values.begin(), key_index = ordering.begin();
      key_value != values.end() && key_index != ordering.end(); ++key_value, ++key_index) {
    assert(key_value->first == key_index->first);
    const Index var = key_index->second;
    if(ISDEBUG("ISAM2 update verbose")) {
      if(mask[var])
        cout << "expmap " << (string)key_value->first << " (j = " << var << "), delta = " << delta[var].transpose() << endl;
      else
        cout << "       " << (string)key_value->first << " (j = " << var << "), delta = " << delta[var].transpose() << endl;
    }
    assert(delta[var].size() == (int)key_value->second.dim());
    assert(delta[var].unaryExpr(&isfinite<double>).all());
    if(mask[var]) {
      Value* retracted = key_value->second.retract_(delta[var]);
      key_value->second = *retracted;
      retracted->deallocate_();
      if(invalidateIfDebug)
        (*invalidateIfDebug)[var].operator=(Vector::Constant(delta[var].rows(), numeric_limits<double>::infinity())); // Strange syntax to work with clang++ (bug in clang?)
    }
  }
}

/* ************************************************************************* */
template<class CONDITIONAL, class GRAPH>
typename ISAM2<CONDITIONAL, GRAPH>::Impl::PartialSolveResult
ISAM2<CONDITIONAL, GRAPH>::Impl::PartialSolve(GaussianFactorGraph& factors,
    const FastSet<Index>& keys, const ReorderingMode& reorderingMode) {

  static const bool debug = ISDEBUG("ISAM2 recalculate");

  PartialSolveResult result;

  tic(1,"select affected variables");
#ifndef NDEBUG
  // Debug check that all variables involved in the factors to be re-eliminated
  // are in affectedKeys, since we will use it to select a subset of variables.
  BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, factors) {
    BOOST_FOREACH(Index key, factor->keys()) {
      assert(find(keys.begin(), keys.end(), key) != keys.end());
    }
  }
#endif
  Permutation affectedKeysSelector(keys.size()); // Create a permutation that pulls the affected keys to the front
  Permutation affectedKeysSelectorInverse(keys.size() > 0 ? *keys.rbegin()+1 : 0 /*ordering_.nVars()*/); // And its inverse
#ifndef NDEBUG
  // If debugging, fill with invalid values that will trip asserts if dereferenced
  std::fill(affectedKeysSelectorInverse.begin(), affectedKeysSelectorInverse.end(), numeric_limits<Index>::max());
#endif
  { Index position=0; BOOST_FOREACH(Index var, keys) {
    affectedKeysSelector[position] = var;
    affectedKeysSelectorInverse[var] = position;
    ++ position; } }
  if(debug) affectedKeysSelector.print("affectedKeysSelector: ");
  if(debug) affectedKeysSelectorInverse.print("affectedKeysSelectorInverse: ");
  factors.permuteWithInverse(affectedKeysSelectorInverse);
  if(debug) factors.print("Factors to reorder/re-eliminate: ");
  toc(1,"select affected variables");
  tic(2,"variable index");
  VariableIndex affectedFactorsIndex(factors); // Create a variable index for the factors to be re-eliminated
  if(debug) affectedFactorsIndex.print("affectedFactorsIndex: ");
  toc(2,"variable index");
  tic(3,"ccolamd");
  vector<int> cmember(affectedKeysSelector.size(), 0);
  if(reorderingMode.constrain == ReorderingMode::CONSTRAIN_LAST) {
    assert(reorderingMode.constrainedKeys);
    if(keys.size() > reorderingMode.constrainedKeys->size()) {
      BOOST_FOREACH(Index var, *reorderingMode.constrainedKeys) {
        cmember[affectedKeysSelectorInverse[var]] = 1;
      }
    }
  }
  Permutation::shared_ptr affectedColamd(Inference::PermutationCOLAMD_(affectedFactorsIndex, cmember));
  toc(3,"ccolamd");
  tic(4,"ccolamd permutations");
  Permutation::shared_ptr affectedColamdInverse(affectedColamd->inverse());
  if(debug) affectedColamd->print("affectedColamd: ");
  if(debug) affectedColamdInverse->print("affectedColamdInverse: ");
  result.fullReordering =
      *Permutation::Identity(reorderingMode.nFullSystemVars).partialPermutation(affectedKeysSelector, *affectedColamd);
  result.fullReorderingInverse =
      *Permutation::Identity(reorderingMode.nFullSystemVars).partialPermutation(affectedKeysSelector, *affectedColamdInverse);
  if(debug) result.fullReordering.print("partialReordering: ");
  toc(4,"ccolamd permutations");

  tic(5,"permute affected variable index");
  affectedFactorsIndex.permute(*affectedColamd);
  toc(5,"permute affected variable index");

  tic(6,"permute affected factors");
  factors.permuteWithInverse(*affectedColamdInverse);
  toc(6,"permute affected factors");

  if(debug) factors.print("Colamd-ordered affected factors: ");

#ifndef NDEBUG
    VariableIndex fromScratchIndex(factors);
    assert(assert_equal(fromScratchIndex, affectedFactorsIndex));
#endif

  // eliminate into a Bayes net
  tic(7,"eliminate");
  JunctionTree<GaussianFactorGraph, typename ISAM2Type::Clique> jt(factors, affectedFactorsIndex);
  result.bayesTree = jt.eliminate(EliminatePreferLDL);
  if(debug && result.bayesTree) {
    if(boost::dynamic_pointer_cast<ISAM2Clique<CONDITIONAL> >(result.bayesTree))
      cout << "Is an ISAM2 clique" << endl;
    cout << "Re-eliminated BT:\n";
    result.bayesTree->printTree("");
  }
  toc(7,"eliminate");

  tic(8,"permute eliminated");
  if(result.bayesTree) result.bayesTree->permuteWithInverse(affectedKeysSelector);
  if(debug && result.bayesTree) {
    cout << "Full var-ordered eliminated BT:\n";
    result.bayesTree->printTree("");
  }
  toc(8,"permute eliminated");

  return result;
}

}
