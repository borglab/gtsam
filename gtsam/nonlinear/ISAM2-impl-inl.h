/**
 * @file    ISAM2-impl-inl.h
 * @brief   Incremental update functionality (ISAM2) for BayesTree, with fluid relinearization.
 * @author  Michael Kaess, Richard Roberts
 */

#include <gtsam/base/FastSet.h>

#include <boost/foreach.hpp>

#include <vector>

namespace gtsam {

template<class CONDITIONAL, class VALUES>
struct ISAM2<CONDITIONAL, VALUES>::Impl {
  /**
   * Add new variables to the ISAM2 system.
   * @param newTheta Initial values for new variables
   * @param theta Current solution to be augmented with new initialization
   * @param delta Current linear delta to be augmented with zeros
   * @param ordering Current ordering to be augmented with new variables
   * @param nodes Current BayesTree::Nodes index to be augmented with slots for new variables
   */
  static void AddVariables(const VALUES& newTheta, VALUES& theta, Permuted<VectorValues>& delta, Ordering& ordering, typename Base::Nodes& nodes);

  /**
   * Extract the set of variable indices from a NonlinearFactorGraph.  For each Symbol
   * in each NonlinearFactor, obtains the index by calling ordering[symbol].
   * @param ordering The current ordering from which to obtain the variable indices
   * @param factors The factors from which to extract the variables
   * @return The set of variables indices from the factors
   */
  static FastSet<Index> IndicesFromFactors(const Ordering& ordering, const NonlinearFactorGraph<VALUES>& factors);

  /**
   * Find the set of variables to be relinearized according to relinearizeThreshold.
   * Any variables in the VectorValues delta whose vector magnitude is greater than
   * or equal to relinearizeThreshold are returned.
   * @param delta The linear delta to check against the threshold
   * @return The set of variable indices in delta whose magnitude is greater than or
   * equal to relinearizeThreshold
   */
  static FastSet<Index> CheckRelinearization(Permuted<VectorValues>& delta, double relinearizeThreshold);
};

/* ************************************************************************* */
struct _VariableAdder {
  Ordering& ordering;
  Permuted<VectorValues>& vconfig;
  _VariableAdder(Ordering& _ordering, Permuted<VectorValues>& _vconfig) : ordering(_ordering), vconfig(_vconfig) {}
  template<typename I>
  void operator()(I xIt) {
    const bool debug = ISDEBUG("ISAM2 AddVariables");
    Index var = vconfig->push_back_preallocated(zero(xIt->second.dim()));
    vconfig.permutation()[var] = var;
    ordering.insert(xIt->first, var);
    if(debug) cout << "Adding variable " << (string)xIt->first << " with order " << var << endl;
  }
};

/* ************************************************************************* */
template<class CONDITIONAL, class VALUES>
void ISAM2<CONDITIONAL,VALUES>::Impl::AddVariables(
    const VALUES& newTheta, VALUES& theta, Permuted<VectorValues>& delta, Ordering& ordering, typename Base::Nodes& nodes) {
  const bool debug = ISDEBUG("ISAM2 AddVariables");

  theta.insert(newTheta);
  if(debug) newTheta.print("The new variables are: ");
  // Add the new keys onto the ordering, add zeros to the delta for the new variables
  vector<Index> dims(newTheta.dims(*newTheta.orderingArbitrary(ordering.nVars())));
  if(debug) cout << "New variables have total dimensionality " << accumulate(dims.begin(), dims.end(), 0) << endl;
  delta.container().reserve(delta->size() + newTheta.size(), delta->dim() + accumulate(dims.begin(), dims.end(), 0));
  delta.permutation().resize(delta->size() + newTheta.size());
  {
    _VariableAdder vadder(ordering, delta);
    newTheta.apply(vadder);
    assert(delta.permutation().size() == delta.container().size());
    assert(delta.container().dim() == delta.container().dimCapacity());
    assert(ordering.nVars() == delta.size());
    assert(ordering.size() == delta.size());
  }
  assert(ordering.nVars() >= nodes.size());
  nodes.resize(ordering.nVars());
}

/* ************************************************************************* */
template<class CONDITIONAL, class VALUES>
FastSet<Index> ISAM2<CONDITIONAL,VALUES>::Impl::IndicesFromFactors(const Ordering& ordering, const NonlinearFactorGraph<VALUES>& factors) {
  FastSet<Index> indices;
  BOOST_FOREACH(const typename NonlinearFactor<VALUES>::shared_ptr& factor, factors) {
    BOOST_FOREACH(const Symbol& key, factor->keys()) {
      indices.insert(ordering[key]);
    }
  }
  return indices;
}

/* ************************************************************************* */
template<class CONDITIONAL, class VALUES>
FastSet<Index> ISAM2<CONDITIONAL,VALUES>::Impl::CheckRelinearization(Permuted<VectorValues>& delta, double relinearizeThreshold) {
  FastSet<Index> relinKeys;
  for(Index var=0; var<delta.size(); ++var) {
    double maxDelta = delta[var].lpNorm<Eigen::Infinity>();
    if(maxDelta >= relinearizeThreshold) {
      relinKeys.insert(var);
    }
  }
  return relinKeys;
}

}
