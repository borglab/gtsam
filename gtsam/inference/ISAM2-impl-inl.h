/**
 * @file    ISAM2-impl-inl.h
 * @brief   Incremental update functionality (ISAM2) for BayesTree, with fluid relinearization.
 * @author  Michael Kaess, Richard Roberts
 */

namespace gtsam {

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

}
