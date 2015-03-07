/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ISAM2-impl.cpp
 * @brief   Incremental update functionality (ISAM2) for BayesTree, with fluid relinearization.
 * @author  Michael Kaess
 * @author  Richard Roberts
 */

#include <gtsam/nonlinear/ISAM2-impl.h>
#include <gtsam/nonlinear/Symbol.h> // for selective linearization thresholds
#include <gtsam/base/debug.h>
#include <functional>
#include <boost/range/adaptors.hpp>
#include <boost/range/algorithm.hpp>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
void ISAM2::Impl::AddVariables(
    const Values& newTheta, Values& theta, VectorValues& delta,
    VectorValues& deltaNewton, VectorValues& deltaGradSearch, vector<bool>& replacedKeys,
    Ordering& ordering, const KeyFormatter& keyFormatter) {
  const bool debug = ISDEBUG("ISAM2 AddVariables");

  theta.insert(newTheta);
  if(debug) newTheta.print("The new variables are: ");
  // Add the new keys onto the ordering, add zeros to the delta for the new variables
  std::vector<Index> dims(newTheta.dims(*newTheta.orderingArbitrary()));
  if(debug) cout << "New variables have total dimensionality " << accumulate(dims.begin(), dims.end(), 0) << endl;
  const size_t newDim = accumulate(dims.begin(), dims.end(), 0);
  const size_t originalDim = delta.dim();
  const size_t originalnVars = delta.size();
  delta.append(dims);
  delta.vector().segment(originalDim, newDim).operator=(Vector::Zero(newDim));
  deltaNewton.append(dims);
  deltaNewton.vector().segment(originalDim, newDim).operator=(Vector::Zero(newDim));
  deltaGradSearch.append(dims);
  deltaGradSearch.vector().segment(originalDim, newDim).operator=(Vector::Zero(newDim));
  {
    Index nextVar = originalnVars;
    BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, newTheta) {
      ordering.insert(key_value.key, nextVar);
      if(debug) cout << "Adding variable " << keyFormatter(key_value.key) << " with order " << nextVar << endl;
      ++ nextVar;
    }
    assert(ordering.nVars() == delta.size());
    assert(ordering.size() == delta.size());
  }
  replacedKeys.resize(ordering.nVars(), false);
}

/* ************************************************************************* */
void ISAM2::Impl::RemoveVariables(const FastSet<Key>& unusedKeys, const ISAM2Clique::shared_ptr& root,
																	Values& theta, VariableIndex& variableIndex,
																	VectorValues& delta, VectorValues& deltaNewton, VectorValues& deltaGradSearch,
																	std::vector<bool>& replacedKeys, Ordering& ordering, Base::Nodes& nodes,
																	GaussianFactorGraph& linearFactors) {

		 // Get indices of unused keys
		 vector<Index> unusedIndices;  unusedIndices.reserve(unusedKeys.size());
		 BOOST_FOREACH(Key key, unusedKeys) { unusedIndices.push_back(ordering[key]); }

		 // Create a permutation that shifts the unused variables to the end
		 Permutation unusedToEnd = Permutation::PushToBack(unusedIndices, delta.size());
		 Permutation unusedToEndInverse = *unusedToEnd.inverse();

		 // Use the permutation to remove unused variables while shifting all the others to take up the space
		 variableIndex.permuteInPlace(unusedToEnd);
		 variableIndex.removeUnusedAtEnd(unusedIndices.size());
		 {
			 // Create a vector of variable dimensions with the unused ones removed
			 // by applying the unusedToEnd permutation to the original vector of
			 // variable dimensions.  We only allocate space in the shifted dims
			 // vector for the used variables, so that the unused ones are dropped
			 // when the permutation is applied.
			 vector<size_t> originalDims = delta.dims();
			 vector<size_t> dims(delta.size() - unusedIndices.size());
			 unusedToEnd.applyToCollection(dims, originalDims);

			 // Copy from the old data structures to new ones, only iterating up to
			 // the number of used variables, and applying the unusedToEnd permutation
			 // in order to remove the unused variables.
			 VectorValues newDelta(dims);
			 VectorValues newDeltaNewton(dims);
			 VectorValues newDeltaGradSearch(dims);
			 std::vector<bool> newReplacedKeys(replacedKeys.size() - unusedIndices.size());
			 Base::Nodes newNodes(replacedKeys.size() - unusedIndices.size());

			 for(size_t j = 0; j < dims.size(); ++j) {
				 newDelta[j] = delta[unusedToEnd[j]];
				 newDeltaNewton[j] = deltaNewton[unusedToEnd[j]];
				 newDeltaGradSearch[j] = deltaGradSearch[unusedToEnd[j]];
				 newReplacedKeys[j] = replacedKeys[unusedToEnd[j]];
				 newNodes[j] = nodes[unusedToEnd[j]];
			 }

			 // Swap the new data structures with the outputs of this function
			 delta.swap(newDelta);
			 deltaNewton.swap(newDeltaNewton);
			 deltaGradSearch.swap(newDeltaGradSearch);
			 replacedKeys.swap(newReplacedKeys);
			 nodes.swap(newNodes);
		 }

		 // Reorder and remove from ordering and solution
		 ordering.permuteWithInverse(unusedToEndInverse);
		 BOOST_REVERSE_FOREACH(Key key, unusedKeys) {
		   ordering.pop_back(key);
			 theta.erase(key);
		 }

		 // Finally, permute references to variables
		 if(root)
			 root->permuteWithInverse(unusedToEndInverse);
		 linearFactors.permuteWithInverse(unusedToEndInverse);
}

/* ************************************************************************* */
FastSet<Index> ISAM2::Impl::IndicesFromFactors(const Ordering& ordering, const NonlinearFactorGraph& factors) {
  FastSet<Index> indices;
  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, factors) {
    BOOST_FOREACH(Key key, factor->keys()) {
      indices.insert(ordering[key]);
    }
  }
  return indices;
}

/* ************************************************************************* */
FastSet<Index> ISAM2::Impl::CheckRelinearizationFull(const VectorValues& delta, const Ordering& ordering,
    const ISAM2Params::RelinearizationThreshold& relinearizeThreshold, const KeyFormatter& keyFormatter) {
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
      const Vector& threshold = thresholds.find(Symbol(key_index.first).chr())->second;
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
void CheckRelinearizationRecursiveDouble(FastSet<Index>& relinKeys, double threshold, const VectorValues& delta, const ISAM2Clique::shared_ptr& clique) {

  // Check the current clique for relinearization
  bool relinearize = false;
  BOOST_FOREACH(Index var, clique->conditional()->keys()) {
    double maxDelta = delta[var].lpNorm<Eigen::Infinity>();
    if(maxDelta >= threshold) {
      relinKeys.insert(var);
      relinearize = true;
    }
  }

  // If this node was relinearized, also check its children
  if(relinearize) {
    BOOST_FOREACH(const ISAM2Clique::shared_ptr& child, clique->children()) {
      CheckRelinearizationRecursiveDouble(relinKeys, threshold, delta, child);
    }
  }
}

/* ************************************************************************* */
void CheckRelinearizationRecursiveMap(FastSet<Index>& relinKeys, const FastMap<char,Vector>& thresholds, const VectorValues& delta, const Ordering::InvertedMap& decoder, const ISAM2Clique::shared_ptr& clique) {

  // Check the current clique for relinearization
  bool relinearize = false;
  BOOST_FOREACH(Index var, clique->conditional()->keys()) {

    // Lookup the key associated with this index
    Key key = decoder.at(var);

    // Find the threshold for this variable type
    const Vector& threshold = thresholds.find(Symbol(key).chr())->second;

    // Verify the threshold vector matches the actual variable size
    if(threshold.rows() != delta[var].rows())
      throw std::invalid_argument("Relinearization threshold vector dimensionality passed into iSAM2 parameters does not match actual variable dimensionality");

    // Check for relinearization
    if((delta[var].array().abs() > threshold.array()).any()) {
      relinKeys.insert(var);
      relinearize = true;
    }
  }

  // If this node was relinearized, also check its children
  if(relinearize) {
    BOOST_FOREACH(const ISAM2Clique::shared_ptr& child, clique->children()) {
      CheckRelinearizationRecursiveMap(relinKeys, thresholds, delta, decoder, child);
    }
  }
}

/* ************************************************************************* */
FastSet<Index> ISAM2::Impl::CheckRelinearizationPartial(const ISAM2Clique::shared_ptr& root, const VectorValues& delta, const Ordering& ordering,
    const ISAM2Params::RelinearizationThreshold& relinearizeThreshold, const KeyFormatter& keyFormatter) {

  FastSet<Index> relinKeys;

  if(root) {
    if(relinearizeThreshold.type() == typeid(double)) {
      CheckRelinearizationRecursiveDouble(relinKeys, boost::get<double>(relinearizeThreshold), delta, root);
    } else if(relinearizeThreshold.type() == typeid(FastMap<char,Vector>)) {
      Ordering::InvertedMap decoder = ordering.invert();
      CheckRelinearizationRecursiveMap(relinKeys, boost::get<FastMap<char,Vector> >(relinearizeThreshold), delta, decoder, root);
    }
  }

  return relinKeys;
}

/* ************************************************************************* */
void ISAM2::Impl::FindAll(ISAM2Clique::shared_ptr clique, FastSet<Index>& keys, const vector<bool>& markedMask) {
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
  BOOST_FOREACH(const ISAM2Clique::shared_ptr& child, clique->children_) {
    FindAll(child, keys, markedMask);
  }
}

/* ************************************************************************* */
void ISAM2::Impl::ExpmapMasked(Values& values, const VectorValues& delta, const Ordering& ordering,
    const vector<bool>& mask, boost::optional<VectorValues&> invalidateIfDebug, const KeyFormatter& keyFormatter) {
  // If debugging, invalidate if requested, otherwise do not invalidate.
  // Invalidating means setting expmapped entries to Inf, to trigger assertions
  // if we try to re-use them.
#ifdef NDEBUG
  invalidateIfDebug = boost::none;
#endif

  assert(values.size() == ordering.nVars());
  assert(delta.size() == ordering.nVars());
  Values::iterator key_value;
  Ordering::const_iterator key_index;
  for(key_value = values.begin(), key_index = ordering.begin();
      key_value != values.end() && key_index != ordering.end(); ++key_value, ++key_index) {
    assert(key_value->key == key_index->first);
    const Index var = key_index->second;
    if(ISDEBUG("ISAM2 update verbose")) {
      if(mask[var])
        cout << "expmap " << keyFormatter(key_value->key) << " (j = " << var << "), delta = " << delta[var].transpose() << endl;
      else
        cout << "       " << keyFormatter(key_value->key) << " (j = " << var << "), delta = " << delta[var].transpose() << endl;
    }
    assert(delta[var].size() == (int)key_value->value.dim());
    assert(delta[var].unaryExpr(ptr_fun(isfinite<double>)).all());
    if(mask[var]) {
      Value* retracted = key_value->value.retract_(delta[var]);
      key_value->value = *retracted;
      retracted->deallocate_();
      if(invalidateIfDebug)
        (*invalidateIfDebug)[var].operator=(Vector::Constant(delta[var].rows(), numeric_limits<double>::infinity())); // Strange syntax to work with clang++ (bug in clang?)
    }
  }
}

/* ************************************************************************* */
ISAM2::Impl::PartialSolveResult
ISAM2::Impl::PartialSolve(GaussianFactorGraph& factors,
    const FastSet<Index>& keys, const ReorderingMode& reorderingMode, bool useQR) {

  const bool debug = ISDEBUG("ISAM2 recalculate");

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
    if(!reorderingMode.constrainedKeys->empty()) {
      typedef std::pair<const Index,int> Index_Group;
      if(keys.size() > reorderingMode.constrainedKeys->size()) { // Only if some variables are unconstrained
        BOOST_FOREACH(const Index_Group& index_group, *reorderingMode.constrainedKeys) {
          cmember[affectedKeysSelectorInverse[index_group.first]] = index_group.second; }
      } else {
        int minGroup = *boost::range::min_element(boost::adaptors::values(*reorderingMode.constrainedKeys));
        BOOST_FOREACH(const Index_Group& index_group, *reorderingMode.constrainedKeys) {
          cmember[affectedKeysSelectorInverse[index_group.first]] = index_group.second - minGroup; }
      }
    }
  }
  Permutation::shared_ptr affectedColamd(inference::PermutationCOLAMD_(affectedFactorsIndex, cmember));
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
  affectedFactorsIndex.permuteInPlace(*affectedColamd);
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
  JunctionTree<GaussianFactorGraph, ISAM2::Clique> jt(factors, affectedFactorsIndex);
  if(!useQR)
    result.bayesTree = jt.eliminate(EliminatePreferCholesky);
  else
    result.bayesTree = jt.eliminate(EliminateQR);
  toc(7,"eliminate");

  tic(8,"permute eliminated");
  if(result.bayesTree) result.bayesTree->permuteWithInverse(affectedKeysSelector);
  if(debug && result.bayesTree) {
    cout << "Full var-ordered eliminated BT:\n";
    result.bayesTree->printTree("");
  }
  // Undo permutation on our subset of cached factors, we must later permute *all* of the cached factors
  factors.permuteWithInverse(*affectedColamd);
  factors.permuteWithInverse(affectedKeysSelector);
  toc(8,"permute eliminated");

  return result;
}

/* ************************************************************************* */
namespace internal {
inline static void optimizeInPlace(const boost::shared_ptr<ISAM2Clique>& clique, VectorValues& result) {
  // parents are assumed to already be solved and available in result
  clique->conditional()->solveInPlace(result);

  // starting from the root, call optimize on each conditional
  BOOST_FOREACH(const boost::shared_ptr<ISAM2Clique>& child, clique->children_)
    optimizeInPlace(child, result);
}
}

/* ************************************************************************* */
size_t ISAM2::Impl::UpdateDelta(const boost::shared_ptr<ISAM2Clique>& root, std::vector<bool>& replacedKeys, VectorValues& delta, double wildfireThreshold) {

  size_t lastBacksubVariableCount;

  if (wildfireThreshold <= 0.0) {
    // Threshold is zero or less, so do a full recalculation
    internal::optimizeInPlace(root, delta);
    lastBacksubVariableCount = delta.size();

  } else {
    // Optimize with wildfire
    lastBacksubVariableCount = optimizeWildfire(root, wildfireThreshold, replacedKeys, delta); // modifies delta_

#ifndef NDEBUG
    for(size_t j=0; j<delta.size(); ++j)
      assert(delta[j].unaryExpr(ptr_fun(isfinite<double>)).all());
#endif
  }

  // Clear replacedKeys
  replacedKeys.assign(replacedKeys.size(), false);

  return lastBacksubVariableCount;
}

/* ************************************************************************* */
namespace internal {
void updateDoglegDeltas(const boost::shared_ptr<ISAM2Clique>& clique, std::vector<bool>& replacedKeys,
    const VectorValues& grad, VectorValues& deltaNewton, VectorValues& RgProd, size_t& varsUpdated) {

  // Check if any frontal or separator keys were recalculated, if so, we need
  // update deltas and recurse to children, but if not, we do not need to
  // recurse further because of the running separator property.
  bool anyReplaced = false;
  BOOST_FOREACH(Index j, *clique->conditional()) {
    if(replacedKeys[j]) {
      anyReplaced = true;
      break;
    }
  }

  if(anyReplaced) {
    // Update the current variable
    // Get VectorValues slice corresponding to current variables
    Vector gR = internal::extractVectorValuesSlices(grad, (*clique)->beginFrontals(), (*clique)->endFrontals());
    Vector gS = internal::extractVectorValuesSlices(grad, (*clique)->beginParents(), (*clique)->endParents());

    // Compute R*g and S*g for this clique
    Vector RSgProd = (*clique)->get_R() * gR + (*clique)->get_S() * gS;

    // Write into RgProd vector
    internal::writeVectorValuesSlices(RSgProd, RgProd, (*clique)->beginFrontals(), (*clique)->endFrontals());

    // Now solve the part of the Newton's method point for this clique (back-substitution)
    //(*clique)->solveInPlace(deltaNewton);

    varsUpdated += (*clique)->nrFrontals();

    // Recurse to children
    BOOST_FOREACH(const ISAM2Clique::shared_ptr& child, clique->children()) {
      updateDoglegDeltas(child, replacedKeys, grad, deltaNewton, RgProd, varsUpdated); }
  }
}
}

/* ************************************************************************* */
size_t ISAM2::Impl::UpdateDoglegDeltas(const ISAM2& isam, double wildfireThreshold, std::vector<bool>& replacedKeys,
    VectorValues& deltaNewton, VectorValues& RgProd) {

  // Get gradient
  VectorValues grad = *allocateVectorValues(isam);
  gradientAtZero(isam, grad);

  // Update variables
  size_t varsUpdated = 0;
  internal::updateDoglegDeltas(isam.root(), replacedKeys, grad, deltaNewton, RgProd, varsUpdated);
  optimizeWildfire(isam.root(), wildfireThreshold, replacedKeys, deltaNewton);

#if 0
  VectorValues expected = *allocateVectorValues(isam);
  internal::optimizeInPlace<ISAM2>(isam.root(), expected);
  for(size_t j = 0; j<expected.size(); ++j)
    assert(equal_with_abs_tol(expected[j], deltaNewton[j], 1e-2));

  FactorGraph<JacobianFactor> Rd_jfg(isam);
  Errors Rg = Rd_jfg * grad;
  double RgMagExpected = dot(Rg, Rg);
  double RgMagActual = RgProd.container().vector().squaredNorm();
  cout << fabs(RgMagExpected - RgMagActual) << endl;
  assert(fabs(RgMagExpected - RgMagActual) < (1e-8 * RgMagActual + 1e-4));
#endif

  replacedKeys.assign(replacedKeys.size(), false);

  return varsUpdated;
}

}
