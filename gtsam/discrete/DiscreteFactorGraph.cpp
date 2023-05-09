/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file DiscreteFactorGraph.cpp
 *  @date Feb 14, 2011
 *  @author Duy-Nguyen Ta
 *  @author Frank Dellaert
 */

//#define ENABLE_TIMING
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/discrete/DiscreteBayesTree.h>
#include <gtsam/discrete/DiscreteEliminationTree.h>
#include <gtsam/discrete/DiscreteJunctionTree.h>
#include <gtsam/inference/FactorGraph-inst.h>
#include <gtsam/inference/EliminateableFactorGraph-inst.h>
#include <boost/make_shared.hpp>

namespace gtsam {

  // Instantiate base classes
  template class FactorGraph<DiscreteFactor>;
  template class EliminateableFactorGraph<DiscreteFactorGraph>;

  /* ************************************************************************* */
  bool DiscreteFactorGraph::equals(const This& fg, double tol) const
  {
    return Base::equals(fg, tol);
  }

  /* ************************************************************************* */
  KeySet DiscreteFactorGraph::keys() const {
    KeySet keys;
    for(const sharedFactor& factor: *this)
    if (factor) keys.insert(factor->begin(), factor->end());
    return keys;
  }

  /* ************************************************************************* */
  DecisionTreeFactor DiscreteFactorGraph::product() const {
    DecisionTreeFactor result;
    for(const sharedFactor& factor: *this)
      if (factor) result = (*factor) * result;
    return result;
  }

  /* ************************************************************************* */
  double DiscreteFactorGraph::operator()(
      const DiscreteFactor::Values &values) const {
    double product = 1.0;
    for( const sharedFactor& factor: factors_ )
      product *= (*factor)(values);
    return product;
  }

  /* ************************************************************************* */
  void DiscreteFactorGraph::print(const std::string& s,
      const KeyFormatter& formatter) const {
    std::cout << s << std::endl;
    std::cout << "size: " << size() << std::endl;
    for (size_t i = 0; i < factors_.size(); i++) {
      std::stringstream ss;
      ss << "factor " << i << ": ";
      if (factors_[i] != nullptr) factors_[i]->print(ss.str(), formatter);
    }
  }

//  /* ************************************************************************* */
//  void DiscreteFactorGraph::permuteWithInverse(
//    const Permutation& inversePermutation) {
//      for(const sharedFactor& factor: factors_) {
//        if(factor)
//          factor->permuteWithInverse(inversePermutation);
//      }
//  }
//
//  /* ************************************************************************* */
//  void DiscreteFactorGraph::reduceWithInverse(
//    const internal::Reduction& inverseReduction) {
//      for(const sharedFactor& factor: factors_) {
//        if(factor)
//          factor->reduceWithInverse(inverseReduction);
//      }
//  }

  /* ************************************************************************* */
  DiscreteFactor::sharedValues DiscreteFactorGraph::optimize() const
  {
    gttic(DiscreteFactorGraph_optimize);
    return BaseEliminateable::eliminateSequential()->optimize();
  }

  /* ************************************************************************* */
  std::pair<DiscreteConditional::shared_ptr, DecisionTreeFactor::shared_ptr>  //
  EliminateDiscrete(const DiscreteFactorGraph& factors, const Ordering& frontalKeys) {

    // PRODUCT: multiply all factors
    gttic(product);
    DecisionTreeFactor product;
    for(const DiscreteFactor::shared_ptr& factor: factors)
      product = (*factor) * product;
    gttoc(product);

    // sum out frontals, this is the factor on the separator
    gttic(sum);
    DecisionTreeFactor::shared_ptr sum = product.sum(frontalKeys);
    gttoc(sum);

    // Ordering keys for the conditional so that frontalKeys are really in front
    Ordering orderedKeys;
    orderedKeys.insert(orderedKeys.end(), frontalKeys.begin(), frontalKeys.end());
    orderedKeys.insert(orderedKeys.end(), sum->keys().begin(), sum->keys().end());

    // now divide product/sum to get conditional
    gttic(divide);
    DiscreteConditional::shared_ptr cond(new DiscreteConditional(product, *sum, orderedKeys));
    gttoc(divide);

    return std::make_pair(cond, sum);
  }

/* ************************************************************************* */
} // namespace

