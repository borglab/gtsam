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

#include <gtsam/discrete/DiscreteBayesTree.h>
#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/discrete/DiscreteEliminationTree.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/discrete/DiscreteJunctionTree.h>
#include <gtsam/inference/EliminateableFactorGraph-inst.h>
#include <gtsam/inference/FactorGraph-inst.h>

using std::vector;
using std::string;
using std::map;

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
    for (const sharedFactor& factor : *this) {
      if (factor) keys.insert(factor->begin(), factor->end());
    }
    return keys;
  }

  /* ************************************************************************* */
  DiscreteKeys DiscreteFactorGraph::discreteKeys() const {
    DiscreteKeys result;
    for (auto&& factor : *this) {
      if (auto p = boost::dynamic_pointer_cast<DecisionTreeFactor>(factor)) {
        DiscreteKeys factor_keys = p->discreteKeys();
        result.insert(result.end(), factor_keys.begin(), factor_keys.end());
      }
    }

    return result;
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
      const DiscreteValues &values) const {
    double product = 1.0;
    for( const sharedFactor& factor: factors_ )
      product *= (*factor)(values);
    return product;
  }

  /* ************************************************************************* */
  void DiscreteFactorGraph::print(const string& s,
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
  DiscreteValues DiscreteFactorGraph::optimize() const
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
  string DiscreteFactorGraph::markdown(
      const KeyFormatter& keyFormatter,
      const DiscreteFactor::Names& names) const {
    using std::endl;
    std::stringstream ss;
    ss << "`DiscreteFactorGraph` of size " << size() << endl << endl;
    for (size_t i = 0; i < factors_.size(); i++) {
      ss << "factor " << i << ":\n";
      ss << factors_[i]->markdown(keyFormatter, names) << endl;
    }
    return ss.str();
  }

  /* ************************************************************************* */
  }  // namespace gtsam
