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

#include <gtsam/base/timing.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/inference/FactorGraph-inst.h>
#include <gtsam/inference/Ordering.h>
#include <boost/make_shared.hpp>

namespace gtsam {

  /* ************************************************************************* */
  FastSet<Index> DiscreteFactorGraph::keys() const {
    FastSet<Index> keys;
    BOOST_FOREACH(const sharedFactor& factor, *this)
    if (factor) keys.insert(factor->begin(), factor->end());
    return keys;
  }

  /* ************************************************************************* */
  DecisionTreeFactor DiscreteFactorGraph::product() const {
    DecisionTreeFactor result;
    BOOST_FOREACH(const sharedFactor& factor, *this)
      if (factor) result = (*factor) * result;
    return result;
  }

  /* ************************************************************************* */
  double DiscreteFactorGraph::operator()(
      const DiscreteFactor::Values &values) const {
    double product = 1.0;
    BOOST_FOREACH( const sharedFactor& factor, factors_ )
      product *= (*factor)(values);
    return product;
  }

  /* ************************************************************************* */
  void DiscreteFactorGraph::print(const std::string& s,
      const IndexFormatter& formatter) const {
    std::cout << s << std::endl;
    std::cout << "size: " << size() << std::endl;
    for (size_t i = 0; i < factors_.size(); i++) {
      std::stringstream ss;
      ss << "factor " << i << ": ";
      if (factors_[i] != NULL) factors_[i]->print(ss.str(), formatter);
    }
  }

  /* ************************************************************************* */
  std::pair<DiscreteConditional::shared_ptr, DecisionTreeFactor::shared_ptr>  //
  EliminateDiscrete(const DiscreteFactorGraph& factors, const Ordering& keys) {

    // PRODUCT: multiply all factors
    gttic(product);
    DecisionTreeFactor product;
    BOOST_FOREACH(const DiscreteFactor::shared_ptr& factor, factors)
      product = (*factor) * product;

    gttoc(product);

    // sum out frontals, this is the factor on the separator
    gttic(sum);
    DecisionTreeFactor::shared_ptr sum = product.sum(num);
    gttoc(sum);

    // now divide product/sum to get conditional
    gttic(divide);
    DiscreteConditional::shared_ptr cond(new DiscreteConditional(product, *sum));
    gttoc(divide);

    return std::make_pair(cond, sum);
  }


/* ************************************************************************* */
} // namespace

