/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    IndexFactor.cpp
 * @brief   
 * @author  Richard Roberts
 * @created Oct 17, 2010
 */

#include <gtsam/inference/FactorBase-inl.h>
#include <gtsam/inference/IndexFactor.h>
#include <gtsam/inference/VariableSlots.h>

using namespace std;

namespace gtsam {

template class FactorBase<Index>;

IndexFactor::IndexFactor(const IndexConditional& c) : Base(static_cast<const Base>(c)) {}

pair<BayesNet<IndexConditional>::shared_ptr, IndexFactor::shared_ptr> IndexFactor::CombineAndEliminate(
    const FactorGraph<This>& factors, size_t nrFrontals) {
  pair<BayesNet<Conditional>::shared_ptr, shared_ptr> result;
  result.second = Combine(factors, VariableSlots(factors));
  result.first = result.second->eliminate(nrFrontals);
  return result;
}

IndexFactor::shared_ptr IndexFactor::Combine(
    const FactorGraph<This>& factors, const FastMap<Index, std::vector<Index> >& variableSlots) {
  return Base::Combine<This>(factors, variableSlots); }

boost::shared_ptr<IndexConditional> IndexFactor::eliminateFirst() {
  return Base::eliminateFirst<IndexConditional>(); }

boost::shared_ptr<BayesNet<IndexConditional> > IndexFactor::eliminate(size_t nrFrontals) {
  return Base::eliminate<IndexConditional>(nrFrontals); }

}
