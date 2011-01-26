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

#include <gtsam/base/FastSet.h>
#include <gtsam/inference/FactorBase-inl.h>
#include <gtsam/inference/IndexFactor.h>
#include <gtsam/inference/IndexConditional.h>
#include <gtsam/inference/VariableSlots.h>

using namespace std;

namespace gtsam {

template class FactorBase<Index>;

IndexFactor::IndexFactor(const IndexConditional& c) : Base(static_cast<const Base>(c)) {}

pair<BayesNet<IndexConditional>::shared_ptr, IndexFactor::shared_ptr> IndexFactor::CombineAndEliminate(
    const FactorGraph<This>& factors, size_t nrFrontals) {

  FastSet<Index> variables;
  BOOST_FOREACH(const shared_ptr& factor, factors) {
    BOOST_FOREACH(Index var, *factor) {
      variables.insert(var); } }

  if(variables.size() < 1)
    throw invalid_argument("IndexFactor::CombineAndEliminate called on factors with zero total variables.");

  pair<BayesNet<Conditional>::shared_ptr, shared_ptr> result;
  result.first.reset(new BayesNet<IndexConditional>());
  FastSet<Index>::const_iterator var;
  for(var = variables.begin(); result.first->size() < nrFrontals; ++var)
    result.first->push_back(IndexConditional::FromRange(var, variables.end(), 1));
  result.second.reset(new IndexFactor(var, variables.end()));

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
