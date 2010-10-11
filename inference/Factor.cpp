/**
 * @file    Factor.cpp
 * @brief   
 * @author  Richard Roberts
 * @created Sep 1, 2010
 */

#include <gtsam/inference/Factor-inl.h>
#include <gtsam/inference/Conditional.h>

#include <iostream>
#include <boost/foreach.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>

using namespace std;
using namespace boost::lambda;

namespace gtsam {

/* ************************************************************************* */
Factor::Factor(const Factor& f) : keys_(f.keys_), permuted_(f.permuted_) {}

/* ************************************************************************* */
void Factor::print(const std::string& s) const {
  cout << s << " ";
  BOOST_FOREACH(Index key, keys_) cout << " " << key;
  cout << endl;
}

/* ************************************************************************* */
bool Factor::equals(const Factor& other, double tol) const {
  return keys_ == other.keys_;
}

/* ************************************************************************* */
Conditional::shared_ptr Factor::eliminateFirst() {
  assert(!keys_.empty());
  checkSorted();
  Index eliminated = keys_.front();
  keys_.erase(keys_.begin());
  return Conditional::shared_ptr(new Conditional(eliminated, keys_));
}

/* ************************************************************************* */
boost::shared_ptr<BayesNet<Conditional> > Factor::eliminate(Index nFrontals) {
  assert(keys_.size() >= nFrontals);
  checkSorted();
  BayesNet<Conditional>::shared_ptr fragment(new BayesNet<Conditional>());
  const_iterator nextFrontal = this->begin();
  for(Index n = 0; n < nFrontals; ++n, ++nextFrontal)
    fragment->push_back(Conditional::shared_ptr(new Conditional(nextFrontal, const_iterator(this->end()), 1)));
  if(nFrontals > 0)
    keys_.assign(fragment->back()->beginParents(), fragment->back()->endParents());
  return fragment;
}

/* ************************************************************************* */
void Factor::permuteWithInverse(const Permutation& inversePermutation) {
  this->permuted_.value = true;
  BOOST_FOREACH(Index& key, keys_) { key = inversePermutation[key]; }
}

}
