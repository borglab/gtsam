/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Potentials.cpp
 * @date March 24, 2011
 * @author Frank Dellaert
 */

#include <gtsam/discrete/DecisionTree-inl.h>
#include <gtsam/discrete/Potentials.h>

#include <boost/format.hpp>

#include <string>

using namespace std;

namespace gtsam {

// explicit instantiation
template class DecisionTree<Key, double>;
template class AlgebraicDecisionTree<Key>;

/* ************************************************************************* */
double Potentials::safe_div(const double& a, const double& b) {
  // cout << boost::format("%g / %g = %g\n") % a % b % ((a == 0) ? 0 : (a / b));
  // The use for safe_div is when we divide the product factor by the sum
  // factor. If the product or sum is zero, we accord zero probability to the
  // event.
  return (a == 0 || b == 0) ? 0 : (a / b);
}

/* ********************************************************************************
 */
Potentials::Potentials() : ADT(1.0) {}

/* ********************************************************************************
 */
Potentials::Potentials(const DiscreteKeys& keys, const ADT& decisionTree)
    : ADT(decisionTree), cardinalities_(keys.cardinalities()) {}

/* ************************************************************************* */
bool Potentials::equals(const Potentials& other, double tol) const {
  return ADT::equals(other, tol);
}

/* ************************************************************************* */
void Potentials::print(const string& s, const KeyFormatter& formatter) const {
  cout << s << "\n  Cardinalities: {";
  for (const std::pair<const Key,size_t>& key : cardinalities_)
    cout << formatter(key.first) << ":" << key.second << ", ";
  cout << "}" << endl;
  ADT::print(" ");
}
//
//  /* ************************************************************************* */
//  template<class P>
//  void Potentials::remapIndices(const P& remapping) {
//    // Permute the _cardinalities (TODO: Inefficient Consider Improving)
//    DiscreteKeys keys;
//    map<Key, Key> ordering;
//
//    // Get the original keys from cardinalities_
//    for(const DiscreteKey& key: cardinalities_)
//      keys & key;
//
//    // Perform Permutation
//    for(DiscreteKey& key: keys) {
//      ordering[key.first] = remapping[key.first];
//      key.first = ordering[key.first];
//    }
//
//    // Change *this
//    AlgebraicDecisionTree<Key> permuted((*this), ordering);
//    *this = permuted;
//    cardinalities_ = keys.cardinalities();
//  }
//
//  /* ************************************************************************* */
//  void Potentials::permuteWithInverse(const Permutation& inversePermutation) {
//    remapIndices(inversePermutation);
//  }
//
//  /* ************************************************************************* */
//  void Potentials::reduceWithInverse(const internal::Reduction& inverseReduction) {
//    remapIndices(inverseReduction);
//  }

  /* ************************************************************************* */

}  // namespace gtsam
