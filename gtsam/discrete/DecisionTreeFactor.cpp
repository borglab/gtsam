/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DecisionTreeFactor.cpp
 * @brief discrete factor
 * @date Feb 14, 2011
 * @author Duy-Nguyen Ta
 * @author Frank Dellaert
 */

#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/base/FastSet.h>

#include <boost/make_shared.hpp>

using namespace std;

namespace gtsam {

  /* ******************************************************************************** */
  DecisionTreeFactor::DecisionTreeFactor() {
  }

  /* ******************************************************************************** */
  DecisionTreeFactor::DecisionTreeFactor(const DiscreteKeys& keys,
      const ADT& potentials) :
      DiscreteFactor(keys.indices()), Potentials(keys, potentials) {
  }

  /* *************************************************************************/
  DecisionTreeFactor::DecisionTreeFactor(const DiscreteConditional& c) :
      DiscreteFactor(c.keys()), Potentials(c) {
  }

  /* ************************************************************************* */
  bool DecisionTreeFactor::equals(const DiscreteFactor& other, double tol) const {
    if(!dynamic_cast<const DecisionTreeFactor*>(&other)) {
      return false;
    }
    else {
      const DecisionTreeFactor& f(static_cast<const DecisionTreeFactor&>(other));
      return Potentials::equals(f, tol);
    }
  }

  /* ************************************************************************* */
  void DecisionTreeFactor::print(const string& s,
      const KeyFormatter& formatter) const {
    cout << s;
    Potentials::print("Potentials:",formatter);
  }

  /* ************************************************************************* */
  DecisionTreeFactor DecisionTreeFactor::apply(const DecisionTreeFactor& f,
    ADT::Binary op) const {
    map<Key,size_t> cs; // new cardinalities
    // make unique key-cardinality map
    for(Key j: keys()) cs[j] = cardinality(j);
    for(Key j: f.keys()) cs[j] = f.cardinality(j);
    // Convert map into keys
    DiscreteKeys keys;
    for(const std::pair<const Key,size_t>& key: cs)
      keys.push_back(key);
    // apply operand
    ADT result = ADT::apply(f, op);
    // Make a new factor
    return DecisionTreeFactor(keys, result);
  }

  /* ************************************************************************* */
  DecisionTreeFactor::shared_ptr DecisionTreeFactor::combine(size_t nrFrontals,
    ADT::Binary op) const {

    if (nrFrontals > size()) throw invalid_argument(
        (boost::format(
            "DecisionTreeFactor::combine: invalid number of frontal keys %d, nr.keys=%d")
            % nrFrontals % size()).str());

    // sum over nrFrontals keys
    size_t i;
    ADT result(*this);
    for (i = 0; i < nrFrontals; i++) {
      Key j = keys()[i];
      result = result.combine(j, cardinality(j), op);
    }

    // create new factor, note we start keys after nrFrontals
    DiscreteKeys dkeys;
    for (; i < keys().size(); i++) {
      Key j = keys()[i];
      dkeys.push_back(DiscreteKey(j,cardinality(j)));
    }
    return boost::make_shared<DecisionTreeFactor>(dkeys, result);
  }


  /* ************************************************************************* */
  DecisionTreeFactor::shared_ptr DecisionTreeFactor::combine(const Ordering& frontalKeys,
    ADT::Binary op) const {

    if (frontalKeys.size() > size()) throw invalid_argument(
        (boost::format(
            "DecisionTreeFactor::combine: invalid number of frontal keys %d, nr.keys=%d")
            % frontalKeys.size() % size()).str());

    // sum over nrFrontals keys
    size_t i;
    ADT result(*this);
    for (i = 0; i < frontalKeys.size(); i++) {
      Key j = frontalKeys[i];
      result = result.combine(j, cardinality(j), op);
    }

    // create new factor, note we collect keys that are not in frontalKeys
    // TODO: why do we need this??? result should contain correct keys!!!
    DiscreteKeys dkeys;
    for (i = 0; i < keys().size(); i++) {
      Key j = keys()[i];
      // TODO: inefficient!
      if (std::find(frontalKeys.begin(), frontalKeys.end(), j) != frontalKeys.end())
        continue;
      dkeys.push_back(DiscreteKey(j,cardinality(j)));
    }
    return boost::make_shared<DecisionTreeFactor>(dkeys, result);
  }

/* ************************************************************************* */
} // namespace gtsam
