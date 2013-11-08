/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file DiscreteConditional.cpp
 *  @date Feb 14, 2011
 *  @author Duy-Nguyen Ta
 *  @author Frank Dellaert
 */

#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/discrete/Signature.h>
#include <gtsam/inference/Conditional-inst.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/debug.h>

#include <boost/make_shared.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

#include <vector>
#include <algorithm>
#include <stdexcept>

using namespace std;

namespace gtsam {

// Instantiate base class
template class Conditional<DecisionTreeFactor, DiscreteConditional> ;

/* ******************************************************************************** */
DiscreteConditional::DiscreteConditional(const size_t nrFrontals,
    const DecisionTreeFactor& f) :
    BaseFactor(f / (*f.sum(nrFrontals))), BaseConditional(nrFrontals) {
}

/* ******************************************************************************** */
DiscreteConditional::DiscreteConditional(const DecisionTreeFactor& joint,
    const DecisionTreeFactor& marginal, const boost::optional<Ordering>& orderedKeys) :
    BaseFactor(
        ISDEBUG("DiscreteConditional::COUNT") ? joint : joint / marginal), BaseConditional(
            joint.size()-marginal.size()) {
  if (ISDEBUG("DiscreteConditional::DiscreteConditional"))
    cout << (firstFrontalKey()) << endl; //TODO Print all keys
  if (orderedKeys) {
    keys_.clear();
    keys_.insert(keys_.end(), orderedKeys->begin(), orderedKeys->end());
  }
}

/* ******************************************************************************** */
DiscreteConditional::DiscreteConditional(const Signature& signature) :
        BaseFactor(signature.discreteKeysParentsFirst(), signature.cpt()), BaseConditional(
            1) {
}

/* ******************************************************************************** */
void DiscreteConditional::print(const std::string& s,
    const KeyFormatter& formatter) const {
  std::cout << s << std::endl;
  Potentials::print(s);
}

/* ******************************************************************************** */
bool DiscreteConditional::equals(const DiscreteFactor& other,
    double tol) const {
  if (!dynamic_cast<const DecisionTreeFactor*>(&other))
    return false;
  else {
    const DecisionTreeFactor& f(
        static_cast<const DecisionTreeFactor&>(other));
    return DecisionTreeFactor::equals(f, tol);
  }
}

/* ******************************************************************************** */
Potentials::ADT DiscreteConditional::choose(const Values& parentsValues) const {
  ADT pFS(*this);
  Key j; size_t value;
  BOOST_FOREACH(Key key, parents())
  try {
    j = (key);
    value = parentsValues.at(j);
    pFS = pFS.choose(j, value);
  } catch (exception&) {
    cout << "Key: " << j << "  Value: " << value << endl;
    parentsValues.print("parentsValues: ");
//    pFS.print("pFS: ");
    throw runtime_error("DiscreteConditional::choose: parent value missing");
  };
  return pFS;
}

/* ******************************************************************************** */
void DiscreteConditional::solveInPlace(Values& values) const {
  // TODO: Abhijit asks: is this really the fastest way? He thinks it is.
  ADT pFS = choose(values); // P(F|S=parentsValues)

  // Initialize
  Values mpe;
  double maxP = 0;

  DiscreteKeys keys;
  BOOST_FOREACH(Key idx, frontals()) {
    DiscreteKey dk(idx, cardinality(idx));
    keys & dk;
  }
  // Get all Possible Configurations
  vector<Values> allPosbValues = cartesianProduct(keys);

  // Find the MPE
  BOOST_FOREACH(Values& frontalVals, allPosbValues) {
    double pValueS = pFS(frontalVals); // P(F=value|S=parentsValues)
    // Update MPE solution if better
    if (pValueS > maxP) {
      maxP = pValueS;
      mpe = frontalVals;
    }
  }

  //set values (inPlace) to mpe
  BOOST_FOREACH(Key j, frontals()) {
    values[j] = mpe[j];
  }
}

/* ******************************************************************************** */
void DiscreteConditional::sampleInPlace(Values& values) const {
  assert(nrFrontals() == 1);
  Key j = (firstFrontalKey());
  size_t sampled = sample(values); // Sample variable
  values[j] = sampled; // store result in partial solution
}

/* ******************************************************************************** */
size_t DiscreteConditional::solve(const Values& parentsValues) const {

  // TODO: is this really the fastest way? I think it is.
  ADT pFS = choose(parentsValues); // P(F|S=parentsValues)

  // Then, find the max over all remaining
  // TODO, only works for one key now, seems horribly slow this way
  size_t mpe = 0;
  Values frontals;
  double maxP = 0;
  assert(nrFrontals() == 1);
  Key j = (firstFrontalKey());
  for (size_t value = 0; value < cardinality(j); value++) {
    frontals[j] = value;
    double pValueS = pFS(frontals); // P(F=value|S=parentsValues)
    // Update MPE solution if better
    if (pValueS > maxP) {
      maxP = pValueS;
      mpe = value;
    }
  }
  return mpe;
}

/* ******************************************************************************** */
size_t DiscreteConditional::sample(const Values& parentsValues) const {

  using boost::uniform_real;
  static boost::mt19937 gen(2); // random number generator

  bool debug = ISDEBUG("DiscreteConditional::sample");

  // Get the correct conditional density
  ADT pFS = choose(parentsValues); // P(F|S=parentsValues)
  if (debug)
    GTSAM_PRINT(pFS);

  // get cumulative distribution function (cdf)
  // TODO, only works for one key now, seems horribly slow this way
  assert(nrFrontals() == 1);
  Key j = (firstFrontalKey());
  size_t nj = cardinality(j);
  vector<double> cdf(nj);
  Values frontals;
  double sum = 0;
  for (size_t value = 0; value < nj; value++) {
    frontals[j] = value;
    double pValueS = pFS(frontals); // P(F=value|S=parentsValues)
    sum += pValueS; // accumulate
    if (debug)
      cout << sum << " ";
    if (pValueS == 1) {
      if (debug)
        cout << "--> " << value << endl;
      return value; // shortcut exit
    }
    cdf[value] = sum;
  }

  // inspired by http://www.boost.org/doc/libs/1_46_1/doc/html/boost_random/tutorial.html
  uniform_real<> dist(0, cdf.back());
  boost::variate_generator<boost::mt19937&, uniform_real<> > die(gen, dist);
  size_t sampled = lower_bound(cdf.begin(), cdf.end(), die()) - cdf.begin();
  if (debug)
    cout << "-> " << sampled << endl;

  return sampled;

  return 0;
}

/* ******************************************************************************** */
//void DiscreteConditional::permuteWithInverse(
//    const Permutation& inversePermutation) {
//  IndexConditionalOrdered::permuteWithInverse(inversePermutation);
//  Potentials::permuteWithInverse(inversePermutation);
//}
/* ******************************************************************************** */

}// namespace
