/**
 * @file    Factor.h
 * @brief   A simple factor class to use in a factor graph
 * @brief   factor
 * @author  Kai Ni
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <vector>
#include <map>
#include <boost/utility.hpp> // for noncopyable
#include <boost/serialization/nvp.hpp>
#include <gtsam/base/types.h>
#include <gtsam/base/Testable.h>
#include <gtsam/inference/inference.h>

namespace gtsam {

class Conditional;

/**
 * A simple factor class to use in a factor graph.
 *
 * We make it noncopyable so we enforce the fact that factors are
 * kept in pointer containers. To be safe, you should make them
 * immutable, i.e., practicing functional programming. However, this
 * conflicts with efficiency as well, esp. in the case of incomplete
 * QR factorization. A solution is still being sought.
 *
 * A Factor is templated on a Values, for example VectorValues is a values structure of
 * labeled vectors. This way, we can have factors that might be defined on discrete
 * variables, continuous ones, or a combination of both. It is up to the config to
 * provide the appropriate values at the appropriate time.
 */
class Factor : public Testable<Factor> {
protected:

  std::vector<Index> keys_;

  /** Internal check to make sure keys are sorted.
   * If NDEBUG is defined, this is empty and optimized out. */
  void assertInvariants() const;

public:

  typedef gtsam::Conditional Conditional;
  typedef boost::shared_ptr<Factor> shared_ptr;
  typedef std::vector<Index>::iterator iterator;
  typedef std::vector<Index>::const_iterator const_iterator;

  /** Copy constructor */
  Factor(const Factor& f);

  /** Construct from derived type */
  Factor(const Conditional& c);

  /** Constructor from a collection of keys */
  template<class KeyIterator> Factor(KeyIterator beginKey, KeyIterator endKey);

  /** Default constructor for I/O */
  Factor() {}

  /** Construct unary factor */
  Factor(Index key) : keys_(1) {
    keys_[0] = key; assertInvariants(); }

  /** Construct binary factor */
  Factor(Index key1, Index key2) : keys_(2) {
    keys_[0] = key1; keys_[1] = key2; assertInvariants(); }

  /** Construct ternary factor */
  Factor(Index key1, Index key2, Index key3) : keys_(3) {
    keys_[0] = key1; keys_[1] = key2; keys_[2] = key3; assertInvariants(); }

  /** Construct 4-way factor */
  Factor(Index key1, Index key2, Index key3, Index key4) : keys_(4) {
    keys_[0] = key1; keys_[1] = key2; keys_[2] = key3; keys_[3] = key4; assertInvariants(); }

  /** Named constructor for combining a set of factors with pre-computed set of
   * variables.  (Old style - will be removed when scalar elimination is
   * removed in favor of the EliminationTree). */
  template<class FactorGraphType, class VariableIndexStorage>
  static shared_ptr Combine(const FactorGraphType& factorGraph,
      const VariableIndex<VariableIndexStorage>& variableIndex, const std::vector<size_t>& factors,
      const std::vector<Index>& variables, const std::vector<std::vector<size_t> >& variablePositions);

  /** Create a combined joint factor (new style for EliminationTree). */
  template<class MapAllocator>
  static shared_ptr Combine(const FactorGraph<Factor>& factors, const std::map<Index, std::vector<Index>, std::less<Index>, MapAllocator>& variableSlots);

  /**
   * eliminate the first variable involved in this factor
   * @return a conditional on the eliminated variable
   */
  boost::shared_ptr<Conditional> eliminateFirst();

  /**
   * eliminate the first nrFrontals frontal variables.
   */
  boost::shared_ptr<BayesNet<Conditional> > eliminate(size_t nrFrontals = 1);

  /**
   * Permutes the GaussianFactor, but for efficiency requires the permutation
   * to already be inverted.
   */
  void permuteWithInverse(const Permutation& inversePermutation);

  /** iterators */
  const_iterator begin() const { return keys_.begin(); }
  const_iterator end() const { return keys_.end(); }

  /** mutable iterators */
  iterator begin() { return keys_.begin(); }
  iterator end() { return keys_.end(); }

  /** First key*/
  Index front() const { return keys_.front(); }

  /** Last key */
  Index back() const { return keys_.back(); }

  /** find */
  const_iterator find(Index key) const { return std::find(begin(), end(), key); }

  /** print */
  void print(const std::string& s = "Factor") const;

  /** check equality */
  bool equals(const Factor& other, double tol = 1e-9) const;

  /**
   * return keys in order as created
   */
  const std::vector<Index>& keys() const { return keys_; }

  /**
   * @return the number of nodes the factor connects
   */
  size_t size() const { return keys_.size(); }

protected:

  /** Conditional makes internal use of a Factor for storage */
  friend class gtsam::Conditional;
  friend class GaussianConditional;

  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_NVP(keys_);
  }
};

/* ************************************************************************* */
inline void Factor::assertInvariants() const {
#ifndef NDEBUG
  std::set<Index> uniqueSorted(keys_.begin(), keys_.end());
  assert(uniqueSorted.size() == keys_.size());
  assert(std::equal(uniqueSorted.begin(), uniqueSorted.end(), keys_.begin()));
#endif
}

}
