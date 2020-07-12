/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file DiscreteConditional.h
 *  @date Feb 14, 2011
 *  @author Duy-Nguyen Ta
 *  @author Frank Dellaert
 */

#pragma once

#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/Signature.h>
#include <gtsam/inference/Conditional.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <string>

namespace gtsam {

/**
 * Discrete Conditional Density
 * Derives from DecisionTreeFactor
 */
class GTSAM_EXPORT DiscreteConditional: public DecisionTreeFactor,
    public Conditional<DecisionTreeFactor, DiscreteConditional> {

public:
  // typedefs needed to play nice with gtsam
  typedef DiscreteConditional This; ///< Typedef to this class
  typedef boost::shared_ptr<This> shared_ptr; ///< shared_ptr to this class
  typedef DecisionTreeFactor BaseFactor; ///< Typedef to our factor base class
  typedef Conditional<BaseFactor, This> BaseConditional; ///< Typedef to our conditional base class

  /** A map from keys to values..
   * TODO: Again, do we need this??? */
  typedef Assignment<Key> Values;
  typedef boost::shared_ptr<Values> sharedValues;

  /// @name Standard Constructors
  /// @{

  /** default constructor needed for serialization */
  DiscreteConditional() {
  }

  /** constructor from factor */
  DiscreteConditional(size_t nFrontals, const DecisionTreeFactor& f);

  /** Construct from signature */
  DiscreteConditional(const Signature& signature);

  /** construct P(X|Y)=P(X,Y)/P(Y) from P(X,Y) and P(Y) */
  DiscreteConditional(const DecisionTreeFactor& joint,
      const DecisionTreeFactor& marginal);

  /** construct P(X|Y)=P(X,Y)/P(Y) from P(X,Y) and P(Y) */
  DiscreteConditional(const DecisionTreeFactor& joint,
      const DecisionTreeFactor& marginal, const Ordering& orderedKeys);

  /**
   * Combine several conditional into a single one.
   * The conditionals must be given in increasing order, meaning that the parents
   * of any conditional may not include a conditional coming before it.
   * @param firstConditional Iterator to the first conditional to combine, must dereference to a shared_ptr<DiscreteConditional>.
   * @param lastConditional Iterator to after the last conditional to combine, must dereference to a shared_ptr<DiscreteConditional>.
   * */
  template<typename ITERATOR>
  static shared_ptr Combine(ITERATOR firstConditional,
      ITERATOR lastConditional);

  /// @}
  /// @name Testable
  /// @{

  /// GTSAM-style print
  void print(const std::string& s = "Discrete Conditional: ",
      const KeyFormatter& formatter = DefaultKeyFormatter) const;

  /// GTSAM-style equals
  bool equals(const DiscreteFactor& other, double tol = 1e-9) const;

  /// @}
  /// @name Standard Interface
  /// @{

  /// print index signature only
  void printSignature(
      const std::string& s = "Discrete Conditional: ",
      const KeyFormatter& formatter = DefaultKeyFormatter) const {
    static_cast<const BaseConditional*>(this)->print(s, formatter);
  }

  /// Evaluate, just look up in AlgebraicDecisonTree
  virtual double operator()(const Values& values) const {
    return Potentials::operator()(values);
  }

  /** Convert to a factor */
  DecisionTreeFactor::shared_ptr toFactor() const {
    return DecisionTreeFactor::shared_ptr(new DecisionTreeFactor(*this));
  }

  /** Restrict to given parent values, returns AlgebraicDecisionDiagram */
  ADT choose(const Assignment<Key>& parentsValues) const;

  /**
   * solve a conditional
   * @param parentsValues Known values of the parents
   * @return MPE value of the child (1 frontal variable).
   */
  size_t solve(const Values& parentsValues) const;

  /**
   * sample
   * @param parentsValues Known values of the parents
   * @return sample from conditional
   */
  size_t sample(const Values& parentsValues) const;

  /// @}
  /// @name Advanced Interface
  /// @{

  /// solve a conditional, in place
  void solveInPlace(Values& parentsValues) const;

  /// sample in place, stores result in partial solution
  void sampleInPlace(Values& parentsValues) const;

  /// @}

};
// DiscreteConditional

// traits
template<> struct traits<DiscreteConditional> : public Testable<DiscreteConditional> {};

/* ************************************************************************* */
template<typename ITERATOR>
DiscreteConditional::shared_ptr DiscreteConditional::Combine(
    ITERATOR firstConditional, ITERATOR lastConditional) {
  // TODO:  check for being a clique

  // multiply all the potentials of the given conditionals
  size_t nrFrontals = 0;
  DecisionTreeFactor product;
  for (ITERATOR it = firstConditional; it != lastConditional;
      ++it, ++nrFrontals) {
    DiscreteConditional::shared_ptr c = *it;
    DecisionTreeFactor::shared_ptr factor = c->toFactor();
    product = (*factor) * product;
  }
  // and then create a new multi-frontal conditional
  return boost::make_shared<DiscreteConditional>(nrFrontals, product);
}

} // gtsam

