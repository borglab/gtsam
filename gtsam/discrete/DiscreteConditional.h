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

#include <gtsam/inference/Conditional-inst.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/Signature.h>

#include <memory>
#include <string>
#include <vector>

namespace gtsam {

/**
 * Discrete Conditional Density
 * Derives from DecisionTreeFactor
 *
 * @ingroup discrete
 */
class GTSAM_EXPORT DiscreteConditional
    : public DecisionTreeFactor,
      public Conditional<DecisionTreeFactor, DiscreteConditional> {
 public:
  // typedefs needed to play nice with gtsam
  typedef DiscreteConditional This;            ///< Typedef to this class
  typedef std::shared_ptr<This> shared_ptr;  ///< shared_ptr to this class
  typedef DecisionTreeFactor BaseFactor;  ///< Typedef to our factor base class
  typedef Conditional<BaseFactor, This>
      BaseConditional;  ///< Typedef to our conditional base class

  using Values = DiscreteValues;  ///< backwards compatibility

  /// @name Standard Constructors
  /// @{

  /// Default constructor needed for serialization.
  DiscreteConditional() {}

  /// Construct from factor, taking the first `nFrontals` keys as frontals.
  DiscreteConditional(size_t nFrontals, const DecisionTreeFactor& f);

  /**
   * Construct from DiscreteKeys and AlgebraicDecisionTree, taking the first
   * `nFrontals` keys as frontals, in the order given.
   */
  DiscreteConditional(size_t nFrontals, const DiscreteKeys& keys,
                      const ADT& potentials);

  /** Construct from signature */
  explicit DiscreteConditional(const Signature& signature);

  /**
   * Construct from key, parents, and a Signature::Table specifying the
   * conditional probability table (CPT) in 00 01 10 11 order. For
   * three-valued, it would be 00 01 02 10 11 12 20 21 22, etc....
   *
   * Example: DiscreteConditional P(D, {B,E}, table);
   */
  DiscreteConditional(const DiscreteKey& key, const DiscreteKeys& parents,
                      const Signature::Table& table)
      : DiscreteConditional(Signature(key, parents, table)) {}

  /**
   * Construct from key, parents, and a vector<double> specifying the
   * conditional probability table (CPT) in 00 01 10 11 order. For
   * three-valued, it would be 00 01 02 10 11 12 20 21 22, etc....
   *
   * Example: DiscreteConditional P(D, {B,E}, table);
   */
  DiscreteConditional(const DiscreteKey& key, const DiscreteKeys& parents,
                      const std::vector<double>& table)
      : DiscreteConditional(1, DiscreteKeys{key} & parents,
                            ADT(DiscreteKeys{key} & parents, table)) {}

  /**
   * Construct from key, parents, and a string specifying the conditional
   * probability table (CPT) in 00 01 10 11 order. For three-valued, it would
   * be 00 01 02 10 11 12 20 21 22, etc....
   *
   * The string is parsed into a Signature::Table.
   *
   * Example: DiscreteConditional P(D, {B,E}, "9/1 2/8 3/7 1/9");
   */
  DiscreteConditional(const DiscreteKey& key, const DiscreteKeys& parents,
                      const std::string& spec)
      : DiscreteConditional(Signature(key, parents, spec)) {}

  /// No-parent specialization; can also use DiscreteDistribution.
  DiscreteConditional(const DiscreteKey& key, const std::string& spec)
      : DiscreteConditional(Signature(key, {}, spec)) {}

  /**
   * @brief construct P(X|Y) = f(X,Y)/f(Y) from f(X,Y) and f(Y)
   * Assumes but *does not check* that f(Y)=sum_X f(X,Y).
   */
  DiscreteConditional(const DecisionTreeFactor& joint,
                      const DecisionTreeFactor& marginal);

  /**
   * @brief construct P(X|Y) = f(X,Y)/f(Y) from f(X,Y) and f(Y)
   * Assumes but *does not check* that f(Y)=sum_X f(X,Y).
   * Makes sure the keys are ordered as given. Does not check orderedKeys.
   */
  DiscreteConditional(const DecisionTreeFactor& joint,
                      const DecisionTreeFactor& marginal,
                      const Ordering& orderedKeys);

  /**
   * @brief Combine two conditionals, yielding a new conditional with the union
   * of the frontal keys, ordered by gtsam::Key.
   *
   * The two conditionals must make a valid Bayes net fragment, i.e.,
   * the frontal variables cannot overlap, and must be acyclic:
   * Example of correct use:
   *   P(A,B) = P(A|B) * P(B)
   *   P(A,B|C) = P(A|B) * P(B|C)
   *   P(A,B,C) = P(A,B|C) * P(C)
   * Example of incorrect use:
   *   P(A|B) * P(A|C) = ?
   *   P(A|B) * P(B|A) = ?
   * We check for overlapping frontals, but do *not* check for cyclic.
   */
  DiscreteConditional operator*(const DiscreteConditional& other) const;

  /** Calculate marginal on given key, no parent case. */
  DiscreteConditional marginal(Key key) const;

  /// @}
  /// @name Testable
  /// @{

  /// GTSAM-style print
  void print(
      const std::string& s = "Discrete Conditional: ",
      const KeyFormatter& formatter = DefaultKeyFormatter) const override;

  /// GTSAM-style equals
  bool equals(const DiscreteFactor& other, double tol = 1e-9) const override;

  /// @}
  /// @name Standard Interface
  /// @{

  /// Log-probability is just -error(x).
  double logProbability(const DiscreteValues& x) const  {
    return -error(x);
  }

  /// print index signature only
  void printSignature(
      const std::string& s = "Discrete Conditional: ",
      const KeyFormatter& formatter = DefaultKeyFormatter) const {
    static_cast<const BaseConditional*>(this)->print(s, formatter);
  }

  /// Evaluate, just look up in AlgebraicDecisonTree
  double evaluate(const DiscreteValues& values) const {
    return ADT::operator()(values);
  }

  using DecisionTreeFactor::error;       ///< DiscreteValues version
  using DecisionTreeFactor::operator();  ///< DiscreteValues version

  /**
   * @brief restrict to given *parent* values.
   *
   * Note: does not need be complete set. Examples:
   *
   * P(C|D,E) + . -> P(C|D,E)
   * P(C|D,E) + E -> P(C|D)
   * P(C|D,E) + D -> P(C|E)
   * P(C|D,E) + D,E -> P(C)
   * P(C|D,E) + C -> error!
   *
   * @return a shared_ptr to a new DiscreteConditional
   */
  shared_ptr choose(const DiscreteValues& given) const;

  /** Convert to a likelihood factor by providing value before bar. */
  DecisionTreeFactor::shared_ptr likelihood(
      const DiscreteValues& frontalValues) const;

  /** Single variable version of likelihood. */
  DecisionTreeFactor::shared_ptr likelihood(size_t frontal) const;

  /**
   * sample
   * @param parentsValues Known values of the parents
   * @return sample from conditional
   */
  size_t sample(const DiscreteValues& parentsValues) const;

  /// Single parent version.
  size_t sample(size_t parent_value) const;

  /// Zero parent version.
  size_t sample() const;

  /**
   * @brief Return assignment that maximizes distribution.
   * @return Optimal assignment (1 frontal variable).
   */
  size_t argmax(const DiscreteValues& parentsValues = DiscreteValues()) const;

  /// @}
  /// @name Advanced Interface
  /// @{

  /// sample in place, stores result in partial solution
  void sampleInPlace(DiscreteValues* parentsValues) const;

  /// Return all assignments for frontal variables.
  std::vector<DiscreteValues> frontalAssignments() const;

  /// Return all assignments for frontal *and* parent variables.
  std::vector<DiscreteValues> allAssignments() const;

  /// @}
  /// @name Wrapper support
  /// @{

  /// Render as markdown table.
  std::string markdown(const KeyFormatter& keyFormatter = DefaultKeyFormatter,
                       const Names& names = {}) const override;

  /// Render as html table.
  std::string html(const KeyFormatter& keyFormatter = DefaultKeyFormatter,
                   const Names& names = {}) const override;


  /// @}
  /// @name HybridValues methods.
  /// @{

  /**
   * Calculate probability for HybridValues `x`.
   * Dispatches to DiscreteValues version.
   */
  double evaluate(const HybridValues& x) const override;

  using BaseConditional::operator();  ///< HybridValues version

  /**
   * Calculate log-probability log(evaluate(x)) for HybridValues `x`.
   * This is actually just -error(x).
   */
  double logProbability(const HybridValues& x) const override {
    return -error(x);
  }

  /**
   * logNormalizationConstant K is just zero, such that
   * logProbability(x) = log(evaluate(x)) = - error(x)
   * and hence error(x) = - log(evaluate(x)) > 0 for all x.
   */
  double logNormalizationConstant() const override { return 0.0; }

  /// @}

 protected:
  /// Internal version of choose
  DiscreteConditional::ADT choose(const DiscreteValues& given,
                                  bool forceComplete) const;

 private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(BaseFactor);
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(BaseConditional);
  }
#endif
};
// DiscreteConditional

// traits
template <>
struct traits<DiscreteConditional> : public Testable<DiscreteConditional> {};

}  // namespace gtsam
