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
#include <gtsam/inference/Conditional-inst.h>

#include <memory>
#include <random>  // for std::mt19937_64
#include <string>
#include <vector>

#if GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/shared_ptr.hpp>
#endif

namespace gtsam {

/**
 * Discrete Conditional Density
 *
 * Stores its probabilities in any DiscreteFactor, such as a DecisionTreeFactor
 * or a TableFactor, and implements the conditional operations through the
 * DiscreteFactor interface. The stored factor always has the same keys, in the
 * same order, as the conditional: frontal keys first, then parents.
 *
 * @ingroup discrete
 */
class GTSAM_EXPORT DiscreteConditional
    : public DiscreteFactor,
      public Conditional<DiscreteFactor, DiscreteConditional> {
 public:
  // typedefs needed to play nice with gtsam
  typedef DiscreteConditional This;          ///< Typedef to this class
  typedef std::shared_ptr<This> shared_ptr;  ///< shared_ptr to this class
  typedef DiscreteFactor BaseFactor;  ///< Typedef to our factor base class
  typedef Conditional<BaseFactor, This>
      BaseConditional;  ///< Typedef to our conditional base class
  using ADT = AlgebraicDecisionTree<Key>;  ///< Decision tree of probabilities

  using Values = DiscreteValues;  ///< backwards compatibility

 protected:
  /// The probabilities, with our keys in our key order. Never modified in
  /// place, since copies share it.
  DiscreteFactor::shared_ptr factor_;

 public:
  /// @name Standard Constructors
  /// @{

  /// Default constructor needed for serialization.
  DiscreteConditional();

  /**
   * Construct from a factor by normalizing it, taking the first `nFrontals`
   * keys as frontals. The conditional uses the same representation as `f`.
   */
  DiscreteConditional(size_t nFrontals, const DiscreteFactor& f);

  /**
   * Construct from DiscreteKeys and AlgebraicDecisionTree, taking the first
   * `nFrontals` keys as frontals, in the order given.
   */
  DiscreteConditional(size_t nFrontals, const DiscreteKeys& keys,
                      const ADT& potentials);

  /**
   * Construct from DiscreteKeys and a factor holding the probabilities, taking
   * the first `nFrontals` keys as frontals, in the order given. The factor is
   * stored as is: it is neither copied nor normalized.
   */
  DiscreteConditional(size_t nFrontals, const DiscreteKeys& keys,
                      const DiscreteFactor::shared_ptr& potentials);

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

  /// Return the factor holding the probabilities.
  DiscreteFactor::shared_ptr factor() const { return factor_; }

  /// Log-probability is just -error(x).
  double logProbability(const DiscreteValues& x) const { return -error(x); }

  /// print index signature only
  void printSignature(
      const std::string& s = "Discrete Conditional: ",
      const KeyFormatter& formatter = DefaultKeyFormatter) const {
    static_cast<const BaseConditional*>(this)->print(s, formatter);
  }

  using BaseFactor::error;       ///< DiscreteValues version
  using BaseFactor::operator();  ///< DiscreteValues version

  /// Calculate probability for given values.
  double evaluate(const Assignment<Key>& values) const override {
    return factor_->evaluate(values);
  }

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
   * Sample from conditional, given missing variables
   * Example:
   *   std::mt19937_64 rng(42);
   *   DiscreteValues given = ...;
   *   size_t sample = dc.sample(given, &rng);
   *
   * @param parentsValues Known values of the parents
   * @param rng Pseudo-Random Number Generator.
   * @return sample from conditional
   */
  virtual size_t sample(const DiscreteValues& parentsValues,
                        std::mt19937_64* rng = nullptr) const;

  /// Single parent version.
  size_t sample(size_t parent_value, std::mt19937_64* rng = nullptr) const;

  /**
   * Sample from conditional, zero parent version
   * Example:
   *   std::mt19937_64 rng(42);
   *   auto sample = dc.sample(&rng);
   */
  size_t sample(std::mt19937_64* rng = nullptr) const;

  /**
   * @brief Return assignment for single frontal variable that maximizes value.
   * @param parentsValues Known assignments for the parents.
   * @return maximizing assignment for the frontal variable.
   */
  size_t argmax(const DiscreteValues& parentsValues = DiscreteValues()) const;

  /// @}
  /// @name DiscreteFactor interface, forwarded to the stored factor.
  /// Results are plain factors, not conditionals.
  /// @{

  /// Multiply with a scalar.
  DiscreteFactor::shared_ptr operator*(double s) const override {
    return *factor_ * s;
  }

  /// Multiply with a DecisionTreeFactor.
  DecisionTreeFactor operator*(const DecisionTreeFactor& f) const override {
    return *factor_ * f;
  }

  /// Multiply with any DiscreteFactor.
  DiscreteFactor::shared_ptr multiply(
      const DiscreteFactor::shared_ptr& f) const override;

  /// Divide by any DiscreteFactor.
  DiscreteFactor::shared_ptr operator/(
      const DiscreteFactor::shared_ptr& f) const override;

  /// Convert to a DecisionTreeFactor.
  DecisionTreeFactor toDecisionTreeFactor() const override {
    return factor_->toDecisionTreeFactor();
  }

  /// Convert to a TableFactor.
  TableFactor toTableFactor() const override;

  /// Sum out the first `nrFrontals` keys.
  DiscreteFactor::shared_ptr sum(size_t nrFrontals) const override {
    return factor_->sum(nrFrontals);
  }

  /// Sum out the given keys.
  DiscreteFactor::shared_ptr sum(const Ordering& keys) const override {
    return factor_->sum(keys);
  }

  /// Find the maximum value.
  double max() const override { return factor_->max(); }

  /// Max out the first `nrFrontals` keys.
  DiscreteFactor::shared_ptr max(size_t nrFrontals) const override {
    return factor_->max(nrFrontals);
  }

  /// Max out the given keys.
  DiscreteFactor::shared_ptr max(const Ordering& keys) const override {
    return factor_->max(keys);
  }

  /// Get the number of non-zero values in the stored factor.
  uint64_t nrValues() const override { return factor_->nrValues(); }

  /// Restrict the stored factor to the given assignment.
  DiscreteFactor::shared_ptr restrict(
      const DiscreteValues& assignment) const override {
    return factor_->restrict(assignment);
  }

  /// @}
  /// @name Advanced Interface
  /// @{

  /// Sample in place with optional PRNG, stores result in partial solution
  void sampleInPlace(DiscreteValues* parentsValues,
                     std::mt19937_64* rng = nullptr) const;

  /// Return all assignments for frontal variables.
  std::vector<DiscreteValues> frontalAssignments() const;

  /// Return all assignments for frontal *and* parent variables.
  std::vector<DiscreteValues> allAssignments() const;

  /// @}
  /// @name Wrapper support
  /// @{

  /// Output the probabilities as a decision tree to a graphviz stream.
  void dot(std::ostream& os,
           const KeyFormatter& keyFormatter = DefaultKeyFormatter,
           bool showZero = true) const {
    toDecisionTreeFactor().dot(os, keyFormatter, showZero);
  }

  /// Output the probabilities as a decision tree to a graphviz file.
  void dot(const std::string& name,
           const KeyFormatter& keyFormatter = DefaultKeyFormatter,
           bool showZero = true) const {
    toDecisionTreeFactor().dot(name, keyFormatter, showZero);
  }

  /// Output the probabilities as a decision tree in graphviz format.
  std::string dot(const KeyFormatter& keyFormatter = DefaultKeyFormatter,
                  bool showZero = true) const {
    return toDecisionTreeFactor().dot(keyFormatter, showZero);
  }

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
   * negLogConstant is just zero, such that
   * -logProbability(x) = -log(evaluate(x)) = error(x)
   * and hence error(x) > 0 for all x.
   * Thus -log(K) for the normalization constant k is 0.
   */
  double negLogConstant() const override;

  /**
   * Prune the conditional, keeping the `maxNrAssignments` most probable
   * assignments, and renormalize. Returns a DiscreteConditional with the same
   * keys and number of frontals.
   */
  DiscreteFactor::shared_ptr prune(size_t maxNrAssignments) const override;

  /**
   * @brief Remove the discrete modes whose assignments are given to us.
   * Only applies to discrete conditionals.
   *
   * Imperative method so we can update nodes in the Bayes net or Bayes tree.
   *
   * @param given The discrete modes whose assignments we know.
   */
  void removeDiscreteModes(const DiscreteValues& given);

  /// @}

 protected:
  /**
   * Return the entries of `given` assigned to the parent variables.
   * If `forceComplete` is true, throw when a parent is missing.
   */
  DiscreteValues parentValues(const DiscreteValues& given,
                              bool forceComplete) const;

  /// Print the stored values; decision trees print just the tree.
  void printValues(const KeyFormatter& formatter) const;

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /**
   * Serialization function. Archives written when DiscreteConditional derived
   * from DecisionTreeFactor cannot be read. Serializing requires the stored
   * factor type, e.g. DecisionTreeFactor or TableFactor, to be exported with
   * BOOST_CLASS_EXPORT_GUID.
   */
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(BaseFactor);
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(BaseConditional);
    ar& BOOST_SERIALIZATION_NVP(factor_);
  }
#endif
};
// DiscreteConditional

// traits
template <>
struct traits<DiscreteConditional> : public Testable<DiscreteConditional> {};

}  // namespace gtsam
