/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file DiscreteFactor.h
 *  @date Feb 14, 2011
 *  @author Duy-Nguyen Ta
 *  @author Frank Dellaert
 */

#pragma once

#include <gtsam/base/Testable.h>
#include <gtsam/discrete/AlgebraicDecisionTree.h>
#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/inference/Factor.h>

#include <string>
namespace gtsam {

class DecisionTreeFactor;
class DiscreteConditional;
class HybridValues;

/**
 * Base class for discrete probabilistic factors
 * The most general one is the derived DecisionTreeFactor
 *
 * @ingroup discrete
 */
class GTSAM_EXPORT DiscreteFactor : public Factor {
 public:
  // typedefs needed to play nice with gtsam
  typedef DiscreteFactor This;  ///< This class
  typedef std::shared_ptr<DiscreteFactor>
      shared_ptr;       ///< shared_ptr to this class
  typedef Factor Base;  ///< Our base class

  using Values = DiscreteValues;  ///< backwards compatibility

 protected:
  /// Map of Keys and their cardinalities.
  std::map<Key, size_t> cardinalities_;

 public:
  /// @name Standard Constructors
  /// @{

  /** Default constructor creates empty factor */
  DiscreteFactor() {}

  /**
   * Construct from container of keys and map of cardinalities.
   * This constructor is used internally from derived factor constructors,
   * either from a container of keys or from a boost::assign::list_of.
   */
  template <typename CONTAINER>
  DiscreteFactor(const CONTAINER& keys,
                 const std::map<Key, size_t> cardinalities = {})
      : Base(keys), cardinalities_(cardinalities) {}

  /// @}
  /// @name Testable
  /// @{

  /// equals
  virtual bool equals(const DiscreteFactor& lf, double tol = 1e-9) const = 0;

  /// print
  void print(
      const std::string& s = "DiscreteFactor\n",
      const KeyFormatter& formatter = DefaultKeyFormatter) const override {
    Base::print(s, formatter);
  }

  /// @}
  /// @name Standard Interface
  /// @{

  /// Return all the discrete keys associated with this factor.
  DiscreteKeys discreteKeys() const;

  std::map<Key, size_t> cardinalities() const { return cardinalities_; }

  size_t cardinality(Key j) const { return cardinalities_.at(j); }

  /// Find value for given assignment of values to variables
  virtual double operator()(const DiscreteValues&) const = 0;

  /// Error is just -log(value)
  virtual double error(const DiscreteValues& values) const;

  /**
   * The Factor::error simply extracts the \class DiscreteValues from the
   * \class HybridValues and calculates the error.
   */
  double error(const HybridValues& c) const override;

  /// Compute error for each assignment and return as a tree
  virtual AlgebraicDecisionTree<Key> errorTree() const;

  /// Multiply in a DecisionTreeFactor and return the result as
  /// DecisionTreeFactor
  virtual DecisionTreeFactor operator*(const DecisionTreeFactor&) const = 0;

  virtual DecisionTreeFactor toDecisionTreeFactor() const = 0;

  /// @}
  /// @name Wrapper support
  /// @{

  /// Translation table from values to strings.
  using Names = DiscreteValues::Names;

  /**
   * @brief Render as markdown table
   *
   * @param keyFormatter GTSAM-style Key formatter.
   * @param names optional, category names corresponding to choices.
   * @return std::string a markdown string.
   */
  virtual std::string markdown(
      const KeyFormatter& keyFormatter = DefaultKeyFormatter,
      const Names& names = {}) const = 0;

  /**
   * @brief Render as html table
   *
   * @param keyFormatter GTSAM-style Key formatter.
   * @param names optional, category names corresponding to choices.
   * @return std::string a html string.
   */
  virtual std::string html(
      const KeyFormatter& keyFormatter = DefaultKeyFormatter,
      const Names& names = {}) const = 0;

  /// @}

 private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(cardinalities_);
  }
#endif
};
// DiscreteFactor

// traits
template <>
struct traits<DiscreteFactor> : public Testable<DiscreteFactor> {};

/**
 * @brief Normalize a set of log probabilities.
 *
 * Normalizing a set of log probabilities in a numerically stable way is
 * tricky. To avoid overflow/underflow issues, we compute the largest
 * (finite) log probability and subtract it from each log probability before
 * normalizing. This comes from the observation that if:
 *    p_i = exp(L_i) / ( sum_j exp(L_j) ),
 * Then,
 *    p_i = exp(Z) exp(L_i - Z) / (exp(Z) sum_j exp(L_j - Z)),
 *        = exp(L_i - Z) / ( sum_j exp(L_j - Z) )
 *
 * Setting Z = max_j L_j, we can avoid numerical issues that arise when all
 * of the (unnormalized) log probabilities are either very large or very
 * small.
 */
std::vector<double> expNormalize(const std::vector<double>& logProbs);

}  // namespace gtsam
