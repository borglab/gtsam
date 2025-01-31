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
#include <gtsam/inference/Ordering.h>

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

  using Unary = std::function<double(const double&)>;
  using UnaryAssignment =
      std::function<double(const Assignment<Key>&, const double&)>;
  using Binary = std::function<double(const double, const double)>;

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
  virtual bool equals(const DiscreteFactor& lf, double tol = 1e-9) const;

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

  /**
   * @brief Calculate probability for given values.
   * Calls specialized evaluation under the hood.
   *
   * Note: Uses Assignment<Key> as it is the base class of DiscreteValues.
   *
   * @param values Discrete assignment.
   * @return double
   */
  virtual double evaluate(const Assignment<Key>& values) const = 0;

  /// Find value for given assignment of values to variables
  double operator()(const DiscreteValues& values) const {
    return evaluate(values);
  }

  /// Error is just -log(value)
  virtual double error(const DiscreteValues& values) const;

  /**
   * The Factor::error simply extracts the \class DiscreteValues from the
   * \class HybridValues and calculates the error.
   */
  double error(const HybridValues& c) const override;

  /// Compute error for each assignment and return as a tree
  virtual AlgebraicDecisionTree<Key> errorTree() const;

  /// Multiply with a scalar
  virtual DiscreteFactor::shared_ptr operator*(double s) const = 0;

  /// Multiply in a DecisionTreeFactor and return the result as
  /// DecisionTreeFactor
  virtual DecisionTreeFactor operator*(const DecisionTreeFactor&) const = 0;

  /**
   * @brief Multiply in a DiscreteFactor and return the result as
   * DiscreteFactor, both via shared pointers.
   */
  virtual DiscreteFactor::shared_ptr multiply(
      const DiscreteFactor::shared_ptr& df) const = 0;

  /// divide by DiscreteFactor::shared_ptr f (safely)
  virtual DiscreteFactor::shared_ptr operator/(
      const DiscreteFactor::shared_ptr& df) const = 0;

  virtual DecisionTreeFactor toDecisionTreeFactor() const = 0;

  /// Create new factor by summing all values with the same separator values
  virtual DiscreteFactor::shared_ptr sum(size_t nrFrontals) const = 0;

  /// Create new factor by summing all values with the same separator values
  virtual DiscreteFactor::shared_ptr sum(const Ordering& keys) const = 0;

  /// Find the maximum value in the factor.
  virtual double max() const = 0;

  /// Create new factor by maximizing over all values with the same separator.
  virtual DiscreteFactor::shared_ptr max(size_t nrFrontals) const = 0;

  /// Create new factor by maximizing over all values with the same separator.
  virtual DiscreteFactor::shared_ptr max(const Ordering& keys) const = 0;

  /**
   * @brief Scale the factor values by the maximum
   * to prevent underflow/overflow.
   * 
   * @return DiscreteFactor::shared_ptr 
   */
  virtual DiscreteFactor::shared_ptr scale() const;

  /**
   * Get the number of non-zero values contained in this factor.
   * It could be much smaller than `prod_{key}(cardinality(key))`.
   */
  virtual uint64_t nrValues() const = 0;

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
#if GTSAM_ENABLE_BOOST_SERIALIZATION
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

}  // namespace gtsam
