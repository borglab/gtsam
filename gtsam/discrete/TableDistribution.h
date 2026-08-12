/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file TableDistribution.h
 *  @date Dec 22, 2024
 *  @author Varun Agrawal
 */

#pragma once

#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/discrete/TableFactor.h>
#include <gtsam/inference/Conditional-inst.h>

#include <memory>
#include <string>
#include <vector>

namespace gtsam {

/**
 * Distribution which uses a SparseVector as the internal
 * representation, similar to the TableFactor.
 *
 * This is primarily used in the case when we have a clique in the BayesTree
 * which consists of all the discrete variables, e.g. in hybrid elimination.
 *
 * @ingroup discrete
 */
class GTSAM_EXPORT TableDistribution : public DiscreteConditional {
 private:
  TableFactor table_;

  typedef Eigen::SparseVector<double>::InnerIterator SparseIt;

 public:
  // typedefs needed to play nice with gtsam
  typedef TableDistribution This;            ///< Typedef to this class
  typedef std::shared_ptr<This> shared_ptr;  ///< shared_ptr to this class
  typedef DiscreteConditional
      BaseConditional;  ///< Typedef to our conditional base class

  using Values = DiscreteValues;  ///< backwards compatibility

  /// @name Standard Constructors
  /// @{

  /// Default constructor needed for serialization.
  TableDistribution() {}

  /// Construct from TableFactor.
  TableDistribution(const TableFactor& f);

  /**
   * Construct from DiscreteKeys and std::vector.
   */
  TableDistribution(const DiscreteKeys& keys,
                    const std::vector<double>& potentials);

  /**
   * Construct from single DiscreteKey and std::vector.
   */
  TableDistribution(const DiscreteKey& key,
                    const std::vector<double>& potentials)
      : TableDistribution(DiscreteKeys(key), potentials) {}

  /**
   * Construct from DiscreteKey and std::string.
   */
  TableDistribution(const DiscreteKeys& keys, const std::string& potentials);

  /**
   * Construct from single DiscreteKey and std::string.
   */
  TableDistribution(const DiscreteKey& key, const std::string& potentials)
      : TableDistribution(DiscreteKeys(key), potentials) {}

  /// @}
  /// @name Testable
  /// @{

  /// GTSAM-style print
  void print(
      const std::string& s = "Table Distribution: ",
      const KeyFormatter& formatter = DefaultKeyFormatter) const override;

  /// GTSAM-style equals
  bool equals(const DiscreteFactor& other, double tol = 1e-9) const override;

  /// @}
  /// @name Standard Interface
  /// @{

  /// Return the underlying TableFactor
  TableFactor table() const { return table_; }

  using BaseConditional::evaluate;  // HybridValues version

  /// Evaluate the conditional given the values.
  virtual double evaluate(const Assignment<Key>& values) const override {
    return table_.evaluate(values);
  }

  /// Create new factor by summing all values with the same separator values
  DiscreteFactor::shared_ptr sum(size_t nrFrontals) const override;

  /// Create new factor by summing all values with the same separator values
  DiscreteFactor::shared_ptr sum(const Ordering& keys) const override;

  /// Find the maximum value in the factor.
  double max() const override { return table_.max(); }

  /// Create new factor by maximizing over all values with the same separator.
  DiscreteFactor::shared_ptr max(size_t nrFrontals) const override;

  /// Create new factor by maximizing over all values with the same separator.
  DiscreteFactor::shared_ptr max(const Ordering& keys) const override;

  /// Multiply by scalar s
  DiscreteFactor::shared_ptr operator*(double s) const override;

  /// divide by DiscreteFactor::shared_ptr f (safely)
  DiscreteFactor::shared_ptr operator/(
      const DiscreteFactor::shared_ptr& f) const override;

  /**
   * @brief Return assignment that maximizes value.
   *
   * @return maximizing assignment for the variables.
   */
  DiscreteValues argmax() const;

  /**
   * sample
   * @param parentsValues Known values of the parents
   * @param rng Pseudo random number generator
   * @return sample from conditional
   */
  virtual size_t sample(const DiscreteValues& parentsValues,
                        std::mt19937_64* rng = nullptr) const override;

  /// @}
  /// @name Advanced Interface
  /// @{

  /// Prune the conditional
  virtual void prune(size_t maxNrAssignments) override;

  /// Get a DecisionTreeFactor representation.
  DecisionTreeFactor toDecisionTreeFactor() const override {
    return table_.toDecisionTreeFactor();
  }

  /// Get the number of non-zero values.
  uint64_t nrValues() const override { return table_.sparseTable().nonZeros(); }

  /// @}

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(BaseConditional);
    ar& BOOST_SERIALIZATION_NVP(table_);
  }
#endif
};
// TableDistribution

// traits
template <>
struct traits<TableDistribution> : public Testable<TableDistribution> {};

}  // namespace gtsam
