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
  typedef SparseVector::InnerIterator SparseIt;

  /// The stored factor, which is always a TableFactor.
  const TableFactor& tableFactor() const {
    return static_cast<const TableFactor&>(*factor_);
  }

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

  /// Construct from TableFactor, normalizing it.
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
      const KeyFormatter& formatter = DefaultKeyFormatter) const override {
    BaseConditional::print(s, formatter);
  }

  /// GTSAM-style equals
  bool equals(const DiscreteFactor& other, double tol = 1e-9) const override;

  /// @}
  /// @name Standard Interface
  /// @{

  /// Return the underlying TableFactor
  TableFactor table() const { return tableFactor(); }

  /**
   * @brief Return assignment that maximizes value.
   *
   * @return maximizing assignment for the variables.
   */
  DiscreteValues argmax() const;

  /// @}
  /// @name Advanced Interface
  /// @{

  /// Prune the conditional and renormalize, returning a TableDistribution.
  DiscreteFactor::shared_ptr prune(size_t maxNrAssignments) const override;

  /// @}

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(BaseConditional);
  }
#endif
};
// TableDistribution

// traits
template <>
struct traits<TableDistribution> : public Testable<TableDistribution> {};

}  // namespace gtsam
