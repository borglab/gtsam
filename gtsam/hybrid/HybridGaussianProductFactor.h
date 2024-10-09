/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file HybridGaussianProductFactor.h
 *  @date Oct 2, 2024
 *  @author Frank Dellaert
 *  @author Varun Agrawal
 */

#pragma once

#include <gtsam/discrete/DecisionTree.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/GaussianFactorGraph.h>

#include <iostream>

namespace gtsam {

class HybridGaussianFactor;

using GaussianFactorGraphValuePair = std::pair<GaussianFactorGraph, double>;

/// Alias for DecisionTree of GaussianFactorGraphs and their scalar sums
class GTSAM_EXPORT HybridGaussianProductFactor
    : public DecisionTree<Key, GaussianFactorGraphValuePair> {
 public:
  using Base = DecisionTree<Key, GaussianFactorGraphValuePair>;

  /// @name Constructors
  /// @{

  /// Default constructor
  HybridGaussianProductFactor() = default;

  /**
   * @brief Construct from a single factor
   * @tparam FACTOR Factor type
   * @param factor Shared pointer to the factor
   */
  template <class FACTOR>
  HybridGaussianProductFactor(const std::shared_ptr<FACTOR>& factor)
      : Base(GaussianFactorGraphValuePair{GaussianFactorGraph{factor}, 0.0}) {}

  /**
   * @brief Construct from DecisionTree
   * @param tree Decision tree to construct from
   */
  HybridGaussianProductFactor(Base&& tree) : Base(std::move(tree)) {}

  ///@}

  /// @name Operators
  ///@{

  /// Add GaussianFactor into HybridGaussianProductFactor
  HybridGaussianProductFactor operator+(
      const GaussianFactor::shared_ptr& factor) const;

  /// Add HybridGaussianFactor into HybridGaussianProductFactor
  HybridGaussianProductFactor operator+(
      const HybridGaussianFactor& factor) const;

  /// Add-assign operator for GaussianFactor
  HybridGaussianProductFactor& operator+=(
      const GaussianFactor::shared_ptr& factor);

  /// Add-assign operator for HybridGaussianFactor
  HybridGaussianProductFactor& operator+=(const HybridGaussianFactor& factor);

  ///@}

  /// @name Testable
  /// @{

  /**
   * @brief Print the HybridGaussianProductFactor
   * @param s Optional string to prepend
   * @param formatter Optional key formatter
   */
  void print(const std::string& s = "",
             const KeyFormatter& formatter = DefaultKeyFormatter) const;

  /**
   * @brief Check if this HybridGaussianProductFactor is equal to another
   * @param other The other HybridGaussianProductFactor to compare with
   * @param tol Tolerance for floating point comparisons
   * @return true if equal, false otherwise
   */
  bool equals(const HybridGaussianProductFactor& other,
              double tol = 1e-9) const;

  /// @}

  /// @name Other methods
  ///@{

  /**
   * @brief Remove empty GaussianFactorGraphs from the decision tree
   * @return A new HybridGaussianProductFactor with empty GaussianFactorGraphs
   * removed
   *
   * If any GaussianFactorGraph in the decision tree contains a nullptr, convert
   * that leaf to an empty GaussianFactorGraph with zero scalar sum. This is
   * needed because the DecisionTree will otherwise create a GaussianFactorGraph
   * with a single (null) factor, which doesn't register as null.
   */
  HybridGaussianProductFactor removeEmpty() const;

  ///@}

 private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }
#endif
};

// Testable traits
template <>
struct traits<HybridGaussianProductFactor>
    : public Testable<HybridGaussianProductFactor> {};

/**
 * Create a dummy overload of >> for GaussianFactorGraphValuePair
 * so that HybridGaussianProductFactor compiles
 * with the constructor
 * `DecisionTree(const std::vector<LabelC>& labelCs, const std::string& table)`.
 *
 * Needed to compile on Windows.
 */
std::istream& operator>>(std::istream& is, GaussianFactorGraphValuePair& pair);

}  // namespace gtsam
