/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file HybridFactor.h
 *  @date Mar 11, 2022
 *  @author Fan Jiang
 *  @author Varun Agrawal
 */

#pragma once

#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DecisionTree.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/discrete/TableFactor.h>
#include <gtsam/inference/Factor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <cstddef>
#include <string>
namespace gtsam {

class HybridValues;

/// Alias for DecisionTree of GaussianFactorGraphs
using GaussianFactorGraphTree = DecisionTree<Key, GaussianFactorGraph>;

KeyVector CollectKeys(const KeyVector &continuousKeys,
                      const DiscreteKeys &discreteKeys);
KeyVector CollectKeys(const KeyVector &keys1, const KeyVector &keys2);
DiscreteKeys CollectDiscreteKeys(const DiscreteKeys &key1,
                                 const DiscreteKeys &key2);

/**
 * Base class for *truly* hybrid probabilistic factors
 *
 * Examples:
 *  - HybridNonlinearFactor
 *  - HybridGaussianFactor
 *  - HybridGaussianConditional
 *
 * @ingroup hybrid
 */
class GTSAM_EXPORT HybridFactor : public Factor {
 public:
  /// Enum to help with categorizing hybrid factors.
  enum class Category { None, Discrete, Continuous, Hybrid };

 protected:
  /// Record what category of HybridFactor this is.
  Category category_ = Category::None;
  // Set of DiscreteKeys for this factor.
  DiscreteKeys discreteKeys_;
  /// Record continuous keys for book-keeping
  KeyVector continuousKeys_;

 public:
  // typedefs needed to play nice with gtsam
  typedef HybridFactor This;  ///< This class
  typedef std::shared_ptr<HybridFactor>
      shared_ptr;       ///< shared_ptr to this class
  typedef Factor Base;  ///< Our base class

  /// @name Standard Constructors
  /// @{

  /** Default constructor creates empty factor */
  HybridFactor() = default;

  /**
   * @brief Construct hybrid factor from continuous keys.
   *
   * @param keys Vector of continuous keys.
   */
  explicit HybridFactor(const KeyVector &keys);

  /**
   * @brief Construct hybrid factor from discrete keys.
   *
   * @param keys Vector of discrete keys.
   */
  explicit HybridFactor(const DiscreteKeys &discreteKeys);

  /**
   * @brief Construct a new Hybrid Factor object.
   *
   * @param continuousKeys Vector of keys for continuous variables.
   * @param discreteKeys Vector of keys for discrete variables.
   */
  HybridFactor(const KeyVector &continuousKeys,
               const DiscreteKeys &discreteKeys);

  /// @}
  /// @name Testable
  /// @{

  /// equals
  virtual bool equals(const HybridFactor &lf, double tol = 1e-9) const;

  /// print
  void print(
      const std::string &s = "HybridFactor\n",
      const KeyFormatter &formatter = DefaultKeyFormatter) const override;

  /// @}
  /// @name Standard Interface
  /// @{

  /// True if this is a factor of discrete variables only.
  bool isDiscrete() const { return category_ == Category::Discrete; }

  /// True if this is a factor of continuous variables only.
  bool isContinuous() const { return category_ == Category::Continuous; }

  /// True is this is a Discrete-Continuous factor.
  bool isHybrid() const { return category_ == Category::Hybrid; }

  /// Return the number of continuous variables in this factor.
  size_t nrContinuous() const { return continuousKeys_.size(); }

  /// Return the discrete keys for this factor.
  const DiscreteKeys &discreteKeys() const { return discreteKeys_; }

  /// Return only the continuous keys for this factor.
  const KeyVector &continuousKeys() const { return continuousKeys_; }

  /// Virtual class to compute tree of linear errors.
  virtual AlgebraicDecisionTree<Key> errorTree(
      const VectorValues &values) const = 0;

  /// @}

 protected:
  /// protected constructor to initialize the category
  HybridFactor(Category category) : category_(category) {}

 private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(category_);
    ar &BOOST_SERIALIZATION_NVP(discreteKeys_);
    ar &BOOST_SERIALIZATION_NVP(continuousKeys_);
  }
#endif
};
// HybridFactor

// traits
template <>
struct traits<HybridFactor> : public Testable<HybridFactor> {};

}  // namespace gtsam
