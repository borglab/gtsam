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
 */

#pragma once

#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/inference/Factor.h>
#include <gtsam/nonlinear/Values.h>

#include <cstddef>
#include <string>
namespace gtsam {

KeyVector CollectKeys(const KeyVector &continuousKeys,
                      const DiscreteKeys &discreteKeys);
KeyVector CollectKeys(const KeyVector &keys1, const KeyVector &keys2);
DiscreteKeys CollectDiscreteKeys(const DiscreteKeys &key1,
                                 const DiscreteKeys &key2);

/**
 * Base class for hybrid probabilistic factors
 *
 * Examples:
 *  - HybridGaussianFactor
 *  - HybridDiscreteFactor
 *  - GaussianMixtureFactor
 *  - GaussianMixture
 */
class GTSAM_EXPORT HybridFactor : public Factor {
 private:
  bool isDiscrete_ = false;
  bool isContinuous_ = false;
  bool isHybrid_ = false;

  size_t nrContinuous_ = 0;

 protected:
  DiscreteKeys discreteKeys_;

  /// Record continuous keys for book-keeping
  KeyVector continuousKeys_;

 public:
  // typedefs needed to play nice with gtsam
  typedef HybridFactor This;  ///< This class
  typedef boost::shared_ptr<HybridFactor>
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

  /// Virtual destructor
  virtual ~HybridFactor() = default;

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
  bool isDiscrete() const { return isDiscrete_; }

  /// True if this is a factor of continuous variables only.
  bool isContinuous() const { return isContinuous_; }

  /// True is this is a Discrete-Continuous factor.
  bool isHybrid() const { return isHybrid_; }

  /// Return the number of continuous variables in this factor.
  size_t nrContinuous() const { return nrContinuous_; }

  /// Return vector of discrete keys.
  DiscreteKeys discreteKeys() const { return discreteKeys_; }

  /// @}
};
// HybridFactor

// traits
template <>
struct traits<HybridFactor> : public Testable<HybridFactor> {};

}  // namespace gtsam
