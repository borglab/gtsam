/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file HybridConditional.h
 *  @date Mar 11, 2022
 *  @author Fan Jiang
 */

#pragma once

#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/hybrid/HybridGaussianConditional.h>
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
#include <gtsam/inference/Conditional.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/GaussianConditional.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <vector>

namespace gtsam {

/**
 * Hybrid Conditional Density
 *
 * As a type-erased variant of:
 * - DiscreteConditional
 * - GaussianConditional
 * - HybridGaussianConditional
 *
 * The reason why this is important is that `Conditional<T>` is a CRTP class.
 * CRTP is static polymorphism such that all CRTP classes, while bearing the
 * same name, are different classes not sharing a vtable. This prevents them
 * from being contained in any container, and thus it is impossible to
 * dynamically cast between them. A better option, as illustrated here, is
 * treating them as an implementation detail - such that the hybrid mechanism
 * does not know what is inside the HybridConditional. This prevents us from
 * having diamond inheritances, and neutralized the need to change other
 * components of GTSAM to make hybrid elimination work.
 *
 * A great reference to the type-erasure pattern is Eduardo Madrid's CppCon
 * talk (https://www.youtube.com/watch?v=s082Qmd_nHs).
 *
 * @ingroup hybrid
 */
class GTSAM_EXPORT HybridConditional
    : public HybridFactor,
      public Conditional<HybridFactor, HybridConditional> {
 public:
  // typedefs needed to play nice with gtsam
  typedef HybridConditional This;            ///< Typedef to this class
  typedef std::shared_ptr<This> shared_ptr;  ///< shared_ptr to this class
  typedef HybridFactor BaseFactor;  ///< Typedef to our factor base class
  typedef Conditional<BaseFactor, This>
      BaseConditional;  ///< Typedef to our conditional base class

 protected:
  /// Type-erased pointer to the inner type
  std::shared_ptr<Factor> inner_;

 public:
  /// @name Standard Constructors
  /// @{

  /// Default constructor needed for serialization.
  HybridConditional() = default;

  /**
   * @brief Construct a new Hybrid Conditional object
   *
   * @param continuousKeys Vector of keys for continuous variables.
   * @param discreteKeys Keys and cardinalities for discrete variables.
   * @param nFrontals The number of frontal variables in the conditional.
   */
  HybridConditional(const KeyVector& continuousKeys,
                    const DiscreteKeys& discreteKeys, size_t nFrontals)
      : BaseFactor(continuousKeys, discreteKeys), BaseConditional(nFrontals) {}

  /**
   * @brief Construct a new Hybrid Conditional object
   *
   * @param continuousFrontals Vector of keys for continuous variables.
   * @param discreteFrontals Keys and cardinalities for discrete variables.
   * @param continuousParents Vector of keys for parent continuous variables.
   * @param discreteParents Keys and cardinalities for parent discrete
   * variables.
   */
  HybridConditional(const KeyVector& continuousFrontals,
                    const DiscreteKeys& discreteFrontals,
                    const KeyVector& continuousParents,
                    const DiscreteKeys& discreteParents);

  /**
   * @brief Construct a new Hybrid Conditional object
   *
   * @param continuousConditional Conditional used to create the
   * HybridConditional.
   */
  HybridConditional(
      const std::shared_ptr<GaussianConditional>& continuousConditional);

  /**
   * @brief Construct a new Hybrid Conditional object
   *
   * @param discreteConditional Conditional used to create the
   * HybridConditional.
   */
  HybridConditional(
      const std::shared_ptr<DiscreteConditional>& discreteConditional);

  /**
   * @brief Construct a new Hybrid Conditional object
   *
   * @param hybridGaussianCond Hybrid Gaussian Conditional used to create the
   * HybridConditional.
   */
  HybridConditional(
      const std::shared_ptr<HybridGaussianConditional>& hybridGaussianCond);

  /// @}
  /// @name Testable
  /// @{

  /// GTSAM-style print
  void print(
      const std::string& s = "Hybrid Conditional: ",
      const KeyFormatter& formatter = DefaultKeyFormatter) const override;

  /// GTSAM-style equals
  bool equals(const HybridFactor& other, double tol = 1e-9) const override;

  /// @}
  /// @name Standard Interface
  /// @{

  /**
   * @brief Return HybridConditional as a HybridGaussianConditional
   * @return nullptr if not a conditional
   * @return HybridGaussianConditional::shared_ptr otherwise
   */
  HybridGaussianConditional::shared_ptr asHybrid() const {
    return std::dynamic_pointer_cast<HybridGaussianConditional>(inner_);
  }

  /**
   * @brief Return HybridConditional as a GaussianConditional
   * @return nullptr if not a GaussianConditional
   * @return GaussianConditional::shared_ptr otherwise
   */
  GaussianConditional::shared_ptr asGaussian() const {
    return std::dynamic_pointer_cast<GaussianConditional>(inner_);
  }

  /**
   * @brief Return conditional as a DiscreteConditional
   * @return nullptr if not a DiscreteConditional
   * @return DiscreteConditional::shared_ptr
   */
  DiscreteConditional::shared_ptr asDiscrete() const {
    return std::dynamic_pointer_cast<DiscreteConditional>(inner_);
  }

  /// Get the type-erased pointer to the inner type
  std::shared_ptr<Factor> inner() const { return inner_; }

  /// Return the error of the underlying conditional.
  double error(const HybridValues& values) const override;

  /**
   * @brief Compute error of the HybridConditional as a tree.
   *
   * @param continuousValues The continuous VectorValues.
   * @return AlgebraicDecisionTree<Key> A decision tree with the same keys
   * as the conditionals involved, and leaf values as the error.
   */
  virtual AlgebraicDecisionTree<Key> errorTree(
      const VectorValues& values) const override;

  /// Return the log-probability (or density) of the underlying conditional.
  double logProbability(const HybridValues& values) const override;

  /**
   * Return the log normalization constant.
   * Note this is 0.0 for discrete and hybrid conditionals, but depends
   * on the continuous parameters for Gaussian conditionals.
   */
  double logNormalizationConstant() const override;

  /// Return the probability (or density) of the underlying conditional.
  double evaluate(const HybridValues& values) const override;

  /// Check if VectorValues `measurements` contains all frontal keys.
  bool frontalsIn(const VectorValues& measurements) const {
    for (Key key : frontals()) {
      if (!measurements.exists(key)) {
        return false;
      }
    }
    return true;
  }

  /// @}

 private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(BaseFactor);
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(BaseConditional);
    ar& BOOST_SERIALIZATION_NVP(inner_);

    // register the various casts based on the type of inner_
    // https://www.boost.org/doc/libs/1_80_0/libs/serialization/doc/serialization.html#runtimecasting
    if (isDiscrete()) {
      boost::serialization::void_cast_register<DiscreteConditional, Factor>(
          static_cast<DiscreteConditional*>(NULL), static_cast<Factor*>(NULL));
    } else if (isContinuous()) {
      boost::serialization::void_cast_register<GaussianConditional, Factor>(
          static_cast<GaussianConditional*>(NULL), static_cast<Factor*>(NULL));
    } else {
      boost::serialization::void_cast_register<HybridGaussianConditional,
                                               Factor>(
          static_cast<HybridGaussianConditional*>(NULL),
          static_cast<Factor*>(NULL));
    }
  }
#endif

};  // HybridConditional

// traits
template <>
struct traits<HybridConditional> : public Testable<HybridConditional> {};

}  // namespace gtsam
