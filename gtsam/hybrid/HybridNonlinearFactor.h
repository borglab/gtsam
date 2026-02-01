/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   HybridNonlinearFactor.h
 * @brief  A set of nonlinear factors indexed by a set of discrete keys.
 * @author Kevin Doherty, kdoherty@mit.edu
 * @author Varun Agrawal
 * @date   December 2021
 */

#pragma once

#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Symbol.h>

#include <vector>

namespace gtsam {

/// Alias for a NoiseModelFactor shared pointer and double scalar pair.
using NonlinearFactorValuePair =
  std::pair<NoiseModelFactor::shared_ptr, double>;

/**
 * @brief Implementation of a discrete-conditioned hybrid factor.
 *
 * Implements a joint discrete-continuous factor where the discrete variable
 * serves to "select" a hybrid component corresponding to a NoiseModelFactor.
 *
 * This class stores all factors as HybridFactors which can then be typecast to
 * one of (NoiseModelFactor, GaussianFactor) which can then be checked to
 * perform the correct operation.
 *
 * In factor graphs the error function typically returns 0.5*|h(x)-z|^2, i.e.,
 * the negative log-likelihood for a Gaussian noise model.
 * In hybrid factor graphs we allow *adding* an arbitrary scalar dependent on
 * the discrete assignment.
 * For example, adding a 70/30 mode probability is supported by providing the
 * scalars $-log(.7)$ and $-log(.3)$.
 * Note that adding a common constant will not make any difference in the
 * optimization, so $-log(70)$ and $-log(30)$ work just as well.
 *
 * @ingroup hybrid
 */
class GTSAM_EXPORT HybridNonlinearFactor : public HybridFactor {
 public:
  using Base = HybridFactor;
  using This = HybridNonlinearFactor;
  using shared_ptr = std::shared_ptr<HybridNonlinearFactor>;
  using sharedFactor = std::shared_ptr<NoiseModelFactor>;

  /**
   * @brief typedef for DecisionTree which has Keys as node labels and
   * pairs of NoiseModelFactor & an arbitrary scalar as leaf nodes.
   */
  using FactorValuePairs = DecisionTree<Key, NonlinearFactorValuePair>;

 private:
  /// Decision tree of nonlinear factors indexed by discrete keys.
  FactorValuePairs factors_;

  /**
   * @brief HybridFactor method implementation.
   * Should not be used for this class.
   *
   * This version is for linear/Gaussian continuous values.
   * Use the overload taking gtsam::Values for nonlinear values.
   *
   * @param continuousValues 
   * @return AlgebraicDecisionTree<Key> 
   */
  AlgebraicDecisionTree<Key> errorTree(
    const VectorValues& continuousValues) const override {
    throw std::runtime_error(
      "HybridNonlinearFactor::errorTree: use errorTree(gtsam::Values) for nonlinear values.");
  }

public:
  /// @name Constructors
  /// @{

  /// Default constructor, mainly for serialization.
  HybridNonlinearFactor() = default;

  /**
   * @brief Construct a new HybridNonlinearFactor on a single discrete key,
   * providing the factors for each mode m as a vector of factors ϕ_m(x).
   * The value ϕ(x,m) for the factor is simply ϕ_m(x) (i.e. scalar part is 0.0).
   *
   * @param discreteKey The discrete key for the "mode", indexing components.
   * @param factors Vector of nonlinear factors, one for each mode.
   */
  HybridNonlinearFactor(
    const DiscreteKey& discreteKey,
    const std::vector<NoiseModelFactor::shared_ptr>& factors);

  /**
   * @brief Construct a new HybridNonlinearFactor on a single discrete key,
   * including a scalar error value for each mode m. The factors and scalars are
   * provided as a vector of pairs (ϕ_m(x), E_m).
   * The value ϕ(x,m) for the factor is now ϕ_m(x) + E_m.
   *
   * @param discreteKey The discrete key for the "mode", indexing components.
   * @param pairs Vector of nonlinear factor-scalar pairs, one per mode.
   */
  HybridNonlinearFactor(const DiscreteKey& discreteKey,
                        const std::vector<NonlinearFactorValuePair>& pairs);

  /**
   * @brief Construct a new HybridNonlinearFactor on a several discrete keys M,
   * including a scalar error value for each assignment m. The factors and
   * scalars are provided as a DecisionTree<Key> of pairs (ϕ_M(x), E_M).
   * The value ϕ(x,M) for the factor is again ϕ_m(x) + E_m.
   *
   * @param discreteKeys Discrete variables and their cardinalities.
   * @param factors The decision tree of nonlinear factor/scalar pairs.
   */
  HybridNonlinearFactor(const DiscreteKeys& discreteKeys,
                        const FactorValuePairs& factors);

  /// @}
  /// @name Standard Interface
  /// @{

  /**
   * @brief Compute error of the HybridNonlinearFactor as a tree.
   *
   * @param continuousValues The continuous gtsam::Values for which to compute the error.
   * @return AlgebraicDecisionTree<Key> A decision tree with the same discrete
   * keys as the factor, and leaf values as the error for each continuous component.
   */
  AlgebraicDecisionTree<Key> errorTree(const Values& continuousValues) const;

  /**
   * @brief Compute error of factor given both continuous and discrete values.
   *
   * @param continuousValues The continuous gtsam::Values.
   * @param assignment The assignment for the discrete keys.
   * @return double The error of this factor for the given assignment.
   */
  double error(const Values& continuousValues,
               const DiscreteValues& assignment) const;

  /**
   * @brief Compute error of factor given hybrid values.
   *
   * @param values The HybridValues containing continuous (gtsam::Values) and
   * discrete assignments.
   * @return double The error of this factor.
   */
  double error(const HybridValues& hybridValues) const override;

  /**
   * @brief Get the dimension of the factor (number of rows on linearization).
   * Returns the dimension of the first component factor found in the tree.
   * Assumes all component factors have the same dimension.
   * @return size_t
   */
  size_t dim() const;

  /// Getter for NonlinearFactor decision tree
  const FactorValuePairs& factors() const { return factors_; }

  /**
   * @brief Linearize specific nonlinear factors based on
   * the assignment in discreteValues.
   *
   * @param continuousValues The continuous values point to linearize around.
   * @param assignment The discrete assignment specifying which continuous
   * factors to linearize.
   * @return GaussianFactor::shared_ptr
   */
  GaussianFactor::shared_ptr linearize(const Values& continuousValues,
    const DiscreteValues& assignment) const;

  /// Linearize all the continuous factors to get a HybridGaussianFactor.
  std::shared_ptr<HybridGaussianFactor> linearize(
    const Values& continuousValues) const;

  /**
   * @brief Prune this factor based on the discrete probabilities.
   * Entries with probability 0 (or very small) in discreteProbs will lead to
   * pruning of corresponding branches in this factor.
   * 
   * @param discreteProbs A DecisionTreeFactor representing P(M) or P(M|...).
   * @return HybridNonlinearFactor::shared_ptr 
   */
  HybridNonlinearFactor::shared_ptr prune(
      const DecisionTreeFactor& discreteProbs) const;

  /**
   * @brief Restrict the factor to the given discrete values.
   *
   * If all discrete keys in this factor are assigned, the result will be
   * a NonlinearFactor (wrapped in a Factor::shared_ptr).
   * Otherwise, it will be a new HybridNonlinearFactor over the remaining
   * unassigned discrete keys.
   */
  std::shared_ptr<Factor> restrict(
      const DiscreteValues& assignment) const override;

  /// @}
  /// @name Testable
  /// @{

  /// print to stdout
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
    DefaultKeyFormatter) const override;

  /// Check equality
  bool equals(const HybridFactor& other, double tol = 1e-9) const override;

  /// @}

 private:
  /// Helper struct to assist private constructor below.
  struct ConstructorHelper;

  // Private constructor using ConstructorHelper above.
  HybridNonlinearFactor(const ConstructorHelper& helper);

#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    // Serialize base class
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    // Serialize derived class members
    ar& BOOST_SERIALIZATION_NVP(factors_);
  }
#endif
};

// traits
template <>
struct traits<HybridNonlinearFactor> : public Testable<HybridNonlinearFactor> {
};

}  // namespace gtsam
