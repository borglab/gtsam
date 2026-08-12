/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DiscreteBoundaryFactor.h
 * @date March, 2025
 * @author Varun Agrawal
 */

#pragma once

#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/hybrid/HybridNonlinearFactor.h>

namespace gtsam {

/* ************************************************************************ */
/**
 * @brief Take negative log-values, shift them so that the minimum value is 0,
 * and then exponentiate to create a TableFactor (not normalized yet!).
 *
 * @param discreteKeys The discrete keys for the resulting DecisionTreeFactor
 * @param errors DecisionTree of (unnormalized) errors.
 * @return TableFactor::shared_ptr
 */
GTSAM_EXPORT DecisionTreeFactor DiscreteFactorFromErrors(
    const DiscreteKeys& discreteKeys, const AlgebraicDecisionTree<Key>& errors);

/**
 * A discrete probabilistic factor which computes its values from the
 * continuous factors connected to it and a set of values.
 *
 * Helps in computing the discrete probabilities at the boundary of
 * discrete and continuous factors.
 *
 * @ingroup discrete
 */
class GTSAM_EXPORT DiscreteBoundaryFactor : public DecisionTreeFactor {
 protected:
  /**
   * @brief Helper method to compute the discrete values from continuous values.
   *
   * @param dkeys The discrete keys for this factor.
   * @param factors A decision tree of nonlinear factors and associated scalars,
   * which is used to compute the discrete values.
   * @param values The continuous values at which to compute the probabilities.
   * @return DecisionTreeFactor
   */
  static DecisionTreeFactor ComputeDiscreteBoundary(
      const DiscreteKeys& dkeys,
      const HybridNonlinearFactor::FactorValuePairs& factors,
      const gtsam::Values& values);

 public:
  // typedefs needed to play nice with gtsam
  typedef DiscreteBoundaryFactor This;
  typedef DecisionTreeFactor Base;  ///< Typedef to base class
  typedef std::shared_ptr<DiscreteBoundaryFactor> shared_ptr;

  /// @name Standard Constructors
  /// @{

  /** Default constructor for I/O */
  DiscreteBoundaryFactor() {}

  /** Constructor from DiscreteKeys and AlgebraicDecisionTree */
  DiscreteBoundaryFactor(const DiscreteKeys& keys, const ADT& potentials)
      : DecisionTreeFactor(keys, potentials) {}

  /** Constructor from DiscreteKeys and AlgebraicDecisionTree */
  DiscreteBoundaryFactor(const DiscreteKeys& dkeys,
                         const HybridNonlinearFactor::FactorValuePairs& factors,
                         const gtsam::Values& continuousVals)
      : DecisionTreeFactor(
            ComputeDiscreteBoundary(dkeys, factors, continuousVals)) {}

  /// @}
  /// @name Testable
  /// @{

  /// Print
  void print(
      const std::string& s = "DiscreteBoundaryFactor:\n",
      const KeyFormatter& formatter = DefaultKeyFormatter) const override;

  /// @}
  /// @name Standard Interface
  /// @{

  using DecisionTreeFactor::operator*;

  /// multiply with a scalar
  DiscreteFactor::shared_ptr operator*(double s) const override;

  /// multiply two factors
  DiscreteBoundaryFactor operator*(const DiscreteBoundaryFactor& f) const;

  using DecisionTreeFactor::operator/;

  /**
   * @brief Divide by factor f (safely).
   * Division of a factor \f$f(x, y)\f$ by another factor \f$g(y, z)\f$
   * results in a function which involves all keys
   * \f$(\frac{f}{g})(x, y, z) = f(x, y) / g(y, z)\f$
   *
   * @param f The DecisinTreeFactor to divide by.
   * @return DecisionTreeFactor
   */
  DiscreteBoundaryFactor operator/(const DiscreteBoundaryFactor& f) const;

  /// @}
  /// @name Advanced Interface
  /// @{

  /**
   * Apply unary operator (*this) "op" f
   * @param op a unary operator that operates on AlgebraicDecisionTree
   */
  DiscreteBoundaryFactor apply(Unary op) const;

  /**
   * Apply unary operator (*this) "op" f
   * @param op a unary operator that operates on AlgebraicDecisionTree. Takes
   * both the assignment and the value.
   */
  DiscreteBoundaryFactor apply(UnaryAssignment op) const;

  /**
   * Apply binary operator (*this) "op" f
   * @param f the second argument for op
   * @param op a binary operator that operates on AlgebraicDecisionTree
   */
  DiscreteBoundaryFactor apply(const DiscreteBoundaryFactor& f,
                               Binary op) const;

  /// @}

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }
#endif
};

// traits
template <>
struct traits<DiscreteBoundaryFactor>
    : public Testable<DiscreteBoundaryFactor> {};
}  // namespace gtsam
