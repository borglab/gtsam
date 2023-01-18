/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DiscreteLookupDAG.h
 * @date January, 2022
 * @author Frank dellaert
 */

#pragma once

#include <gtsam/discrete/DiscreteDistribution.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/FactorGraph.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace gtsam {

class DiscreteBayesNet;

/**
 * @brief DiscreteLookupTable table for max-product
 * @ingroup discrete
 *
 * Inherits from discrete conditional for convenience, but is not normalized.
 * Is used in the max-product algorithm.
 */
class GTSAM_EXPORT DiscreteLookupTable : public DiscreteConditional {
 public:
  using This = DiscreteLookupTable;
  using shared_ptr = std::shared_ptr<This>;
  using BaseConditional = Conditional<DecisionTreeFactor, This>;

  /**
   * @brief Construct a new Discrete Lookup Table object
   *
   * @param nFrontals number of frontal variables
   * @param keys a sorted list of gtsam::Keys
   * @param potentials the algebraic decision tree with lookup values
   */
  DiscreteLookupTable(size_t nFrontals, const DiscreteKeys& keys,
                      const ADT& potentials)
      : DiscreteConditional(nFrontals, keys, potentials) {}

  /// GTSAM-style print
  void print(
      const std::string& s = "Discrete Lookup Table: ",
      const KeyFormatter& formatter = DefaultKeyFormatter) const override;

  /**
   * @brief return assignment for single frontal variable that maximizes value.
   * @param parentsValues Known assignments for the parents.
   * @return maximizing assignment for the frontal variable.
   */
  size_t argmax(const DiscreteValues& parentsValues) const;

  /**
   * @brief Calculate assignment for frontal variables that maximizes value.
   * @param (in/out) parentsValues Known assignments for the parents.
   */
  void argmaxInPlace(DiscreteValues* parentsValues) const;
};

/** A DAG made from lookup tables, as defined above. */
class GTSAM_EXPORT DiscreteLookupDAG : public BayesNet<DiscreteLookupTable> {
 public:
  using Base = BayesNet<DiscreteLookupTable>;
  using This = DiscreteLookupDAG;
  using shared_ptr = std::shared_ptr<This>;

  /// @name Standard Constructors
  /// @{

  /// Construct empty DAG.
  DiscreteLookupDAG() {}

  /// Create from BayesNet with LookupTables
  static DiscreteLookupDAG FromBayesNet(const DiscreteBayesNet& bayesNet);

  /// Destructor
  virtual ~DiscreteLookupDAG() {}

  /// @}

  /// @name Testable
  /// @{

  /** Check equality */
  bool equals(const This& bn, double tol = 1e-9) const;

  /// @}

  /// @name Standard Interface
  /// @{

  /** Add a DiscreteLookupTable */
  template <typename... Args>
  void add(Args&&... args) {
    emplace_shared<DiscreteLookupTable>(std::forward<Args>(args)...);
  }

  /**
   * @brief argmax by back-substitution, optionally given certain variables.
   *
   * Assumes the DAG is reverse topologically sorted, i.e. last
   * conditional will be optimized first *and* that the
   * DAG does not contain any conditionals for the given variables. If the DAG
   * resulted from eliminating a factor graph, this is true for the elimination
   * ordering.
   *
   * @return given assignment extended w. optimal assignment for all variables.
   */
  DiscreteValues argmax(DiscreteValues given = DiscreteValues()) const;
  /// @}

 private:
  /** Serialization function */
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }
#endif
};

// traits
template <>
struct traits<DiscreteLookupDAG> : public Testable<DiscreteLookupDAG> {};

}  // namespace gtsam
