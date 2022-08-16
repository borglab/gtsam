/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file HybridLookupDAG.h
 * @date Aug, 2022
 * @author Shangjie Xue
 */

#pragma once

#include <gtsam/discrete/DiscreteDistribution.h>
#include <gtsam/discrete/DiscreteLookupDAG.h>
#include <gtsam/hybrid/HybridConditional.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/FactorGraph.h>

#include <boost/shared_ptr.hpp>
#include <string>
#include <utility>
#include <vector>

namespace gtsam {

/**
 * @brief HybridLookupTable table for max-product
 *
 * Similar to DiscreteLookupTable, inherits from hybrid conditional for
 * convenience. Is used in the max-product algorithm.
 */
class GTSAM_EXPORT HybridLookupTable : public HybridConditional {
 public:
  using Base = HybridConditional;
  using This = HybridLookupTable;
  using shared_ptr = boost::shared_ptr<This>;
  using BaseConditional = Conditional<DecisionTreeFactor, This>;

  /**
   * @brief Construct a new Hybrid Lookup Table object form a HybridConditional.
   *
   * @param conditional input hybrid conditional
   */
  HybridLookupTable(HybridConditional& conditional) : Base(conditional){};

  /**
   * @brief Calculate assignment for frontal variables that maximizes value.
   * @param (in/out) parentsValues Known assignments for the parents.
   */
  void argmaxInPlace(HybridValues* parentsValues) const;
};

/** A DAG made from hybrid lookup tables, as defined above. Similar to
 * DiscreteLookupDAG */
class GTSAM_EXPORT HybridLookupDAG : public BayesNet<HybridLookupTable> {
 public:
  using Base = BayesNet<HybridLookupTable>;
  using This = HybridLookupDAG;
  using shared_ptr = boost::shared_ptr<This>;

  /// @name Standard Constructors
  /// @{

  /// Construct empty DAG.
  HybridLookupDAG() {}

  /// Create from BayesNet with LookupTables
  static HybridLookupDAG FromBayesNet(const HybridBayesNet& bayesNet);

  /// Destructor
  virtual ~HybridLookupDAG() {}

  /// @}

  /// @name Standard Interface
  /// @{

  /** Add a DiscreteLookupTable */
  template <typename... Args>
  void add(Args&&... args) {
    emplace_shared<HybridLookupTable>(std::forward<Args>(args)...);
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
  HybridValues argmax(HybridValues given = HybridValues()) const;
  /// @}

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }
};

// traits
template <>
struct traits<HybridLookupDAG> : public Testable<HybridLookupDAG> {};

}  // namespace gtsam
