/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file HybridFactorGraph.h
 * @date December 2021
 * @brief Custom hybrid factor graph for discrete + continuous factors
 * @author Kevin Doherty, Varun Agrawal
 */

#pragma once

#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/hybrid/DCFactorGraph.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <string>

namespace gtsam {

class HybridFactorGraph {
 protected:
  // Separate internal factor graphs for different types of factors
  NonlinearFactorGraph nonlinear_graph_;
  DiscreteFactorGraph discrete_graph_;
  DCFactorGraph dc_graph_;

 public:
  HybridFactorGraph();

  /**
   * Add a nonlinear factor to the internal nonlinear factor graph
   * @param nonlinear_factor - the factor to add
   */
  template <typename NonlinearFactorType>
  void push_nonlinear(const NonlinearFactorType &nonlinear_factor) {
    nonlinear_graph_.push_back(
        boost::make_shared<NonlinearFactorType>(nonlinear_factor));
  }

  /**
   * Add a nonlinear factor *pointer* to the internal nonlinear factor graph
   * @param nonlinear_factor - boost::shared_ptr to the factor to add
   */
  void push_nonlinear(boost::shared_ptr<NonlinearFactor> nonlinear_factor);

  /**
   * Add a discrete factor to the internal discrete graph
   * @param discrete_factor - the factor to add
   */
  template <typename DiscreteFactorType>
  void push_discrete(const DiscreteFactorType &discrete_factor) {
    discrete_graph_.push_back(
        boost::make_shared<DiscreteFactorType>(discrete_factor));
  }

  /**
   * Add a discrete factor *pointer* to the internal discrete graph
   * @param discrete_factor - boost::shared_ptr to the factor to add
   */
  void push_discrete(boost::shared_ptr<DiscreteFactor> discrete_factor);

  /**
   * Add a discrete-continuous (DC) factor to the internal DC graph
   * @param dc_factor - the factor to add
   */
  template <typename HybridFactorType>
  void push_dc(const HybridFactorType &dc_factor) {
    dc_graph_.push_back(boost::make_shared<HybridFactorType>(dc_factor));
  }

  /**
   * Add a discrete-continuous (DC) factor *pointer* to the internal DC graph
   * @param dc_factor - boost::shared_ptr to the factor to add
   */
  void push_dc(boost::shared_ptr<HybridFactor> dc_factor);

  /**
   * Simply prints the factor graph.
   */
  void print(const std::string &str = "HybridFactorGraph",
             const KeyFormatter &keyFormatter = DefaultKeyFormatter) const;

  /**
   * Mimics the gtsam::FactorGraph API: retrieve the keys from each internal
   * factor graph. Internally uses FastSet::merge(const FastSet &other) to
   * combine sets from the different member factor graphs.
   *
   * @return the (aggregate) set of keys in all of the internal factor graphs.
   */
  FastSet<Key> keys() const;

  /**
   * Utility for retrieving the internal nonlinear factor graph
   * @return the member variable nolinearGraph_
   */
  NonlinearFactorGraph nonlinearGraph() const;

  /**
   * Utility for retrieving the internal discrete factor graph
   * @return the member variable discrete_graph_
   */
  DiscreteFactorGraph discreteGraph() const;

  /**
   * Utility for retrieving the internal DC factor graph
   * @return the member variable dc_graph_
   */
  DCFactorGraph dcGraph() const;

  /**
   * @return true if all internal graphs are empty
   */
  bool empty() const;

  /**
   * @return true if all internal graphs of `this` are equal to those of `other`
   */
  bool equals(const HybridFactorGraph &other, double tol = 1e-9) const;

  /**
   * @return the total number of factors across all internal graphs
   */
  size_t size() const;

  /**
   * @return the total number of factors in the nonlinear factor graph
   */
  size_t size_nonlinear() const;

  /**
   * @return the total number of factors in the discrete factor graph
   */
  size_t size_discrete() const;

  /**
   * @return the total number of factors in the DC factor graph
   */
  size_t size_dc() const;

  /**
   * Clears all internal factor graphs
   */
  void clear();
};

}  // namespace gtsam