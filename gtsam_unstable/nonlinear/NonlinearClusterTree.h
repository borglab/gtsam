/**
 * @file NonlinearClusterTree.h
 * @author Frank Dellaert
 * @date   March, 2016
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/inference/ClusterTree.h>

namespace gtsam {
class NonlinearClusterTree : public ClusterTree<NonlinearFactorGraph> {
 public:
  NonlinearClusterTree() {}

  struct NonlinearCluster : Cluster {
    // Given graph, index, add factors with specified keys into
    // Factors are erased in the graph
    // TODO(frank): fairly hacky and inefficient. Think about iterating the graph once instead
    NonlinearCluster(const VariableIndex& variableIndex, const KeyVector& keys,
                     NonlinearFactorGraph* graph) {
      for (const Key key : keys) {
        std::vector<NonlinearFactor::shared_ptr> factors;
        for (auto i : variableIndex[key])
          if (graph->at(i)) {
            factors.push_back(graph->at(i));
            graph->remove(i);
          }
        Cluster::addFactors(key, factors);
      }
    }

    GaussianFactorGraph::shared_ptr linearize(const Values& values) {
      return factors.linearize(values);
    }

    static NonlinearCluster* DownCast(const boost::shared_ptr<Cluster>& cluster) {
      auto nonlinearCluster = boost::dynamic_pointer_cast<NonlinearCluster>(cluster);
      if (!nonlinearCluster)
        throw std::runtime_error("Expected NonlinearCluster");
      return nonlinearCluster.get();
    }

    // linearize local custer factors straight into hessianFactor, which is returned
    // If no ordering given, uses colamd
    HessianFactor::shared_ptr linearizeToHessianFactor(
        const Values& values,
        const NonlinearFactorGraph::Dampen& dampen = nullptr) const {
      Ordering ordering;
      ordering = Ordering::ColamdConstrainedFirst(factors, orderedFrontalKeys, true);
      return factors.linearizeToHessianFactor(values, ordering, dampen);
    }

    // linearize local custer factors straight into hessianFactor, which is returned
    // If no ordering given, uses colamd
    HessianFactor::shared_ptr linearizeToHessianFactor(
        const Values& values, const Ordering& ordering,
        const NonlinearFactorGraph::Dampen& dampen = nullptr) const {
      return factors.linearizeToHessianFactor(values, ordering, dampen);
    }

    // Helper function: recursively eliminate subtree rooted at this Cluster into a Bayes net and factor on parent
    // TODO(frank): Use TBB to support deep trees and parallelism
    std::pair<GaussianBayesNet, HessianFactor::shared_ptr> linearizeAndEliminate(
        const Values& values,
        const HessianFactor::shared_ptr& localFactor) const {
      // Get contributions f(front) from children, as well as p(children|front)
      GaussianBayesNet bayesNet;
      for (const auto& child : children) {
        auto message = DownCast(child)->linearizeAndEliminate(values, &bayesNet);
        message->updateHessian(localFactor.get());
      }
      auto gaussianConditional = localFactor->eliminateCholesky(orderedFrontalKeys);
      bayesNet.add(gaussianConditional);
      return {bayesNet, localFactor};
    }

    // Recursively eliminate subtree rooted at this Cluster into a Bayes net and factor on parent
    // TODO(frank): Use TBB to support deep trees and parallelism
    std::pair<GaussianBayesNet, HessianFactor::shared_ptr> linearizeAndEliminate(
        const Values& values,
        const NonlinearFactorGraph::Dampen& dampen = nullptr) const {
      // Linearize and create HessianFactor f(front,separator)
      HessianFactor::shared_ptr localFactor = linearizeToHessianFactor(values, dampen);
      return linearizeAndEliminate(values, localFactor);
    }

    // Recursively eliminate subtree rooted at this Cluster into a Bayes net and factor on parent
    // TODO(frank): Use TBB to support deep trees and parallelism
    std::pair<GaussianBayesNet, HessianFactor::shared_ptr> linearizeAndEliminate(
        const Values& values, const Ordering& ordering,
        const NonlinearFactorGraph::Dampen& dampen = nullptr) const {
      // Linearize and create HessianFactor f(front,separator)
      HessianFactor::shared_ptr localFactor = linearizeToHessianFactor(values, ordering, dampen);
      return linearizeAndEliminate(values, localFactor);
    }

    // Recursively eliminate subtree rooted at this Cluster
    // Version that updates existing Bayes net and returns a new Hessian factor on parent clique
    // It is possible to pass in a nullptr for the bayesNet if only interested in the new factor
    HessianFactor::shared_ptr linearizeAndEliminate(
        const Values& values, GaussianBayesNet* bayesNet,
        const NonlinearFactorGraph::Dampen& dampen = nullptr) const {
      auto bayesNet_newFactor_pair = linearizeAndEliminate(values, dampen);
      if (bayesNet) {
        bayesNet->push_back(bayesNet_newFactor_pair.first);
      }
      return bayesNet_newFactor_pair.second;
    }

    // Recursively eliminate subtree rooted at this Cluster
    // Version that updates existing Bayes net and returns a new Hessian factor on parent clique
    // It is possible to pass in a nullptr for the bayesNet if only interested in the new factor
    HessianFactor::shared_ptr linearizeAndEliminate(
        const Values& values, GaussianBayesNet* bayesNet,
        const Ordering& ordering,
        const NonlinearFactorGraph::Dampen& dampen = nullptr) const {
      auto bayesNet_newFactor_pair = linearizeAndEliminate(values, ordering, dampen);
      if (bayesNet) {
        bayesNet->push_back(bayesNet_newFactor_pair.first);
      }
      return bayesNet_newFactor_pair.second;
    }
  };

  // Linearize and update linearization point with values
  Values updateCholesky(const Values& values) {
    GaussianBayesNet bayesNet;
    for (const auto& root : roots_) {
      auto result = NonlinearCluster::DownCast(root)->linearizeAndEliminate(values);
      bayesNet.push_back(result.first);
    }
    VectorValues delta = bayesNet.optimize();
    return values.retract(delta);
  }
};
}  // namespace gtsam
