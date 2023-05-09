/**
 * @file    testLoopyBelief.cpp
 * @brief
 * @author Duy-Nguyen Ta
 * @date    Oct 11, 2013
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/inference/VariableIndex.h>

#include <fstream>
#include <iostream>

using namespace std;
using namespace boost;
using namespace gtsam;

/**
 * Loopy belief solver for graphs with only binary and unary factors
 */
class LoopyBelief {
  /** Star graph struct for each node, containing
   * - the star graph itself
   * - the product of original unary factors so we don't have to recompute it
   * later, and
   * - the factor indices of the corrected belief factors of the neighboring
   * nodes
   */
  typedef std::map<Key, size_t> CorrectedBeliefIndices;
  struct StarGraph {
    DiscreteFactorGraph::shared_ptr star;
    CorrectedBeliefIndices correctedBeliefIndices;
    DecisionTreeFactor::shared_ptr unary;
    VariableIndex varIndex_;
    StarGraph(const DiscreteFactorGraph::shared_ptr& _star,
              const CorrectedBeliefIndices& _beliefIndices,
              const DecisionTreeFactor::shared_ptr& _unary)
        : star(_star),
          correctedBeliefIndices(_beliefIndices),
          unary(_unary),
          varIndex_(*_star) {}

    void print(const std::string& s = "") const {
      cout << s << ":" << endl;
      star->print("Star graph: ");
      for (const auto& [key, _] : correctedBeliefIndices) {
        cout << "Belief factor index for " << key << ": "
             << correctedBeliefIndices.at(key) << endl;
      }
      if (unary) unary->print("Unary: ");
    }
  };

  typedef std::map<Key, StarGraph> StarGraphs;
  StarGraphs starGraphs_;  ///< star graph at each variable

 public:
  /** Constructor
   * Need all discrete keys to access node's cardinality for creating belief
   * factors
   * TODO: so troublesome!!
   */
  LoopyBelief(const DiscreteFactorGraph& graph,
              const std::map<Key, DiscreteKey>& allDiscreteKeys)
      : starGraphs_(buildStarGraphs(graph, allDiscreteKeys)) {}

  /// print
  void print(const std::string& s = "") const {
    cout << s << ":" << endl;
    for (const auto& [key, _] : starGraphs_) {
      starGraphs_.at(key).print("Node " + std::to_string(key) + ":");
    }
  }

  /// One step of belief propagation
  DiscreteFactorGraph::shared_ptr iterate(
      const std::map<Key, DiscreteKey>& allDiscreteKeys) {
    static const bool debug = false;
    DiscreteFactorGraph::shared_ptr beliefs(new DiscreteFactorGraph());
    std::map<Key, std::map<Key, DiscreteFactor::shared_ptr> > allMessages;
    // Eliminate each star graph
    for (const auto& [key, _] : starGraphs_) {
      //      cout << "***** Node " << key << "*****" << endl;
      // initialize belief to the unary factor from the original graph
      DecisionTreeFactor::shared_ptr beliefAtKey;

      // keep intermediate messages to divide later
      std::map<Key, DiscreteFactor::shared_ptr> messages;

      // eliminate each neighbor in this star graph one by one
      for (const auto& [neighbor, _] : starGraphs_.at(key).correctedBeliefIndices) {
        DiscreteFactorGraph subGraph;
        for (size_t factor : starGraphs_.at(key).varIndex_[neighbor]) {
          subGraph.push_back(starGraphs_.at(key).star->at(factor));
        }
        if (debug) subGraph.print("------- Subgraph:");
        const auto [dummyCond, message] =
            EliminateDiscrete(subGraph, Ordering{neighbor});
        // store the new factor into messages
        messages.insert(make_pair(neighbor, message));
        if (debug) message->print("------- Message: ");

        // Belief is the product of all messages and the unary factor
        // Incorporate new the factor to belief
        if (!beliefAtKey)
          beliefAtKey =
              std::dynamic_pointer_cast<DecisionTreeFactor>(message);
        else
          beliefAtKey = std::make_shared<DecisionTreeFactor>(
              (*beliefAtKey) *
              (*std::dynamic_pointer_cast<DecisionTreeFactor>(message)));
      }
      if (starGraphs_.at(key).unary)
        beliefAtKey = std::make_shared<DecisionTreeFactor>(
            (*beliefAtKey) * (*starGraphs_.at(key).unary));
      if (debug) beliefAtKey->print("New belief at key: ");
      // normalize belief
      double sum = 0.0;
      for (size_t v = 0; v < allDiscreteKeys.at(key).second; ++v) {
        DiscreteValues val;
        val[key] = v;
        sum += (*beliefAtKey)(val);
      }
      // TODO(kartikarcot): Check if this makes sense
      string sumFactorTable;
      for (size_t v = 0; v < allDiscreteKeys.at(key).second; ++v) {
        sumFactorTable = sumFactorTable + " " + std::to_string(sum);
      }
      DecisionTreeFactor sumFactor(allDiscreteKeys.at(key), sumFactorTable);
      if (debug) sumFactor.print("denomFactor: ");
      beliefAtKey =
          std::make_shared<DecisionTreeFactor>((*beliefAtKey) / sumFactor);
      if (debug) beliefAtKey->print("New belief at key normalized: ");
      beliefs->push_back(beliefAtKey);
      allMessages[key] = messages;
    }

    // Update corrected beliefs
    VariableIndex beliefFactors(*beliefs);
    for (const auto& [key, _] : starGraphs_) {
      std::map<Key, DiscreteFactor::shared_ptr> messages = allMessages[key];
      for (const auto& [neighbor, _] : starGraphs_.at(key).correctedBeliefIndices) {
              DecisionTreeFactor correctedBelief =
            (*std::dynamic_pointer_cast<DecisionTreeFactor>(
                beliefs->at(beliefFactors[key].front()))) /
            (*std::dynamic_pointer_cast<DecisionTreeFactor>(
                messages.at(neighbor)));
        if (debug) correctedBelief.print("correctedBelief: ");
        size_t beliefIndex =
            starGraphs_.at(neighbor).correctedBeliefIndices.at(key);
        starGraphs_.at(neighbor).star->replace(
            beliefIndex,
            std::make_shared<DecisionTreeFactor>(correctedBelief));
      }
    }

    if (debug) print("After update: ");

    return beliefs;
  }

 private:
  /**
   * Build star graphs for each node.
   */
  StarGraphs buildStarGraphs(
      const DiscreteFactorGraph& graph,
      const std::map<Key, DiscreteKey>& allDiscreteKeys) const {
    StarGraphs starGraphs;
    VariableIndex varIndex(graph);  ///< access to all factors of each node
    for (const auto& [key, _] : varIndex) {
      // initialize to multiply with other unary factors later
      DecisionTreeFactor::shared_ptr prodOfUnaries;

      // collect all factors involving this key in the original graph
      DiscreteFactorGraph::shared_ptr star(new DiscreteFactorGraph());
      for (size_t factorIndex : varIndex[key]) {
        star->push_back(graph.at(factorIndex));

        // accumulate unary factors
        if (graph.at(factorIndex)->size() == 1) {
          if (!prodOfUnaries)
            prodOfUnaries = std::dynamic_pointer_cast<DecisionTreeFactor>(
                graph.at(factorIndex));
          else
            prodOfUnaries = std::make_shared<DecisionTreeFactor>(
                *prodOfUnaries *
                (*std::dynamic_pointer_cast<DecisionTreeFactor>(
                    graph.at(factorIndex))));
        }
      }

      // add the belief factor for each neighbor variable to this star graph
      // also record the factor index for later modification
      KeySet neighbors = star->keys();
      neighbors.erase(key);
      CorrectedBeliefIndices correctedBeliefIndices;
      for (Key neighbor : neighbors) {
        // TODO: default table for keys with more than 2 values?
        string initialBelief;
        for (size_t v = 0; v < allDiscreteKeys.at(neighbor).second - 1; ++v) {
          initialBelief = initialBelief + "0.0 ";
        }
        initialBelief = initialBelief + "1.0";
        star->push_back(
            DecisionTreeFactor(allDiscreteKeys.at(neighbor), initialBelief));
        correctedBeliefIndices.insert(make_pair(neighbor, star->size() - 1));
      }
      starGraphs.insert(make_pair(
          key, StarGraph(star, correctedBeliefIndices, prodOfUnaries)));
    }
    return starGraphs;
  }
};

/* ************************************************************************* */

TEST_UNSAFE(LoopyBelief, construction) {
  // Variables: Cloudy, Sprinkler, Rain, Wet
  DiscreteKey C(0, 2), S(1, 2), R(2, 2), W(3, 2);

  // Map from key to DiscreteKey for building belief factor.
  // TODO: this is bad!
  std::map<Key, DiscreteKey> allKeys{{0, C}, {1, S}, {2, R}, {3, W}};

  // Build graph
  DecisionTreeFactor pC(C, "0.5 0.5");
  DiscreteConditional pSC(S | C = "0.5/0.5 0.9/0.1");
  DiscreteConditional pRC(R | C = "0.8/0.2 0.2/0.8");
  DecisionTreeFactor pSR(S & R, "0.0 0.9 0.9 0.99");

  DiscreteFactorGraph graph;
  graph.push_back(pC);
  graph.push_back(pSC);
  graph.push_back(pRC);
  graph.push_back(pSR);

  graph.print("graph: ");

  LoopyBelief solver(graph, allKeys);
  solver.print("Loopy belief: ");

  // Main loop
  for (size_t iter = 0; iter < 20; ++iter) {
    cout << "==================================" << endl;
    cout << "iteration: " << iter << endl;
    DiscreteFactorGraph::shared_ptr beliefs = solver.iterate(allKeys);
    beliefs->print();
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
