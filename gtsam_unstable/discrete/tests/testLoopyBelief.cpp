/**
 * @file 	 testLoopyBelief.cpp
 * @brief  
 * @author Duy-Nguyen Ta
 * @date 	 Oct 11, 2013
 */

#include <gtsam/inference/VariableIndex.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <CppUnitLite/TestHarness.h>
#include <boost/range/adaptor/map.hpp>
#include <boost/assign/list_of.hpp>
#include <iostream>
#include <fstream>

using namespace std;
using namespace boost;
using namespace boost::assign;
using namespace gtsam;

/**
 * Loopy belief solver for graphs with only binary and unary factors
 */
class LoopyBelief {

  /** Star graph struct for each node, containing
   * - the star graph itself
   * - the product of original unary factors so we don't have to recompute it later, and
   * - the factor indices of the corrected belief factors of the neighboring nodes
   */
  typedef std::map<Key, size_t> CorrectedBeliefIndices;
  struct StarGraph {
    DiscreteFactorGraph::shared_ptr star;
    DecisionTreeFactor::shared_ptr unary;
    CorrectedBeliefIndices correctedBeliefIndices;
    StarGraph(const DiscreteFactorGraph::shared_ptr& _star,
        const DecisionTreeFactor::shared_ptr& _unary,
        const CorrectedBeliefIndices& _beliefIndices) :
        star(_star), unary(_unary), correctedBeliefIndices(_beliefIndices) {
    }
  };

  typedef std::map<Key, StarGraph> StarGraphs;
  StarGraphs starGraphs_; ///< star graph at each variable

public:
  /** Constructor
   * Need all discrete keys to access node's cardinality for creating belief factors
   * TODO: so troublesome!!
   */
  LoopyBelief(const DiscreteFactorGraph& graph,
      const std::map<Key, DiscreteKey>& allDiscreteKeys) :
      starGraphs_(buildStarGraphs(graph, allDiscreteKeys)) {
  }

  /// One step of belief propagation
  DiscreteFactorGraph::shared_ptr iterate() {
    static DiscreteConditional::shared_ptr dummyCond; // unused by-product of elimination
    DiscreteFactorGraph::shared_ptr beliefs(new DiscreteFactorGraph());
    // Eliminate each star graph
    BOOST_FOREACH(Key key, starGraphs_ | boost::adaptors::map_keys) {
      // initialize belief to the unary factor from the original graph
      DecisionTreeFactor beliefAtKey = *starGraphs_.at(key).unary;

      // keep intermediate messages to divide later
      std::map<Key, DiscreteFactor::shared_ptr> messages;

      // eliminate each neighbor in this star graph one by one
      BOOST_FOREACH(Key neighbor, starGraphs_.at(key).correctedBeliefIndices | boost::adaptors::map_keys) {
        DiscreteFactor::shared_ptr factor;
        boost::tie(dummyCond, factor) = EliminateDiscrete(
            *starGraphs_.at(key).star, Ordering(list_of(neighbor)));
        // store the new factor into messages
        messages.insert(make_pair(neighbor, factor));

        // Belief is the product of all messages and the unary factor
        // Incorporate new the factor to belief
        beliefAtKey = beliefAtKey
            * (*boost::dynamic_pointer_cast<DecisionTreeFactor>(factor));
      }
      beliefs->push_back(beliefAtKey);

      // Update the corrected belief for the neighbor's stargraph
      BOOST_FOREACH(Key neighbor, starGraphs_.at(key).correctedBeliefIndices | boost::adaptors::map_keys) {
        DecisionTreeFactor correctedBelief = beliefAtKey
            / (*boost::dynamic_pointer_cast<DecisionTreeFactor>(
                messages.at(neighbor)));
        size_t beliefIndex = starGraphs_.at(neighbor).correctedBeliefIndices.at(
            key);
        starGraphs_.at(neighbor).star->replace(beliefIndex,
            boost::make_shared<DecisionTreeFactor>(correctedBelief));
      }
    }

    return beliefs;
  }

private:
  /**
   * Build star graphs for each node.
   */
  StarGraphs buildStarGraphs(const DiscreteFactorGraph& graph,
      const std::map<Key, DiscreteKey>& allDiscreteKeys) const {
    StarGraphs starGraphs;
    VariableIndex varIndex(graph); ///< access to all factors of each node
    BOOST_FOREACH(Key key, varIndex | boost::adaptors::map_keys) {
      // initialize to multiply with other unary factors later
      DecisionTreeFactor prodOfUnaries(allDiscreteKeys.at(key), "1 1");

      // collect all factors involving this key in the original graph
      DiscreteFactorGraph::shared_ptr star(new DiscreteFactorGraph());
      BOOST_FOREACH(size_t factorIdx, varIndex[key]) {
        star->push_back(graph.at(factorIdx));

        // accumulate unary factors
        if (graph.at(factorIdx)->size() == 1) {
          prodOfUnaries = prodOfUnaries
              * (*boost::dynamic_pointer_cast<DecisionTreeFactor>(
                  graph.at(factorIdx)));
        }
      }

      // add the belief factor for each neighbor variable to this star graph
      // also record the factor index for later modification
      FastSet<Key> neighbors = star->keys();
      neighbors.erase(key);
      CorrectedBeliefIndices correctedBeliefIndices;
      BOOST_FOREACH(Key neighbor, neighbors) {
        // TODO: default table for keys with more than 2 values?
        star->push_back(
            DecisionTreeFactor(allDiscreteKeys.at(neighbor), "1.0 0.0"));
        correctedBeliefIndices.insert(make_pair(neighbor, star->size() - 1));
      }
      starGraphs.insert(
          make_pair(key,
              StarGraph(star, make_shared<DecisionTreeFactor>(prodOfUnaries),
                  correctedBeliefIndices)));
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
  std::map<Key, DiscreteKey> allKeys = map_list_of(0, C)(1, S)(2, R)(3, W);

  // Build graph
  DecisionTreeFactor pC(C, "0.5 0.5");
  DiscreteConditional pSC(S | C = "0.5/0.5 0.9/0.1");
  DiscreteConditional pRC(R | C = "0.8/0.2 0.2/0.8");
  DiscreteConditional pWSR((W | S, R) = "1.0/0.0 0.1/0.9 0.1/0.9 0.01/0.99");
  DiscreteFactorGraph graph;
  graph.push_back(pC);
  graph.push_back(pSC);
  graph.push_back(pRC);
  graph.push_back(pWSR);

  graph.print("graph: ");

  LoopyBelief solver(graph, allKeys);

  // Main loop
  for (size_t iter = 0; iter < 10; ++iter) {
    DiscreteFactorGraph::shared_ptr beliefs = solver.iterate();
    cout << "iteration: " << iter << endl;
    beliefs->print();
  }

}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
