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
using namespace boost::assign;
using namespace gtsam;

class LoopyBelief {

  typedef std::map<Key, size_t> CorrectedBeliefIndices;

  struct StarGraph {
    DiscreteFactorGraph::shared_ptr star;
    CorrectedBeliefIndices correctedBeliefIndices;
    StarGraph() {
    }
    StarGraph(const DiscreteFactorGraph::shared_ptr& _star,
        const CorrectedBeliefIndices& _beliefIndices) :
        star(_star), correctedBeliefIndices(_beliefIndices) {
    }
  };

  const DiscreteFactorGraph& graph_;
  VariableIndex varIndex_;
  std::map<Key, StarGraph> starGraphs_;
  std::map<Key, DecisionTreeFactor> unary_;
  std::map<Key, DecisionTreeFactor> belief_;

public:
  LoopyBelief(const DiscreteFactorGraph& graph, const std::map<Key, DiscreteKey>& allDiscreteKeys) :
      graph_(graph), varIndex_(graph) {
    initialize(allDiscreteKeys);
  }

  void iterate() {
    static DiscreteConditional::shared_ptr dummyCond; // unused by-product of elimination

    // Eliminate each star graph
    BOOST_FOREACH(const VariableIndex::value_type& keyFactors, varIndex_) {
      Key key = keyFactors.first;
      // reset belief to the unary factor in the original graph
      belief_[key] = unary_.at(key);

      // keep intermediate messages to divide later
      std::map<Key, DiscreteFactor::shared_ptr> messages;

      // eliminate each neighbor in this star graph one by one
      BOOST_FOREACH(Key neighbor, starGraphs_.at(key).correctedBeliefIndices | boost::adaptors::map_keys) {
        DiscreteFactor::shared_ptr factor;
        boost::tie(dummyCond, factor) = EliminateDiscrete(*starGraphs_.at(key).star,
            Ordering(list_of(neighbor)));
        // store the new factor into messages
        messages.insert(make_pair(neighbor, factor));

        // Belief is the product of all messages and the unary factor
        // Incorporate new the factor to belief
        belief_.at(key) = belief_.at(key) * (*boost::dynamic_pointer_cast<DecisionTreeFactor>(factor));
      }

      // Update the corrected belief for the neighbor's stargraph
      BOOST_FOREACH(Key neighbor, starGraphs_.at(key).correctedBeliefIndices | boost::adaptors::map_keys) {
        DecisionTreeFactor correctedBelief = belief_.at(key)
            / (*boost::dynamic_pointer_cast<DecisionTreeFactor>(messages.at(neighbor)));
        size_t beliefIndex = starGraphs_.at(neighbor).correctedBeliefIndices.at(
            key);
        starGraphs_.at(neighbor).star->replace(beliefIndex, boost::make_shared<DecisionTreeFactor>(correctedBelief));
      }
    }
  }

private:
  void initialize(const std::map<Key, DiscreteKey>& allDiscreteKeys) {
    BOOST_FOREACH(Key key, varIndex_ | boost::adaptors::map_keys) {
      // initialize to multiply with other unary factors later
      unary_.insert(make_pair(key, DecisionTreeFactor(allDiscreteKeys.at(key), "1 1")));

      // collect all factors involving this key in the original graph
      DiscreteFactorGraph::shared_ptr star(new DiscreteFactorGraph());
      BOOST_FOREACH(size_t factorIdx, varIndex_[key]) {
        star->push_back(graph_.at(factorIdx));
        if (graph_.at(factorIdx)->size() == 1) {
          unary_.at(key) = unary_.at(key)
              * (*boost::dynamic_pointer_cast<DecisionTreeFactor>(
                  graph_.at(factorIdx)));
        }
      }

      // add the belief factor for each other variable in this star graph
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
      starGraphs_.insert(
          make_pair(key, StarGraph(star, correctedBeliefIndices)));
    }
  }
};

/* ************************************************************************* */

TEST_UNSAFE(LoopyBelief, construction) {
  // Variables: Cloudy, Sprinkler, Rain, Wet
  DiscreteKey C(0, 2), S(1, 2), R(2, 2), W(3, 2);

  // Map from key to DiscreteKey for building belief factor.
  // TODO: this is bad!
  std::map<Key, DiscreteKey> allKeys = map_list_of(0,C)(1,S)(2,R)(3,W);

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
    solver.iterate();
  }

}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
