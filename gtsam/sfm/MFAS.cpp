/**
 *  @file  MFAS.cpp
 *  @brief Source file for the MFAS class
 *  @author Akshay Krishnan
 *  @date July 2020
 */

#include <gtsam/sfm/MFAS.h>

#include <algorithm>
#include <map>
#include <unordered_map>
#include <vector>
#include <unordered_set>

using namespace gtsam;
using std::map;
using std::pair;
using std::unordered_map;
using std::vector;
using std::unordered_set;

// A node in the graph.
struct GraphNode {
  double inWeightSum;               // Sum of absolute weights of incoming edges
  double outWeightSum;              // Sum of absolute weights of outgoing edges
  unordered_set<Key> inNeighbors;   // Nodes from which there is an incoming edge
  unordered_set<Key> outNeighbors;  // Nodes to which there is an outgoing edge

  // Heuristic for the node that is to select nodes in MFAS.
  double heuristic() const { return (outWeightSum + 1) / (inWeightSum + 1); }
};

// A graph is a map from key to GraphNode. This function returns the graph from
// the edgeWeights between keys.
unordered_map<Key, GraphNode> graphFromEdges(
    const map<MFAS::KeyPair, double>& edgeWeights) {
  unordered_map<Key, GraphNode> graph;

  for (const auto& edgeWeight : edgeWeights) {
    // The weights can be either negative or positive. The direction of the edge
    // is the direction of positive weight. This means that the edges is from
    // edge.first -> edge.second if weight is positive and edge.second ->
    // edge.first if weight is negative.
    const MFAS::KeyPair& edge = edgeWeight.first;
    const double& weight = edgeWeight.second;

    Key edgeSource = weight >= 0 ? edge.first : edge.second;
    Key edgeDest = weight >= 0 ? edge.second : edge.first;

    // Update the in weight and neighbors for the destination.
    graph[edgeDest].inWeightSum += std::abs(weight);
    graph[edgeDest].inNeighbors.insert(edgeSource);

    // Update the out weight and neighbors for the source.
    graph[edgeSource].outWeightSum += std::abs(weight);
    graph[edgeSource].outNeighbors.insert(edgeDest);
  }
  return graph;
}

// Selects the next node in the ordering from the graph.
Key selectNextNodeInOrdering(const unordered_map<Key, GraphNode>& graph) {
  // Find the root nodes in the graph.
  for (const auto& keyNode : graph) {
    // It is a root node if the inWeightSum is close to zero.
    if (keyNode.second.inWeightSum < 1e-8) {
      // TODO(akshay-krishnan) if there are multiple roots, it is better to
      // choose the one with highest heuristic. This is missing in the 1dsfm
      // solution.
      return keyNode.first;
    }
  }
  // If there are no root nodes, return the node with the highest heuristic.
  return std::max_element(graph.begin(), graph.end(),
                          [](const std::pair<Key, GraphNode>& keyNode1,
                             const std::pair<Key, GraphNode>& keyNode2) {
                            return keyNode1.second.heuristic() <
                                   keyNode2.second.heuristic();
                          })
      ->first;
}

// Returns the absolute weight of the edge between node1 and node2.
double absWeightOfEdge(const Key key1, const Key key2,
                       const map<MFAS::KeyPair, double>& edgeWeights) {
  // Check the direction of the edge before returning.
  return edgeWeights.find(MFAS::KeyPair(key1, key2)) != edgeWeights.end()
             ? std::abs(edgeWeights.at(MFAS::KeyPair(key1, key2)))
             : std::abs(edgeWeights.at(MFAS::KeyPair(key2, key1)));
}

// Removes a node from the graph and updates edge weights of its neighbors.
void removeNodeFromGraph(const Key node,
                         const map<MFAS::KeyPair, double> edgeWeights,
                         unordered_map<Key, GraphNode>& graph) {
  // Update the outweights and outNeighbors of node's inNeighbors
  for (const Key neighbor : graph[node].inNeighbors) {
    // the edge could be either (*it, choice) with a positive weight or
    // (choice, *it) with a negative weight
    graph[neighbor].outWeightSum -=
        absWeightOfEdge(node, neighbor, edgeWeights);
    graph[neighbor].outNeighbors.erase(node);
  }
  // Update the inWeights and inNeighbors of node's outNeighbors
  for (const Key neighbor : graph[node].outNeighbors) {
    graph[neighbor].inWeightSum -= absWeightOfEdge(node, neighbor, edgeWeights);
    graph[neighbor].inNeighbors.erase(node);
  }
  // Erase node.
  graph.erase(node);
}

MFAS::MFAS(const TranslationEdges& relativeTranslations,
           const Unit3& projectionDirection) {
  // Iterate over edges, obtain weights by projecting
  // their relativeTranslations along the projection direction
  for (const auto& measurement : relativeTranslations) {
    edgeWeights_[std::make_pair(measurement.key1(), measurement.key2())] =
        measurement.measured().dot(projectionDirection);
  }
}

KeyVector MFAS::computeOrdering() const {
  KeyVector ordering;  // Nodes in MFAS order (result).

  // A graph is an unordered map from keys to nodes. Each node contains a list
  // of its adjacent nodes. Create the graph from the edgeWeights.
  unordered_map<Key, GraphNode> graph = graphFromEdges(edgeWeights_);

  // In each iteration, one node is removed from the graph and appended to the
  // ordering.
  while (!graph.empty()) {
    Key selection = selectNextNodeInOrdering(graph);
    removeNodeFromGraph(selection, edgeWeights_, graph);
    ordering.push_back(selection);
  }
  return ordering;
}

map<MFAS::KeyPair, double> MFAS::computeOutlierWeights() const {
  // Find the ordering.
  KeyVector ordering = computeOrdering();

  // Create a map from the node key to its position in the ordering. This makes
  // it easier to lookup positions of different nodes.
  unordered_map<Key, int> orderingPositions;
  for (size_t i = 0; i < ordering.size(); i++) {
    orderingPositions[ordering[i]] = i;
  }

  map<KeyPair, double> outlierWeights;
  // Check if the direction of each edge is consistent with the ordering.
  for (const auto& edgeWeight : edgeWeights_) {
    // Find edge source and destination.
    Key source = edgeWeight.first.first;
    Key dest = edgeWeight.first.second;
    if (edgeWeight.second < 0) {
      std::swap(source, dest);
    }

    // If the direction is not consistent with the ordering (i.e dest occurs
    // before src), it is an outlier edge, and has non-zero outlier weight.
    if (orderingPositions.at(dest) < orderingPositions.at(source)) {
      outlierWeights[edgeWeight.first] = std::abs(edgeWeight.second);
    } else {
      outlierWeights[edgeWeight.first] = 0;
    }
  }
  return outlierWeights;
}
