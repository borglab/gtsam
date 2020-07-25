/**
 *  @file  MFAS.cpp
 *  @brief Source file for the MFAS class
 *  @author Akshay Krishnan
 *  @date July 2020
 */

#include <gtsam/sfm/MFAS.h>

using namespace gtsam;
using std::pair;
using std::vector;

MFAS::MFAS(const std::shared_ptr<vector<Key>> &nodes,
           const TranslationEdges& relativeTranslations,
           const Unit3 &projection_direction)
    : nodes_(nodes) {
  // iterate over edges, obtain weights by projecting
  // their relativeTranslations along the projection direction
  for (auto it = relativeTranslations.begin();
       it != relativeTranslations.end(); it++) {
    edgeWeights_[it->first] = it->second.dot(projection_direction);
  }
}

std::vector<Key> MFAS::computeOrdering() const {
  FastMap<Key, double> in_weights;    // sum on weights of incoming edges for a node
  FastMap<Key, double> out_weights;   // sum on weights of outgoing edges for a node
  FastMap<Key, vector<Key> > in_neighbors;
  FastMap<Key, vector<Key> > out_neighbors;

  vector<Key> ordered_nodes;            // nodes in MFAS order (result)
  FastMap<Key, int> ordered_positions;  // map from node to its position in the output order

  // populate neighbors and weights
  // Since the weights could be obtained by projection, they can be either 
  // negative or positive. Ideally, the weights should be positive in the 
  // direction of the edge. So, we define the direction of the edge as 
  // edge.first -> edge.second if weight is positive and 
  // edge.second -> edge.first if weight is negative. Once we know the
  // direction, we only use the magnitude of the weights. 
  for (auto it = edgeWeights_.begin(); it != edgeWeights_.end(); it++) {
    const KeyPair &edge = it->first;
    const double weight = it->second;
    Key edge_source = weight >= 0 ? edge.first : edge.second;
    Key edge_dest = weight >= 0 ? edge.second : edge.first;

    in_weights[edge_dest] += std::abs(weight);
    out_weights[edge_source] += std::abs(weight);
    in_neighbors[edge_dest].push_back(edge_source);
    out_neighbors[edge_source].push_back(edge_dest);
  }

  // in each iteration, one node is appended to the ordered list
  while (ordered_nodes.size() < nodes_->size()) {

    // finding the node with the max heuristic score
    Key choice;
    double max_score = 0.0;

    for (const Key &node : *nodes_) {
      // if this node has not been chosen so far
      if (ordered_positions.find(node) == ordered_positions.end()) {
        // is this a root node
        if (in_weights[node] < 1e-8) {
          // TODO(akshay-krishnan) if there are multiple roots, it is better to choose the 
          // one with highest heuristic. This is missing in the 1dsfm solution. 
          choice = node;
          break;
        } else {
          double score = (out_weights[node] + 1) / (in_weights[node] + 1);
          if (score > max_score) {
            max_score = score;
            choice = node;
          }
        }
      }
    }
    // find its in_neighbors, adjust their out_weights
    for (auto it = in_neighbors[choice].begin();
         it != in_neighbors[choice].end(); ++it)
      // the edge could be either (*it, choice) with a positive weight or (choice, *it) with a negative weight
      out_weights[*it] -= edgeWeights_.find(KeyPair(*it, choice)) == edgeWeights_.end() ? -edgeWeights_.at(KeyPair(choice, *it)) : edgeWeights_.at(KeyPair(*it, choice));

    // find its out_neighbors, adjust their in_weights
    for (auto it = out_neighbors[choice].begin();
         it != out_neighbors[choice].end(); ++it)
      in_weights[*it] -= edgeWeights_.find(KeyPair(choice, *it)) == edgeWeights_.end() ? -edgeWeights_.at(KeyPair(*it, choice)) : edgeWeights_.at(KeyPair(choice, *it));

    ordered_positions[choice] = ordered_nodes.size();
    ordered_nodes.push_back(choice);
  }
  return ordered_nodes;
}

std::map<MFAS::KeyPair, double> MFAS::computeOutlierWeights() const {
  vector<Key> ordered_nodes = computeOrdering();
  FastMap<Key, int> ordered_positions;
  std::map<KeyPair, double> outlier_weights;

  // create a map becuase it is much faster to lookup the position of each node
  // TODO(akshay-krishnan) this is already computed in computeOrdering. Would be nice if 
  // we could re-use. Either use an optional argument or change the output of
  // computeOrdering
  for(unsigned int i = 0; i < ordered_nodes.size(); i++) {
    ordered_positions[ordered_nodes[i]] = i;
  }

  // iterate over all edges
  for (auto it = edgeWeights_.begin(); it != edgeWeights_.end(); it++) {
    Key edge_source, edge_dest;
    if(it->second > 0) {
      edge_source = it->first.first;
      edge_dest = it->first.second;
    } else {
      edge_source = it->first.second;
      edge_dest = it->first.first;
    }

    // if the ordered position of nodes is not consistent with the edge
    // direction for consistency second should be greater than first
    if (ordered_positions.at(edge_dest) < ordered_positions.at(edge_source)) {
      outlier_weights[it->first] = std::abs(edgeWeights_.at(it->first));
    } else {
      outlier_weights[it->first] = 0;
    }
  }
  return outlier_weights;
}