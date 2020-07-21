#include <gtsam/sfm/MFAS.h>

using namespace gtsam;
using std::pair;
using std::vector;

MFAS::MFAS(const std::shared_ptr<vector<Key>> &nodes,
           const std::shared_ptr<TranslationEdges> &relativeTranslations,
           const Unit3 &projection_direction)
    : nodes_(nodes), relativeTranslations_(relativeTranslations),
      relativeTranslationsForWeights_(std::make_shared<TranslationEdges>()) {
  // iterate over edges and flip all edges that have negative weights,
  // while storing the magnitude of the weights.
  for (auto it = relativeTranslations->begin();
       it != relativeTranslations->end(); it++) {
    KeyPair edge = it->first;
    double weight = it->second.dot(projection_direction);
    if (weight < 0.0) {
      std::swap(edge.first, edge.second);
      weight *= -1;
    }
    positiveEdgeWeights_[edge] = weight;
  }
}

MFAS::MFAS(const std::shared_ptr<std::vector<Key>> &nodes,
           const std::map<KeyPair, double> &edgeWeights) : nodes_(nodes),
                                                           relativeTranslations_(std::make_shared<TranslationEdges>()),
                                                           relativeTranslationsForWeights_(std::make_shared<
                                                               TranslationEdges>()) {
  // similar to the above direction constructor, but here weights are
  // provided as input.
  for (auto it = edgeWeights.begin(); it != edgeWeights.end(); it++) {
    KeyPair edge = it->first;

    // When constructed like this, we do not have access to the relative translations. 
    // So, we store the unswapped edge in the relativeTranslationsForWeights_ map with a default 
    // Unit3 value. This helps retain the original direction of the edge in the returned result
    // of computeOutlierWeights
    relativeTranslationsForWeights_->insert({edge, Unit3()});

    double weight = it->second;
    if (weight < 0.0) {
      // change the direction of the edge to make weight positive
      std::swap(edge.first, edge.second);
      weight *= -1;
    }
    positiveEdgeWeights_[edge] = weight;
  }
}

std::vector<Key> MFAS::computeOrdering() {
  FastMap<Key, double> in_weights;    // sum on weights of incoming edges for a node
  FastMap<Key, double> out_weights;   // sum on weights of outgoing edges for a node
  FastMap<Key, vector<Key> > in_neighbors;
  FastMap<Key, vector<Key> > out_neighbors;

  // populate neighbors and weights
  for (auto it = positiveEdgeWeights_.begin(); it != positiveEdgeWeights_.end(); it++) {
    const KeyPair &edge = it->first;
    in_weights[edge.second] += it->second;
    out_weights[edge.first] += it->second;
    in_neighbors[edge.second].push_back(edge.first);
    out_neighbors[edge.first].push_back(edge.second);
  }

  // in each iteration, one node is appended to the ordered list
  while (orderedNodes_.size() < nodes_->size()) {

    // finding the node with the max heuristic score
    Key choice;
    double max_score = 0.0;

    for (const Key &node : *nodes_) {
      if (orderedPositions_.find(node) == orderedPositions_.end()) {
        // is this a source
        if (in_weights[node] < 1e-8) {
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
    // find its inbrs, adjust their wout_deg
    for (auto it = in_neighbors[choice].begin();
         it != in_neighbors[choice].end(); ++it)
      out_weights[*it] -= positiveEdgeWeights_[KeyPair(*it, choice)];
    // find its onbrs, adjust their win_deg
    for (auto it = out_neighbors[choice].begin();
         it != out_neighbors[choice].end(); ++it)
      in_weights[*it] -= positiveEdgeWeights_[KeyPair(choice, *it)];

    orderedPositions_[choice] = orderedNodes_.size();
    orderedNodes_.push_back(choice);
  }
  return orderedNodes_;
}

std::map<KeyPair, double> MFAS::computeOutlierWeights() {
  // if ordering has not been computed yet
  if (orderedNodes_.size() != nodes_->size()) {
    computeOrdering();
  }
  // iterate over all edges
  // start and end iterators depend on whether we are using relativeTranslations_ or
  // relativeTranslationsForWeights_ to store the original edge directions
  TranslationEdges::iterator start, end;
  if (relativeTranslationsForWeights_->size() == 0) {
    start = relativeTranslations_->begin();
    end = relativeTranslations_->end();
  } else {
    start = relativeTranslationsForWeights_->begin();
    end = relativeTranslationsForWeights_->end();
  }

  for (auto it = start; it != end; it++) {
    // relativeTranslations may have negative weight edges, we make sure all edges
    // are along the positive direction by flipping them if they are not.
    KeyPair edge = it->first;
    if (positiveEdgeWeights_.find(edge) == positiveEdgeWeights_.end()) {
      std::swap(edge.first, edge.second);
    }

    // if the ordered position of nodes is not consistent with the edge
    // direction for consistency second should be greater than first
    if (orderedPositions_.at(edge.second) < orderedPositions_.at(edge.first)) {
      outlierWeights_[it->first] = std::abs(positiveEdgeWeights_[edge]);
    } else {
      outlierWeights_[it->first] = 0;
    }
  }
  return outlierWeights_;
}