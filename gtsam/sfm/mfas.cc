#include "mfas.h"

#include <map>
#include <set>
#include <vector>

using std::map;
using std::pair;
using std::set;
using std::vector;

namespace gtsam {
namespace mfas {

void flipNegEdges(vector<KeyPair> &edges, vector<double> &weights) {
  // now renumber the edges
  for (int i = 0; i < edges.size(); ++i) {
    if (weights[i] < 0.0) {
      Key tmp = edges[i].second;
      edges[i].second = edges[i].first;
      edges[i].first = tmp;
      weights[i] *= -1;
    }
  }
}

void mfasRatio(const std::vector<KeyPair> &edges,
               const std::vector<double> &weights, const KeyVector &nodes,
               FastMap<Key, int> &ordered_positions) {
  // initialize data structures
  FastMap<Key, double> win_deg;
  FastMap<Key, double> wout_deg;
  FastMap<Key, vector<pair<int, double> > > inbrs;
  FastMap<Key, vector<pair<int, double> > > onbrs;

  // stuff data structures
  for (int ii = 0; ii < edges.size(); ++ii) {
    int i = edges[ii].first;
    int j = edges[ii].second;
    double w = weights[ii];

    win_deg[j] += w;
    wout_deg[i] += w;
    inbrs[j].push_back(pair<int, double>(i, w));
    onbrs[i].push_back(pair<int, double>(j, w));
  }

  int ordered_count = 0;
  while (ordered_count < nodes.size()) {
    // choose an unchosen node
    Key choice;
    double max_score = 0.0;
    for (auto node : nodes) {
      if (ordered_positions.find(node) != ordered_positions.end()) {
        // is this a source
        if (win_deg[node] < 1e-8) {
          choice = node;
          break;
        } else {
          double score = (wout_deg[node] + 1) / (win_deg[node] + 1);
          if (score > max_score) {
            max_score = score;
            choice = node;
          }
        }
      }
    }

    // find its inbrs, adjust their wout_deg
    for (auto it = inbrs[choice].begin(); it != inbrs[choice].end(); ++it)
      wout_deg[it->first] -= it->second;
    // find its onbrs, adjust their win_deg
    for (auto it = onbrs[choice].begin(); it != onbrs[choice].end(); ++it)
      win_deg[it->first] -= it->second;

    ordered_positions[choice] = ordered_count++;
  }
}

void brokenWeights(const std::vector<KeyPair> &edges,
                   const std::vector<double> &weight,
                   const FastMap<Key, int> &ordered_positions,
                   FastMap<Key, double> &broken) {
  // find the broken edges
  for (int i = 0; i < edges.size(); ++i) {
    int x0 = ordered_positions.at(edges[i].first);
    int x1 = ordered_positions.at(edges[i].second);
    if ((x1 - x0) * weight[i] < 0)
      broken[i] += weight[i] > 0 ? weight[i] : -weight[i];
  }
}

}  // namespace mfas
}  // namespace gtsam