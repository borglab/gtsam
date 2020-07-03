#include "mfas.h"

#include <map>
#include <set>
#include <vector>

using std::map;
using std::pair;
using std::set;
using std::vector;

void reindex_problem(vector<Edge> &edges, map<int, int> &reindexing_key) {
  // get the unique set of notes
  set<int> nodes;
  for (int i = 0; i < edges.size(); ++i) {
    nodes.insert(edges[i].first);
    nodes.insert(edges[i].second);
  }

  // iterator through them and assign a new name to each vertex
  std::set<int>::const_iterator it;
  reindexing_key.clear();
  int i = 0;
  for (it = nodes.begin(); it != nodes.end(); ++it) {
    reindexing_key[*it] = i;
    ++i;
  }

  // now renumber the edges
  for (int i = 0; i < edges.size(); ++i) {
    edges[i].first = reindexing_key[edges[i].first];
    edges[i].second = reindexing_key[edges[i].second];
  }
}

void flip_neg_edges(vector<Edge> &edges, vector<double> &weights) {
  // now renumber the edges
  for (int i = 0; i < edges.size(); ++i) {
    if (weights[i] < 0.0) {
      double tmp = edges[i].second;
      edges[i].second = edges[i].first;
      edges[i].first = tmp;
      weights[i] *= -1;
    }
  }
}

void mfas_ratio(const std::vector<Edge> &edges,
                const std::vector<double> &weights, std::vector<int> &order) {
  // find the number of nodes in this problem
  int n = -1;
  int m = edges.size();
  for (int i = 0; i < m; ++i) {
    n = (edges[i].first > n) ? edges[i].first : n;
    n = (edges[i].second > n) ? edges[i].second : n;
  }
  n += 1;  // 0 indexed

  // initialize data structures
  vector<double> win_deg(n, 0.0);
  vector<double> wout_deg(n, 0.0);
  vector<bool> unchosen(n, 1);
  vector<vector<pair<int, double> > > inbrs(n);
  vector<vector<pair<int, double> > > onbrs(n);

  // stuff data structures
  for (int ii = 0; ii < m; ++ii) {
    int i = edges[ii].first;
    int j = edges[ii].second;
    double w = weights[ii];

    win_deg[j] += w;
    wout_deg[i] += w;
    inbrs[j].push_back(pair<int, double>(i, w));
    onbrs[i].push_back(pair<int, double>(j, w));
  }

  while (order.size() < n) {
    // choose an unchosen node
    int choice = -1;
    double max_score = 0.0;
    for (int i = 0; i < n; ++i) {
      if (unchosen[i]) {
        // is this a source
        if (win_deg[i] < 1e-8) {
          choice = i;
          break;
        } else {
          double score = (wout_deg[i] + 1) / (win_deg[i] + 1);
          if (score > max_score) {
            max_score = score;
            choice = i;
          }
        }
      }
    }

    // find its inbrs, adjust their wout_deg
    vector<pair<int, double> >::iterator it;
    for (it = inbrs[choice].begin(); it != inbrs[choice].end(); ++it)
      wout_deg[it->first] -= it->second;
    // find its onbrs, adjust their win_deg
    for (it = onbrs[choice].begin(); it != onbrs[choice].end(); ++it)
      win_deg[it->first] -= it->second;

    order.push_back(choice);
    unchosen[choice] = 0;
  }
}

void broken_weight(const std::vector<Edge> &edges,
                   const std::vector<double> &weight,
                   const std::vector<int> &order, std::vector<double> &broken) {
  // clear the output vector
  int m = edges.size();
  broken.resize(m);
  broken.assign(broken.size(), 0.0);

  // find the number of nodes in this problem
  int n = -1;
  for (int i = 0; i < m; ++i) {
    n = (edges[i].first > n) ? edges[i].first : n;
    n = (edges[i].second > n) ? edges[i].second : n;
  }
  n += 1;  // 0 indexed

  // invert the permutation
  std::vector<int> inv_perm(n, 0.0);
  for (int i = 0; i < n; ++i) inv_perm[order[i]] = i;

  // find the broken edges
  for (int i = 0; i < m; ++i) {
    int x0 = inv_perm[edges[i].first];
    int x1 = inv_perm[edges[i].second];
    if ((x1 - x0) * weight[i] < 0)
      broken[i] += weight[i] > 0 ? weight[i] : -weight[i];
  }
}
