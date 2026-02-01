/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * DiscreteSearch.cpp
 *
 * @date January, 2025
 * @author Frank Dellaert
 */

#include <gtsam/discrete/DiscreteEliminationTree.h>
#include <gtsam/discrete/DiscreteJunctionTree.h>
#include <gtsam/discrete/DiscreteSearch.h>

namespace gtsam {

using Slot = DiscreteSearch::Slot;
using Solution = DiscreteSearch::Solution;

/*
 * A SearchNode represents a node in the search tree for the search algorithm.
 * Each SearchNode contains a partial assignment of discrete variables, the
 * current error, a bound on the final error, and the index of the next
 * slot to be assigned.
 */
struct SearchNode {
  DiscreteValues assignment;   // Partial assignment of discrete variables.
  double error;                // Current error for the partial assignment.
  double bound;                // Lower bound on the final error
  std::optional<size_t> next;  // Index of the next slot to be assigned.

  // Construct the root node for the search.
  static SearchNode Root(size_t numSlots, double bound) {
    return {DiscreteValues(), 0.0, bound, 0};
  }

  struct Compare {
    bool operator()(const SearchNode& a, const SearchNode& b) const {
      return a.bound > b.bound;  // smallest bound -> highest priority
    }
  };

  // Checks if the node represents a complete assignment.
  inline bool isComplete() const { return !next; }

  // Expands the node by assigning the next variable(s).
  SearchNode expand(const DiscreteValues& fa, const Slot& slot,
                    std::optional<size_t> nextSlot) const {
    // Combine the new frontal assignment with the current partial assignment
    DiscreteValues newAssignment = assignment;
    for (auto& [key, value] : fa) {
      newAssignment[key] = value;
    }
    double errorSoFar = error + slot.factor->error(newAssignment);
    return {newAssignment, errorSoFar, errorSoFar + slot.heuristic, nextSlot};
  }

  // Prints the SearchNode to an output stream.
  friend std::ostream& operator<<(std::ostream& os, const SearchNode& node) {
    os << "SearchNode(error=" << node.error << ", bound=" << node.bound << ")";
    return os;
  }
};

struct CompareSolution {
  bool operator()(const Solution& a, const Solution& b) const {
    return a.error < b.error;
  }
};

/*
 * A Solutions object maintains a priority queue of the best solutions found
 * during the search. The priority queue is limited to a maximum size, and
 * solutions are only added if they are better than the worst solution.
 */
class Solutions {
  size_t maxSize_;  // Maximum number of solutions to keep
  std::priority_queue<Solution, std::vector<Solution>, CompareSolution> pq_;

 public:
  Solutions(size_t maxSize) : maxSize_(maxSize) {}

  // Add a solution to the priority queue, possibly evicting the worst one.
  // Return true if we added the solution.
  bool maybeAdd(double error, const DiscreteValues& assignment) {
    const bool full = pq_.size() == maxSize_;
    if (full && error >= pq_.top().error) return false;
    if (full) pq_.pop();
    pq_.emplace(error, assignment);
    return true;
  }

  // Check if we have any solutions
  bool empty() const { return pq_.empty(); }

  // Method to print all solutions
  friend std::ostream& operator<<(std::ostream& os, const Solutions& sn) {
    os << "Solutions (top " << sn.pq_.size() << "):\n";
    auto pq = sn.pq_;
    while (!pq.empty()) {
      os << pq.top() << "\n";
      pq.pop();
    }
    return os;
  }

  // Check if (partial) solution with given bound can be pruned. If we have
  // room, we never prune. Otherwise, prune if lower bound on error is worse
  // than our current worst error.
  bool prune(double bound) const {
    if (pq_.size() < maxSize_) return false;
    return bound >= pq_.top().error;
  }

  // Method to extract solutions in ascending order of error
  std::vector<Solution> extractSolutions() {
    std::vector<Solution> result;
    while (!pq_.empty()) {
      result.push_back(pq_.top());
      pq_.pop();
    }
    std::sort(
        result.begin(), result.end(),
        [](const Solution& a, const Solution& b) { return a.error < b.error; });
    return result;
  }
};

// Get the factor associated with a node, possibly product of factors.
template <typename NodeType>
static DiscreteFactor::shared_ptr getFactor(const NodeType& node) {
  const auto& factors = node->factors;
  return factors.size() == 1 ? factors.back()
                             : DiscreteFactorGraph(factors).product();
}

DiscreteSearch::DiscreteSearch(const DiscreteEliminationTree& etree) {
  using NodePtr = std::shared_ptr<DiscreteEliminationTree::Node>;
  auto visitor = [this](const NodePtr& node, int data) {
    const DiscreteFactor::shared_ptr factor = getFactor(node);
    const size_t cardinality = factor->cardinality(node->key);
    std::vector<std::pair<Key, size_t>> pairs{{node->key, cardinality}};
    const Slot slot{factor, DiscreteValues::CartesianProduct(pairs), 0.0};
    slots_.emplace_back(std::move(slot));
    return data + 1;
  };

  int data = 0;  // unused
  treeTraversal::DepthFirstForest(etree, data, visitor);
  lowerBound_ = computeHeuristic();
}

DiscreteSearch::DiscreteSearch(const DiscreteJunctionTree& junctionTree) {
  using NodePtr = std::shared_ptr<DiscreteJunctionTree::Cluster>;
  auto visitor = [this](const NodePtr& cluster, int data) {
    const auto factor = getFactor(cluster);
    std::vector<std::pair<Key, size_t>> pairs;
    for (Key key : cluster->orderedFrontalKeys) {
      pairs.emplace_back(key, factor->cardinality(key));
    }
    const Slot slot{factor, DiscreteValues::CartesianProduct(pairs), 0.0};
    slots_.emplace_back(std::move(slot));
    return data + 1;
  };

  int data = 0;  // unused
  treeTraversal::DepthFirstForest(junctionTree, data, visitor);
  lowerBound_ = computeHeuristic();
}

DiscreteSearch DiscreteSearch::FromFactorGraph(
    const DiscreteFactorGraph& factorGraph, const Ordering& ordering,
    bool buildJunctionTree) {
  const DiscreteEliminationTree etree(factorGraph, ordering);
  if (buildJunctionTree) {
    const DiscreteJunctionTree junctionTree(etree);
    return DiscreteSearch(junctionTree);
  } else {
    return DiscreteSearch(etree);
  }
}

DiscreteSearch::DiscreteSearch(const DiscreteBayesNet& bayesNet) {
  slots_.reserve(bayesNet.size());
  for (auto& conditional : bayesNet) {
    const Slot slot{conditional, conditional->frontalAssignments(), 0.0};
    slots_.emplace_back(std::move(slot));
  }
  std::reverse(slots_.begin(), slots_.end());
  lowerBound_ = computeHeuristic();
}

DiscreteSearch::DiscreteSearch(const DiscreteBayesTree& bayesTree) {
  using NodePtr = DiscreteBayesTree::sharedClique;
  auto visitor = [this](const NodePtr& clique, int data) {
    auto conditional = clique->conditional();
    const Slot slot{conditional, conditional->frontalAssignments(), 0.0};
    slots_.emplace_back(std::move(slot));
    return data + 1;
  };

  int data = 0;  // unused
  treeTraversal::DepthFirstForest(bayesTree, data, visitor);
  lowerBound_ = computeHeuristic();
}

void DiscreteSearch::print(const std::string& name,
                           const KeyFormatter& formatter) const {
  std::cout << name << " with " << slots_.size() << " slots:\n";
  for (size_t i = 0; i < slots_.size(); ++i) {
    std::cout << i << ": " << slots_[i] << std::endl;
  }
}

using SearchNodeQueue = std::priority_queue<SearchNode, std::vector<SearchNode>,
                                            SearchNode::Compare>;

std::vector<Solution> DiscreteSearch::run(size_t K) const {
  if (slots_.empty()) {
    return {Solution(0.0, DiscreteValues())};
  }

  Solutions solutions(K);
  SearchNodeQueue expansions;
  expansions.push(SearchNode::Root(slots_.size(), lowerBound_));

  // Perform the search
  while (!expansions.empty()) {
    // Pop the partial assignment with the smallest bound
    SearchNode current = expansions.top();
    expansions.pop();

    // If we already have K solutions, prune if we cannot beat the worst one.
    if (solutions.prune(current.bound)) {
      continue;
    }

    // Check if we have a complete assignment
    if (current.isComplete()) {
      solutions.maybeAdd(current.error, current.assignment);
      continue;
    }

    // Get the next slot to expand
    const auto& slot = slots_[*current.next];
    std::optional<size_t> nextSlot = *current.next + 1;
    if (nextSlot == slots_.size()) nextSlot.reset();
    for (auto& fa : slot.assignments) {
      auto childNode = current.expand(fa, slot, nextSlot);

      // Again, prune if we cannot beat the worst solution
      if (!solutions.prune(childNode.bound)) {
        expansions.emplace(childNode);
      }
    }
  }

  // Extract solutions from bestSolutions in ascending order of error
  return solutions.extractSolutions();
}
/*
 * We have a number of factors, each with a max value, and we want to compute
 * a lower-bound on the cost-to-go for each slot, *not* including this factor.
 * For the last slot[n-1], this is 0.0, as the cost after that is zero.
 * For the second-to-last slot, it is h = -log(max(factor[n-1])), because after
 * we assign slot[n-2] we still need to assign slot[n-1], which will cost *at
 * least* h. We return the estimated lower bound of the cost for *all* slots.
 */
double DiscreteSearch::computeHeuristic() {
  double error = 0.0;
  for (auto it = slots_.rbegin(); it != slots_.rend(); ++it) {
    it->heuristic = error;
    Ordering ordering(it->factor->begin(), it->factor->end());
    auto maxx = it->factor->max(ordering);
    error -= std::log(maxx->evaluate({}));
  }
  return error;
}

}  // namespace gtsam
