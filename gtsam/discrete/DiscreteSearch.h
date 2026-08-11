/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DiscreteSearch.h
 * @brief Defines the DiscreteSearch class for discrete search algorithms.
 *
 * @details This file contains the definition of the DiscreteSearch class, which
 * is used in discrete search algorithms to find the K best solutions.
 *
 * @date January, 2025
 * @author Frank Dellaert
 */

#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/discrete/DiscreteBayesTree.h>

#include <queue>

namespace gtsam {

/**
 * @brief DiscreteSearch: Search for the K best solutions.
 *
 * This class is used to search for the K best solutions in a DiscreteBayesNet.
 * This is implemented with a modified A* search algorithm that uses a priority
 * queue to manage the search nodes. That machinery is defined in the .cpp file.
 * The heuristic we use is the sum of the log-probabilities of the
 * maximum-probability assignments for each slot, for all slots to the right of
 * the current slot.
 *
 * TODO: The heuristic could be refined by using the partial assignment in
 * search node to refine the max-probability assignment for the remaining slots.
 * This would incur more computation but will lead to fewer expansions.
 */
class GTSAM_EXPORT DiscreteSearch {
 public:
  /**
   * We structure the search as a set of slots, each with a factor and
   * a set of variable assignments that need to be chosen. In addition, each
   * slot has a heuristic associated with it.
   *
   * Example:
   * The factors in the search problem (always parents before descendents!):
   *    [P(A), P(B|A), P(C|A,B)]
   * The assignments for each factor.
   *    [[A0,A1], [B0,B1], [C0,C1,C2]]
   * A lower bound on the cost-to-go after each slot, e.g.,
   *    [-log(max_B P(B|A)) -log(max_C P(C|A,B)), -log(max_C P(C|A,B)), 0.0]
   * Note that these decrease as we move from right to left.
   * We keep the global lower bound as lowerBound_. In the example, it is:
   *    -log(max_B P(B|A)) -log(max_C P(C|A,B)) -log(max_C P(C|A,B))
   */
  struct Slot {
    DiscreteFactor::shared_ptr factor;
    std::vector<DiscreteValues> assignments;
    double heuristic;

    friend std::ostream& operator<<(std::ostream& os, const Slot& slot) {
      os << "Slot with " << slot.assignments.size()
         << " assignments, heuristic=" << slot.heuristic;
      os << ", factor:\n" << slot.factor->markdown() << std::endl;
      return os;
    }
  };

  /**
   * A solution is a set of assignments, covering all the slots.
   * as well as an associated error = -log(probability)
   */
  struct Solution {
    double error;
    DiscreteValues assignment;
    Solution(double err, const DiscreteValues& assign)
        : error(err), assignment(assign) {}
    friend std::ostream& operator<<(std::ostream& os, const Solution& sn) {
      os << "[ error=" << sn.error << " assignment={" << sn.assignment << "}]";
      return os;
    }
  };

 public:
  /// @name Standard Constructors
  /// @{

  /**
   * Construct from a DiscreteFactorGraph.
   *
   * Internally creates either an elimination tree or a junction tree. The
   * latter incurs more up-front computation but the search itself might be
   * faster. Then again, for the elimination tree, the heuristic will be more
   * fine-grained (more slots).
   *
   * @param factorGraph The factor graph to search over.
   * @param ordering The ordering used to create etree (and maybe jtree).
   * @param buildJunctionTree Whether to build a junction tree or not.
   */
  static DiscreteSearch FromFactorGraph(const DiscreteFactorGraph& factorGraph,
                                        const Ordering& ordering,
                                        bool buildJunctionTree = false);

  /// Construct from a DiscreteEliminationTree.
  DiscreteSearch(const DiscreteEliminationTree& etree);

  /// Construct from a DiscreteJunctionTree.
  DiscreteSearch(const DiscreteJunctionTree& junctionTree);

  /// Construct from a DiscreteBayesNet.
  DiscreteSearch(const DiscreteBayesNet& bayesNet);

  /// Construct from a DiscreteBayesTree.
  DiscreteSearch(const DiscreteBayesTree& bayesTree);

  /// @}
  /// @name Testable
  /// @{

  /** Print the tree to cout */
  void print(const std::string& name = "DiscreteSearch: ",
             const KeyFormatter& formatter = DefaultKeyFormatter) const;

  /// @}
  /// @name Standard API
  /// @{

  /// Return lower bound on the cost-to-go for the entire search
  double lowerBound() const { return lowerBound_; }

  /// Read access to the slots
  const std::vector<Slot>& slots() const { return slots_; }

  /**
   * @brief Search for the K best solutions.
   *
   * This method performs a search to find the K best solutions for the given
   * DiscreteBayesNet. It uses a priority queue to manage the search nodes,
   * expanding nodes with the smallest bound first. The search continues until
   * all possible nodes have been expanded or pruned.
   *
   * @return A vector of the K best solutions found during the search.
   */
  std::vector<Solution> run(size_t K = 1) const;

  /// @}

 private:
  /**
   * Compute the cumulative lower-bound cost-to-go after each slot is filled.
   * @return the estimated lower bound of the cost for *all* slots.
   */
  double computeHeuristic();

  double lowerBound_;  ///< Lower bound on the cost-to-go for the entire search.
  std::vector<Slot> slots_;  ///< The slots to fill in the search.
};

using DiscreteSearchSolution = DiscreteSearch::Solution;  // for wrapping
}  // namespace gtsam
