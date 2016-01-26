/**
 * @file     LP.h
 * @brief    Struct used to hold a Linear Programming Problem
 * @author   Ivan Dario Jimenez
 * @date     1/24/16
 */

#pragma once

#include <gtsam_unstable/linear/LinearCost.h>
#include <gtsam_unstable/linear/EqualityFactorGraph.h>

#include <string>

namespace gtsam {

using namespace std;

struct LP {
  LinearCost cost; //!< Linear cost factor
  EqualityFactorGraph equalities; //!< Linear equality constraints: cE(x) = 0
  InequalityFactorGraph inequalities; //!< Linear inequality constraints: cI(x) <= 0

  /// check feasibility
  bool isFeasible(const VectorValues& x) const {
    return (equalities.error(x) == 0 && inequalities.error(x) == 0);
  }

  /// print
  void print(const string& s = "") const {
    std::cout << s << std::endl;
    cost.print("Linear cost: ");
    equalities.print("Linear equality factors: ");
    inequalities.print("Linear inequality factors: ");
  }

  /// equals
  bool equals(const LP& other, double tol = 1e-9) const {
    return cost.equals(other.cost) && equalities.equals(other.equalities)
        && inequalities.equals(other.inequalities);
  }

  typedef boost::shared_ptr<LP> shared_ptr;
};

/// traits
template<> struct traits<LP> : public Testable<LP> {
};
}
