/**
 * @file    NP.h
 * @brief   Factor graphs representing a non-linear Programming problem
 * @date    Jul 27, 2016
 * @author  Ivan Dario Jimenez
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_unstable/nonlinear/NonlinearInequalityFactorGraph.h>
#include <gtsam_unstable/nonlinear/NonlinearEqualityFactorGraph.h>

namespace gtsam {
/**
 * Struct contains factor graphs for a Nonlinear Programming problem.
 */
struct NP {
  NonlinearFactorGraph cost; //!< Nonlinear cost factors
  NonlinearEqualityFactorGraph equalities; //!< Nonlinear equality constraints cE(X) = 0
  NonlinearInequalityFactorGraph inequalities; //!< Nonlinear inequality constriants cE(X) <= 0

  /** Default Constructor  */
  NP() :
      cost(), equalities(), inequalities() {
  }

  /** Constructor */
  NP(const NonlinearFactorGraph & _cost,
      const NonlinearEqualityFactorGraph & _equalities,
      const NonlinearInequalityFactorGraph & _inequalities) :
      cost(_cost), equalities(_equalities), inequalities(_inequalities) {
  }

  /** print */
  void print(const std::string & s = "") {
    std::cout << s << std::endl;
    cost.print("Nonlinear Cost Factors: ");
    equalities.print("Nonlinear Equality Cost Factors: ");
    inequalities.print("Nonlinear Inequality Cost Factors: ");
  }
};

} // namespace gtsam
