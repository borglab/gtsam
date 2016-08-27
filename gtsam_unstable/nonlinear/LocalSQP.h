/*
 * SQPLineSearch2.h
 * @brief:
 * @date: Aug 26, 2016
 * @author: Ivan Dario Jimenez
 */

#pragma once

#include <gtsam_unstable/nonlinear/NP.h>

namespace gtsam {

class LocalSQP {
  NP program_;
public:
  struct State {
    unsigned int k;
    Values x_star;
    VectorValues lambda_star;
    bool converged;
    State(
      const Values& x = Values(),
      const VectorValues& lambda = VectorValues(),
      const unsigned int k0 = 1,
      const bool converged0 = false    ) :
        k(k0), x_star(x), lambda_star(lambda), converged(converged0) {
    }
    void print(const std::string& s = "State") const {
      std::cout << s << ": " << std::endl;
      x_star.print("\tSolution: ");
      lambda_star.print("\tLambdas: ");
      std::cout << "k: " << k << std::endl;
      std::cout << "\tConverged: " << converged << std::endl;
    }
  };

  LocalSQP(const NP& program) :
      program_(program) {
  }

  State iterate(const State& currentState) const;

  Values optimize(const Values& initials, unsigned int max_iter = 100) const;

};

}

