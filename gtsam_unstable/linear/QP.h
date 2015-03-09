/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * QP.h
 * @brief: Factor graphs of a Quadratic Programming problem
 * @date: Dec 8, 2014
 * @author: thduynguyen
 */

#pragma once

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam_unstable/linear/LinearEqualityFactorGraph.h>
#include <gtsam_unstable/linear/LinearInequalityFactorGraph.h>

namespace gtsam {

/**
 * struct contains factor graphs of a Quadratic Programming problem
 */
struct QP {
  GaussianFactorGraph cost; //!< Quadratic cost factors
  LinearEqualityFactorGraph equalities; //!< linear equality constraints
  LinearInequalityFactorGraph inequalities; //!< linear inequality constraints

  /** default constructor */
  QP() :
      cost(), equalities(), inequalities() {
  }

  /** constructor */
  QP(const GaussianFactorGraph& _cost,
      const LinearEqualityFactorGraph& _linearEqualities,
      const LinearInequalityFactorGraph& _linearInequalities) :
      cost(_cost), equalities(_linearEqualities), inequalities(
          _linearInequalities) {
  }

  /** print */
  void print(const std::string& s = "") {
    std::cout << s << std::endl;
    cost.print("Quadratic cost factors: ");
    equalities.print("Linear equality factors: ");
    inequalities.print("Linear inequality factors: ");
  }
};

} // namespace gtsam

