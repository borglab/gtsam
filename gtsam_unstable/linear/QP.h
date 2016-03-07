/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    QP.h
 * @brief   Factor graphs of a Quadratic Programming problem
 * @date    Dec 8, 2014
 * @author  Duy-Nguyen Ta
 */

#pragma once

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam_unstable/linear/EqualityFactorGraph.h>
#include <gtsam_unstable/linear/InequalityFactorGraph.h>
#include <gtsam/slam/dataset.h>

namespace gtsam {

/**
 * Struct contains factor graphs of a Quadratic Programming problem
 */
struct QP {
  GaussianFactorGraph cost; //!< Quadratic cost factors
  EqualityFactorGraph equalities; //!< linear equality constraints: cE(x) = 0
  InequalityFactorGraph inequalities; //!< linear inequality constraints: cI(x) <= 0

  /** default constructor */
  QP() :
      cost(), equalities(), inequalities() {
  }

  /** constructor */
  QP(const GaussianFactorGraph& _cost,
      const EqualityFactorGraph& _linearEqualities,
      const InequalityFactorGraph& _linearInequalities) :
      cost(_cost), equalities(_linearEqualities), inequalities(_linearInequalities) {
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

