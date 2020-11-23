/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LinearSolverParams.cpp
 * @brief   Linear Solver Parameters
 * @author  Fan Jiang
 */

#include "LinearSolverParams.h"

namespace gtsam {

/* ************************************************************************* */
void LinearSolverParams::setIterativeParams(
    const boost::shared_ptr<IterativeOptimizationParameters> params) {
  iterativeParams = params;
}


/* ************************************************************************* */
std::string LinearSolverParams::linearSolverTranslator(
    LinearSolverType linearSolverType) const {
  switch (linearSolverType) {
    case MULTIFRONTAL_CHOLESKY:
      return "MULTIFRONTAL_CHOLESKY";
    case MULTIFRONTAL_QR:
      return "MULTIFRONTAL_QR";
    case SEQUENTIAL_CHOLESKY:
      return "SEQUENTIAL_CHOLESKY";
    case SEQUENTIAL_QR:
      return "SEQUENTIAL_QR";
    case Iterative:
      return "ITERATIVE";
    case CHOLMOD:
      return "CHOLMOD";
    case PCG:
      return "PCG";
    case SUBGRAPH:
      return "SUBGRAPH";
    case EIGEN_QR:
      return "EIGEN_QR";
    case EIGEN_CHOLESKY:
      return "EIGEN_CHOLESKY";
    case SUITESPARSE_CHOLESKY:
      return "SUITESPARSE_CHOLESKY";
    case CUSPARSE_CHOLESKY:
      return "CUSPARSE_CHOLESKY";
    default:
      throw std::invalid_argument(
          "Unknown linear solver type in SuccessiveLinearizationOptimizer");
  }
}

/* ************************************************************************* */
LinearSolverParams::LinearSolverType LinearSolverParams::linearSolverTranslator(
    const std::string &linearSolverType) const {
  if (linearSolverType == "MULTIFRONTAL_CHOLESKY")
    return LinearSolverParams::MULTIFRONTAL_CHOLESKY;
  if (linearSolverType == "MULTIFRONTAL_QR")
    return LinearSolverParams::MULTIFRONTAL_QR;
  if (linearSolverType == "SEQUENTIAL_CHOLESKY")
    return LinearSolverParams::SEQUENTIAL_CHOLESKY;
  if (linearSolverType == "SEQUENTIAL_QR")
    return LinearSolverParams::SEQUENTIAL_QR;
  if (linearSolverType == "ITERATIVE")
    return LinearSolverParams::Iterative;
  if (linearSolverType == "CHOLMOD")
    return LinearSolverParams::CHOLMOD;
  if (linearSolverType == "PCG")
    return LinearSolverParams::PCG;
  if (linearSolverType == "SUBGRAPH")
    return LinearSolverParams::SUBGRAPH;
  if (linearSolverType == "EIGEN_CHOLESKY")
    return LinearSolverParams::EIGEN_CHOLESKY;
  if (linearSolverType == "EIGEN_QR")
    return LinearSolverParams::EIGEN_QR;
  if (linearSolverType == "SUITESPARSE_CHOLESKY")
    return LinearSolverParams::SUITESPARSE_CHOLESKY;
  if (linearSolverType == "CUSPARSE_CHOLESKY")
    return LinearSolverParams::CUSPARSE_CHOLESKY;
  throw std::invalid_argument(
      "Unknown linear solver type in SuccessiveLinearizationOptimizer");
}

/* ************************************************************************* */
std::string LinearSolverParams::orderingTypeTranslator(
    Ordering::OrderingType type) const {
  switch (type) {
    case Ordering::METIS:
      return "METIS";
    case Ordering::COLAMD:
      return "COLAMD";
    default:
      if (ordering)
        return "CUSTOM";
      else
        throw std::invalid_argument(
            "Invalid ordering type: You must provide an ordering for a custom ordering type. See setOrdering");
  }
}

/* ************************************************************************* */
Ordering::OrderingType LinearSolverParams::orderingTypeTranslator(
    const std::string &type) const {
  if (type == "METIS")
    return Ordering::METIS;
  if (type == "COLAMD")
    return Ordering::COLAMD;
  throw std::invalid_argument(
      "Invalid ordering type: You must provide an ordering for a custom ordering type. See setOrdering");
}

}
