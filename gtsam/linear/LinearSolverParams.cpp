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
std::string LinearSolverParams::LinearSolverTranslator(
    LinearSolverType linearSolverType) {
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
    case SUITESPARSE:
      return "SUITESPARSE";
    case CUSPARSE:
      return "CUSPARSE";
    default:
      throw std::invalid_argument(
          "Unknown linear solver type in SuccessiveLinearizationOptimizer");
  }
}

/* ************************************************************************* */
LinearSolverParams::LinearSolverType LinearSolverParams::LinearSolverTranslator(
    const std::string &linearSolverType) {
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
  if (linearSolverType == "SUITESPARSE")
    return LinearSolverParams::SUITESPARSE;
  if (linearSolverType == "CUSPARSE")
    return LinearSolverParams::CUSPARSE;
  throw std::invalid_argument(
      "Unknown linear solver type in SuccessiveLinearizationOptimizer");
}

/* ************************************************************************* */
std::string LinearSolverParams::OrderingTypeTranslator(
    Ordering::OrderingType type) {
  switch (type) {
    case Ordering::METIS:
      return "METIS";
    case Ordering::COLAMD:
      return "COLAMD";
    case Ordering::CUSTOM:
      return "CUSTOM";
    default:
      throw std::invalid_argument(
          "Invalid ordering type: see setOrdering or setOrderingType");
  }
}

/* ************************************************************************* */
Ordering::OrderingType LinearSolverParams::OrderingTypeTranslator(
    const std::string &type) {
  if (type == "METIS")
    return Ordering::METIS;
  if (type == "COLAMD")
    return Ordering::COLAMD;
  if (type == "CUSTOM")
    return Ordering::CUSTOM;
  throw std::invalid_argument(
      "Invalid ordering type: see setOrdering or setOrderingType");
}

}
