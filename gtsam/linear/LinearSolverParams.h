/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LinearSolverParams.h
 * @brief   Parameters base class for Linear Solvers
 * @author  Fan Jiang
 */

#pragma once

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/inference/Ordering.h>

#include <boost/optional.hpp>

namespace gtsam {

// forward declaration
class IterativeOptimizationParameters;

struct GTSAM_EXPORT LinearSolverParams {
 public:
  // Type of solver
  typedef enum LinearSolverType {
    MULTIFRONTAL_CHOLESKY,
    MULTIFRONTAL_QR,
    SEQUENTIAL_CHOLESKY,
    SEQUENTIAL_QR,
    Iterative, /* Experimental Flag - donotuse: for backwards compatibility
                  only. Use PCG or SUBGRAPH instead */
    CHOLMOD,   /* Experimental Flag - donotuse: for backwards compatibility. use
                  SUITESPARSE or PCG/SUBGRAPH w/ iterativeParams instead */
    PCG,
    SUBGRAPH,
    EIGEN_QR,
    EIGEN_CHOLESKY,
    SUITESPARSE,
    CUSPARSE,
    LAST /* for iterating over enum */
  } LinearSolverType;

  /// The type of linear solver to use in the nonlinear optimizer
  /// (default: MULTIFRONTAL_CHOLESKY)
  LinearSolverType linearSolverType = MULTIFRONTAL_CHOLESKY;
  /// The method of ordering use during variable elimination (default: COLAMD)
  Ordering::OrderingType orderingType = Ordering::COLAMD;
  /// The variable elimination ordering, or empty to use COLAMD (default: empty)
  boost::optional<Ordering> ordering;
  /// The container for iterativeOptimization parameters. used in CG Solvers.
  boost::shared_ptr<IterativeOptimizationParameters> iterativeParams;

  /// Construct a params object from the solver and ordering types
  LinearSolverParams(LinearSolverType linearSolverType = MULTIFRONTAL_CHOLESKY,
                     Ordering::OrderingType orderingType = Ordering::COLAMD)
      : linearSolverType(linearSolverType),
        orderingType(orderingType) {};
  /// Construct a params object from the solver type and custom ordering
  LinearSolverParams(LinearSolverType linearSolverType,
                     boost::optional<Ordering> ordering)
      : linearSolverType(linearSolverType),
        orderingType(Ordering::CUSTOM),
        ordering(ordering) {};
  /// Construct a params object from the solver and ordering types as well as
  /// the iterative parameters
  LinearSolverParams(
      LinearSolverType linearSolverType,
      Ordering::OrderingType orderingType,
      boost::shared_ptr<IterativeOptimizationParameters> iterativeParams)
      : linearSolverType(linearSolverType),
        orderingType(orderingType),
        iterativeParams(iterativeParams) {};
  /// Construct a params object from the solver type, custom ordering, and the
  /// iterative parameters
  LinearSolverParams(
      LinearSolverType linearSolverType,
      boost::optional<Ordering> ordering,
      boost::shared_ptr<IterativeOptimizationParameters> iterativeParams)
      : linearSolverType(linearSolverType),
        orderingType(Ordering::CUSTOM),
        ordering(ordering),
        iterativeParams(iterativeParams) {};

  inline bool isMultifrontal() const {
    return (linearSolverType == MULTIFRONTAL_CHOLESKY) ||
           (linearSolverType == MULTIFRONTAL_QR);
  }

  inline bool isSequential() const {
    return (linearSolverType == SEQUENTIAL_CHOLESKY)
           || (linearSolverType == SEQUENTIAL_QR);
  }

  inline bool isCholmod() const {
    return (linearSolverType == CHOLMOD);
  }

  inline bool isIterative() const {
    return (linearSolverType == Iterative) || (linearSolverType == PCG) ||
           (linearSolverType == SUBGRAPH);
  }

  inline bool isEigenQR() const {
    return (linearSolverType == EIGEN_QR);
  }

  inline bool isEigenCholesky() const {
    return (linearSolverType == EIGEN_CHOLESKY);
  }

  inline bool isSuiteSparse() const {
    return (linearSolverType == SUITESPARSE);
  }

  inline bool isCuSparse() const {
    return (linearSolverType == CUSPARSE);
  }

  GaussianFactorGraph::Eliminate getEliminationFunction() const {
    switch (linearSolverType) {
      case MULTIFRONTAL_CHOLESKY:
      case SEQUENTIAL_CHOLESKY:
        return EliminatePreferCholesky;

      case MULTIFRONTAL_QR:
      case SEQUENTIAL_QR:
        return EliminateQR;

      default:
        throw std::runtime_error(
            "Nonlinear optimization parameter \"factorization\" is invalid");
    }
  }

  std::string getLinearSolverType() const {
    return LinearSolverTranslator(linearSolverType);
  }

  void setLinearSolverType(const std::string& solver) {
    linearSolverType = LinearSolverTranslator(solver);
  }

  void setIterativeParams(const boost::shared_ptr<IterativeOptimizationParameters> params);

  void setOrdering(const Ordering& ordering) {
    this->ordering = ordering;
    this->orderingType = Ordering::CUSTOM;
  }

  std::string getOrderingType() const {
    return OrderingTypeTranslator(orderingType);
  }

  // Note that if you want to use a custom ordering, you must set the ordering
  // directly, this will switch to custom type
  void setOrderingType(const std::string& ordering){
    orderingType = OrderingTypeTranslator(ordering);
  }

 private:
  /**
   * @name These functions convert between the string-name and enum versions of
   * these enums.  For example,
   * "SEQUENTIAL_CHOLESKY" <-> LinearSolverType::SEQUENTIAL_CHOLESKY
   * "METIS" <-> Ordering::METIS
   * @{
   */
  static std::string LinearSolverTranslator(
      const LinearSolverType linearSolverType);
  static LinearSolverType LinearSolverTranslator(
      const std::string& linearSolverType);
  static std::string OrderingTypeTranslator(const Ordering::OrderingType type);
  static Ordering::OrderingType OrderingTypeTranslator(const std::string& type);
  /**@}*/
};

}  // namespace gtsam
