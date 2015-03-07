/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SuccessiveLinearizationOptimizer.h
 * @brief 
 * @author Richard Roberts
 * @date Apr 1, 2012
 */

#pragma once

#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/linear/SubgraphSolver.h>

namespace gtsam {

class SuccessiveLinearizationParams : public NonlinearOptimizerParams {
public:
  /** See SuccessiveLinearizationParams::linearSolverType */
  enum LinearSolverType {
    MULTIFRONTAL_CHOLESKY,
    MULTIFRONTAL_QR,
    SEQUENTIAL_CHOLESKY,
    SEQUENTIAL_QR,
    CONJUGATE_GRADIENT,         /* Experimental Flag */
    CHOLMOD,    /* Experimental Flag */
  };

	LinearSolverType linearSolverType; ///< The type of linear solver to use in the nonlinear optimizer
  boost::optional<Ordering> ordering; ///< The variable elimination ordering, or empty to use COLAMD (default: empty)
  IterativeOptimizationParameters::shared_ptr iterativeParams; ///< The container for iterativeOptimization parameters. used in CG Solvers.

  SuccessiveLinearizationParams() : linearSolverType(MULTIFRONTAL_CHOLESKY) {}
  virtual ~SuccessiveLinearizationParams() {}

    inline bool isMultifrontal() const {
    return (linearSolverType == MULTIFRONTAL_CHOLESKY) || (linearSolverType == MULTIFRONTAL_QR); }

  inline bool isSequential() const {
    return (linearSolverType == SEQUENTIAL_CHOLESKY) || (linearSolverType == SEQUENTIAL_QR); }

  inline bool isCholmod() const { return (linearSolverType == CHOLMOD); }

  inline bool isCG() const { return (linearSolverType == CONJUGATE_GRADIENT); }

  virtual void print(const std::string& str) const;

  GaussianFactorGraph::Eliminate getEliminationFunction() const {
    switch (linearSolverType) {
    case MULTIFRONTAL_CHOLESKY:
    case SEQUENTIAL_CHOLESKY:
      return EliminatePreferCholesky;

    case MULTIFRONTAL_QR:
    case SEQUENTIAL_QR:
      return EliminateQR;

    default:
      throw std::runtime_error("Nonlinear optimization parameter \"factorization\" is invalid");
      return EliminateQR;
      break;
    }
  }

	std::string getLinearSolverType() const { return linearSolverTranslator(linearSolverType); }

	void setLinearSolverType(const std::string& solver) { linearSolverType = linearSolverTranslator(solver); }
  void setIterativeParams(const SubgraphSolverParameters &params);
	void setOrdering(const Ordering& ordering) { this->ordering = ordering; }

private:
	std::string linearSolverTranslator(LinearSolverType linearSolverType) const {
		switch(linearSolverType) {
		case MULTIFRONTAL_CHOLESKY: return "MULTIFRONTAL_CHOLESKY";
		case MULTIFRONTAL_QR: return "MULTIFRONTAL_QR";
		case SEQUENTIAL_CHOLESKY: return "SEQUENTIAL_CHOLESKY";
		case SEQUENTIAL_QR: return "SEQUENTIAL_QR";
		case CONJUGATE_GRADIENT: return "CONJUGATE_GRADIENT";
		case CHOLMOD: return "CHOLMOD";
		default: throw std::invalid_argument("Unknown linear solver type in SuccessiveLinearizationOptimizer");
		}
	}
	LinearSolverType linearSolverTranslator(const std::string& linearSolverType) const {
		if(linearSolverType == "MULTIFRONTAL_CHOLESKY") return MULTIFRONTAL_CHOLESKY;
		if(linearSolverType == "MULTIFRONTAL_QR") return MULTIFRONTAL_QR;
		if(linearSolverType == "SEQUENTIAL_CHOLESKY") return SEQUENTIAL_CHOLESKY;
		if(linearSolverType == "SEQUENTIAL_QR") return SEQUENTIAL_QR;
		if(linearSolverType == "CONJUGATE_GRADIENT") return CONJUGATE_GRADIENT;
		if(linearSolverType == "CHOLMOD") return CHOLMOD;
		throw std::invalid_argument("Unknown linear solver type in SuccessiveLinearizationOptimizer");
	}
};

/* a wrapper for solving a GaussianFactorGraph according to the parameters */
VectorValues solveGaussianFactorGraph(const GaussianFactorGraph &gfg, const SuccessiveLinearizationParams &params) ;

} /* namespace gtsam */
