/**
 * @file   SuccessiveLinearizationOptimizer.cpp
 * @brief  
 * @date   Jul 24, 2012
 * @author Yong-Dian Jian
 */

#include <gtsam/nonlinear/SuccessiveLinearizationOptimizer.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianEliminationTree.h>
#include <gtsam/linear/SubgraphSolver.h>
#include <gtsam/linear/VectorValues.h>
#include <boost/shared_ptr.hpp>
#include <stdexcept>

namespace gtsam {

void SuccessiveLinearizationParams::setIterativeParams(const SubgraphSolverParameters &params) {
  iterativeParams = boost::make_shared<SubgraphSolverParameters>(params);
}

void SuccessiveLinearizationParams::print(const std::string& str) const {
  NonlinearOptimizerParams::print(str);
  switch ( linearSolverType ) {
  case MULTIFRONTAL_CHOLESKY:
    std::cout << "         linear solver type: MULTIFRONTAL CHOLESKY\n";
    break;
  case MULTIFRONTAL_QR:
    std::cout << "         linear solver type: MULTIFRONTAL QR\n";
    break;
  case SEQUENTIAL_CHOLESKY:
    std::cout << "         linear solver type: SEQUENTIAL CHOLESKY\n";
    break;
  case SEQUENTIAL_QR:
    std::cout << "         linear solver type: SEQUENTIAL QR\n";
    break;
  case CHOLMOD:
    std::cout << "         linear solver type: CHOLMOD\n";
    break;
  case CONJUGATE_GRADIENT:
    std::cout << "         linear solver type: CONJUGATE GRADIENT\n";
    break;
  default:
    std::cout << "         linear solver type: (invalid)\n";
    break;
  }

  if(ordering)
    std::cout << "                   ordering: custom\n";
  else
    std::cout << "                   ordering: COLAMD\n";

  std::cout.flush();
}

VectorValues solveGaussianFactorGraph(const GaussianFactorGraph &gfg, const SuccessiveLinearizationParams &params)
{
  gttic(solveGaussianFactorGraph);
  VectorValues delta;
  if (params.isMultifrontal()) {
    delta = gfg.optimize(*params.ordering, params.getEliminationFunction());
  } else if(params.isSequential()) {
    delta = gfg.eliminateSequential(*params.ordering, params.getEliminationFunction())->optimize();
  }
  else if ( params.isCG() ) {
    if ( !params.iterativeParams ) throw std::runtime_error("solveGaussianFactorGraph: cg parameter has to be assigned ...");
    if ( boost::dynamic_pointer_cast<SubgraphSolverParameters>(params.iterativeParams) ) {
      SubgraphSolver solver (gfg, *boost::dynamic_pointer_cast<SubgraphSolverParameters>(params.iterativeParams));
      delta = solver.optimize();
    }
    else {
      throw std::runtime_error("solveGaussianFactorGraph: special cg parameter type is not handled in LM solver ...");
    }
  }
  else {
    throw std::runtime_error("solveGaussianFactorGraph: Optimization parameter is invalid");
  }
  return delta;
}

}
