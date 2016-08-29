/*
 * SQPLineSearch2.h
 * @brief:
 * @date: Aug 26, 2016
 * @author: Ivan Dario Jimenez
 */

#include <gtsam_unstable/nonlinear/LocalSQP.h>
#include <gtsam_unstable/linear/QPSolver.h>
#include <gtsam/inference/Symbol.h>

namespace gtsam {
Values LocalSQP::optimize(const Values &initials, unsigned int max_iter) const {
  VectorValues duals;
  for (unsigned int index = 0; index < program_.equalities.size(); index++) {
    duals.insert(program_.equalities[index]->dualKey(),
        zero(program_.equalities[index]->dim()));
  }
  State currentState(initials, duals);
  while (!currentState.converged) {
    currentState = iterate(currentState);
  }
  return currentState.current_solution;
}


Matrix LocalSQP::makeHessianOfLagrangian(const State& currentState) const {
  Matrix HessianOfCost = GaussianFactorGraph(
    *program_.cost.linearize(currentState.current_solution)).hessian().first;
  Matrix HessianOfConstraints = GaussianFactorGraph(
    *program_.cost.linearize(currentState.current_solution)).hessian().first;
  return HessianOfCost - currentState.current_lambda.vector() * HessianOfConstraints;
}
  
LocalSQP::State LocalSQP::iterate(const State& currentState) const {
  //1. Evaluate cost function at current solution
  double currentCost = program_.cost.cost(currentState.current_solution);
  //2. Evaluate gradient of cost function at current solution
  Matrix gradient = getGradientOfCostAt(currentState.current_solution);

  //3. Evaluate Second Gradient on current solution of Lagrangian
  Matrix HessianOfLagrangian = makeHessianOfLagrangian(currentState);
  //=  hessianOfCost - currentState.current_lambda * hessianOfConstraints;
  //4. Evaluate constraint error for all constraints on current solution
  double constraintError = program_.equalities.cost(
      currentState.current_solution);
  //5. Evaluate Jacobian of Constraints at current error
  Matrix jacobianOfConstraints =
      GaussianFactorGraph(
          *program_.equalities.linearize(currentState.current_solution)).jacobian().first;
  //6. Solve for Pk with QP Solver
  Key P(Symbol('P',1)), PL(Symbol('P',0));
  QP qp;
  qp.cost.push_back(HessianFactor(P, HessianOfLagrangian, gradient, currentCost));
  qp.equalities.push_back(LinearEquality(P, jacobianOfConstraints, I_1x1 * constraintError, PL));
  QPSolver solver(qp);
  std::pair<VectorValues,VectorValues> result = solver.optimize();
  Vector pk = result.first.at(P);
  Vector lk = result.second.at(PL);
  //7. Update Everything
  State newState;
  //Update solution
//  for(const Key & k : currentState.current_solution.keys()){
//
//  }
  //Update Lambdas
//  Vector new_solution_tangent = currentState.current_solution.localCoordinates()
//  pk.retract();
  //Update Iteration Count
  newState.k = currentState.k + 1;
  //Check for convergence
//  return newState;
  return State(Values(), VectorValues(), 1, true);
}

Matrix LocalSQP::getGradientOfCostAt(const Values &linearizationPoint) const {
  return GaussianFactorGraph(*program_.cost.linearize(linearizationPoint)).jacobian().first;
}

}
