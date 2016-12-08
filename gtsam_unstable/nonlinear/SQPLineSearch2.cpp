/*
 * SQPLineSearch2.cpp
 * @brief:
 * @author: Duy-Nguyen Ta
 * @author: Ivan Dario Jimenez
 */

#include <boost/range/adaptor/map.hpp>
#include <gtsam_unstable/linear/QPSolver.h>
#include <gtsam_unstable/nonlinear/SQPLineSearch2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam_unstable/nonlinear/MeritFunction.h>

using namespace std;

namespace gtsam {

bool SQPLineSearch2::checkFeasibility(const Values &x,
    boost::optional<double &> equalityError,
    boost::optional<double &> inequalityError) const {

  double eqError = program_.equalities->error(x);
  double ineqError = program_.inequalities->error(x);

  if (equalityError) {
    *equalityError = eqError;
  }
  if (inequalityError) {
    *inequalityError = ineqError;
  }

  return eqError < 1e-9 && ineqError < 1e-9;
}

Values SQPLineSearch2::getFeasiblePoint() const {
  return Values();
}

/* ************************************************************************* */
GaussianFactorGraph::shared_ptr SQPLineSearch2::multiplyConstrainedHessians(
    const Values& x, VectorValues lambdas, double alpha) const {
  VectorValues multipliedLambdas = alpha * lambdas;
  GaussianFactorGraph::shared_ptr multipliedConstrainedHessians = program_.equalities->multipliedHessians(x, multipliedLambdas);
  multipliedConstrainedHessians->push_back(*(program_.inequalities->multipliedHessians(x, multipliedLambdas)));
  return multipliedConstrainedHessians;
}

/* ************************************************************************* */
bool SQPLineSearch2::checkConvergence(const Values& x,
    const VectorValues& lambdas) const {
  static const bool debug = true;
  if (debug)
    cout << "checkConvergence..." << endl;

  // Check constraints
  double maxError = 0.0;
  for (NonlinearConstraint::shared_ptr factor : *program_.equalities) {
    Vector constrainedError = factor->unwhitenedError(x);
    maxError = constrainedError.maxCoeff();
    if (maxError > 1e-5) {
      if (debug)
        cout << "Violating equality constraint with maxerror: " << maxError
            << endl;
      return false;
    }
  }
  for (NonlinearInequalityConstraint::shared_ptr factor : *program_.inequalities) {
    Vector constrainedError = factor->unwhitenedError(x);
    maxError = constrainedError.maxCoeff();
    if (maxError > 1e-5) {
      if (debug)
        cout << "Violating inequality constraint with maxerror: " << maxError
            << endl;
      return false;
    }
  }

  /*  Check Optimality with KKT conditions
   *  Check KKT conditions:
   *    (1) gradf = lambda*gradc, for all active [both eq and ineq] constraints
   *        - We don't check this condition here. It should be satisfied since lambda
   *        is solved with the dual graph
   *        - TODO: unless we solve for a least square estimate of lambda, then we have to check!!!
   *        - TODO: by default, QPSolver solve for lambda exactly, so we don't have to check
   *        - TODO: gtsam's Constrained QR will have problems with linearly dependent constraints
   *                so lambda solution might not satisfy this condition by default.
   *                Assume no linearly dependent constraint in the dual graph.
   *    (2) lambda <=0, for all ineq.
   *    (3) lambda = 0, for inactive ineq
   */
  NonlinearEqualityFactorGraph allConstraints = *program_.equalities;
  allConstraints += *program_.inequalities;
  for (NonlinearConstraint::shared_ptr factor : allConstraints) {
    Vector lambda = lambdas.at(factor->dualKey());
    double maxLambda = lambda.maxCoeff();
    // For active constraints: gradf = lambda*gradc,
    if (fabs(maxError) < 1e-5) {
      // For active ineq, we want lambda <= 0 (see explaination in QPSolver)
      // so lambda > 0 is are bad
      if (fabs(maxLambda) < 1e-5) {
        if (debug)
          cout << "Violating constraint: with maxlambda:" << maxLambda << endl;
        return false;
      }
    } else {
      // The "error=0" check already passes, it can't be an eq constraint,
      // It must be an inactive ineq constraints, so we want lambda = 0
      if (fabs(maxLambda) > 1e-5) {
        if (debug)
          cout << "Violating inactive ineq constraint with maxlambda: "
              << maxLambda << endl;
        return false;
      }
    }

    if (debug)
      factor->print("Satisfied constraint: ");
    if (debug)
      cout << "error: " << maxError << endl;
    if (debug)
      cout << "lambda: " << lambda << endl;
  }
  if (debug)
    cout << "CONVERGED!!!" << endl;
  return true;
}

/* ************************************************************************* */
GaussianFactorGraph::shared_ptr SQPLineSearch2::buildDampedSystem(
    const GaussianFactorGraph::shared_ptr linear,
    const SQPLineSearch2::State& state) const {
  GaussianFactorGraph::shared_ptr damped(new GaussianFactorGraph());
  double sigma = 1.0 / std::sqrt(state.tau);
  // Straightforward damping:
  for (const Values::ConstKeyValuePair& key_value : state.solution) {
    size_t dim = key_value.value.dim();
    Matrix A = Matrix::Identity(dim, dim);
    Vector b = Vector::Zero(dim);
    SharedDiagonal model = noiseModel::Isotropic::Sigma(dim, sigma);
    *damped += boost::make_shared < JacobianFactor
        > (key_value.key, A, b, model);
  }
  return damped;
}

/* ************************************************************************* */
SQPLineSearch2::State SQPLineSearch2::iterate(
    const SQPLineSearch2::State& currentState) const {
  static const bool useDamping = false;
    GTSAM_PRINT(currentState.solution);
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //1. Build the linearized graph
  QP linearizedProblem;
  // Add linearized constraints first, since we want nonlinear lambdas and
  // QP's linear lambdas have the same indices, based on the constraints' indices
  linearizedProblem.equalities = *program_.equalities->linearize(
      currentState.solution);
  linearizedProblem.inequalities = *program_.inequalities->linearize(
      currentState.solution);
  /*
   * Add unconstrained factors and Lagrangian-multiplied Hessian constraints
   * The Lagrange function is:
   *        L(x,lambda) = f(x) - \sum_j lambda_j*c_j(x)
   * Hence, the objective function of the SQP subproblem is the following:
   *   0.5 p'*(\hessian f - \sum_j lambda_j* (\hessian c_j(x)) )*p + \gradf'*x
   */
  GaussianFactorGraph::shared_ptr linearizedCost = program_.cost->linearize(currentState.solution);
  GaussianFactorGraph::shared_ptr subproblemcost = program_.cost->secondOrderApproximation(currentState.solution);
  subproblemcost->push_back(*(program_.equalities->multipliedHessians(currentState.solution, currentState.lambdas)));
  subproblemcost->push_back(*(program_.inequalities->multipliedHessians(currentState.solution, currentState.lambdas)));
  linearizedProblem.cost = *subproblemcost;
  
  Key pk(Symbol('P',0));
  // Combine to a Lagrangian graph and add constraints' Hessian factors with multipliers
  // Try to solve the damped Lagrangian graph. Increase the damping factor if not ok.
  double newTau = currentState.tau;
  if (useDamping) {
    // Add damping factors
    GaussianFactorGraph::shared_ptr dampedSystem = buildDampedSystem(
        GaussianFactorGraph::shared_ptr(&linearizedProblem.cost), currentState);
    *dampedSystem += linearizedProblem.cost;
    double dampingFactor = 10.0;
    try {
      dampedSystem->optimize();
      // Optimization succeeded: decrease damping factor
      if (newTau > 1e-10) {
        newTau /= dampingFactor;
      }
    } catch (IndeterminantLinearSystemException& e) {
      // Optimization failed: increase damping factor
      newTau *= dampingFactor;
      return State(currentState.solution, currentState.lambdas, currentState.mu,
          newTau, currentState.converged, currentState.k + 1);
    }
  }
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //2. Solve the QP subproblem and compute dLambdas
  QPSolver qpSolver(linearizedProblem);
  VectorValues p;
  VectorValues lambdasHat;
  boost::tie(p, lambdasHat) = qpSolver.optimize();
  //compute dLambdas
  VectorValues dLambdas = lambdasHat - currentState.lambdas;
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //3. Choose a new penalty weight mu
  GaussianFactorGraph::shared_ptr hessianOfLagrangain = program_.cost->hessian(currentState.solution);
  hessianOfLagrangain->push_back(*(program_.equalities->multipliedHessians(currentState.solution, currentState.lambdas)));
  hessianOfLagrangain->push_back(*(program_.inequalities->multipliedHessians(currentState.solution, currentState.lambdas)));
  MeritFunction merit(
    program_, linearizedCost,
    hessianOfLagrangain,
      currentState.solution, p);
  double newMu = merit.computeNewMu(currentState.mu);
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //4. Line search: find the steplength alpha
  double tau = 0.9, eta = 0.3;
  double alpha = 1.0;
  
  while (alpha > 1e-2
      && merit.phi(alpha, newMu)
          > merit.phi(0.0, newMu) + eta * alpha * merit.D(newMu)) {
    alpha = tau * alpha;
  }

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //5. Update solution
  Values newSolution = currentState.solution.retract(alpha * p);
  VectorValues newLambdas = currentState.lambdas + alpha * dLambdas;

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //6. Check convergence
  bool newConvergence = (currentState.converged
      || checkConvergence(currentState.solution, currentState.lambdas));
  return State(newSolution, newLambdas, newMu, newTau, newConvergence,
      currentState.k + 1);
}

/* ************************************************************************* */
VectorValues SQPLineSearch2::zeroFromConstraints(
    const NP& nonlinearProgram) const {
  VectorValues lambdas;
  NonlinearEqualityFactorGraph allConstraints = *nonlinearProgram.equalities;
  allConstraints += *nonlinearProgram.inequalities;
  for (NonlinearConstraint::shared_ptr factor : allConstraints) {
    lambdas.insert(factor->dualKey(), zero(factor->dim()));
  }
  return lambdas;
}

/* ************************************************************************* */
Values SQPLineSearch2::optimize(const Values& initials,
    unsigned int max_iter) const {
  // mu = 0, tau = 1e-5, k = 1
  State currentState(initials, zeroFromConstraints(program_), 0.0, 1e-5, false,
      1);
//    std::cout << "STARTING STATE: " << std::endl;
//    GTSAM_PRINT(currentState);
  while (!currentState.converged && currentState.k < max_iter) {
    currentState = iterate(currentState);
//      std::cout << "ITERATION:" << std::endl;
//      GTSAM_PRINT(currentState);
  }
  if (currentState.k >= max_iter)
    cout << "Not converged within " << max_iter << " iterations." << endl;
  return currentState.solution;
}

} /* namespace gtsam */
