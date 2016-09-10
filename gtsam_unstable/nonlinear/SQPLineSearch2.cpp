/*
 * SQPLineSearch2.cpp
 * @brief:
 * @author: Duy-Nguyen Ta
 * @author: Ivan Dario Jimenez
 */

#include <boost/range/adaptor/map.hpp>
#include <gtsam_unstable/linear/QPSolver.h>
#include <gtsam_unstable/nonlinear/SQPLineSearch2.h>

using namespace std;

namespace gtsam {

bool SQPLineSearch2::checkFeasibility(const Values &x) const {
  //TODO: Add Inequalities
  return program_.equalities.checkFeasibility(x, 1e-9);
}

Values SQPLineSearch2::getFeasiblePoint() const {
  return Values();
}

/* ************************************************************************* */
GaussianFactorGraph::shared_ptr SQPLineSearch2::multiplyConstrainedHessians(
    const Values& x, VectorValues lambdas, double alpha) const {
  GaussianFactorGraph::shared_ptr multipliedConstrainedHessians(
      new GaussianFactorGraph());
  VectorValues multipliedLambdas = alpha * lambdas;
  for (NonlinearConstraint::shared_ptr factor : program_.equalities) {
    multipliedConstrainedHessians->push_back(
        factor->multipliedHessian(x, multipliedLambdas));
  }
  for (NonlinearInequalityConstraint::shared_ptr factor : program_.inequalities) {
    multipliedConstrainedHessians->push_back(
        factor->multipliedHessian(x, multipliedLambdas));
  }
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
  for (NonlinearConstraint::shared_ptr factor : program_.equalities) {
    Vector constrainedError = factor->unwhitenedError(x);
    maxError = constrainedError.maxCoeff();
    if (maxError > 1e-5) {
      if (debug)
        cout << "Violating equality constraint with maxerror: " << maxError
            << endl;
      return false;
    }
  }
  for (NonlinearInequalityConstraint::shared_ptr factor : program_.inequalities) {
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
  for (NonlinearInequalityConstraint::shared_ptr factor : program_.inequalities) {
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
    const GaussianFactorGraph& linear,
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
/**
 * Iteration Strategy
 * Given a point x and lambdas
 * 0. Check convergence
 * 1. Build the linear graph of the QP subproblem
 * 2. Solve QP --> p, lambdasHat --> p, dLambdas
 * 3. Update penalty weights mu

 * 4. Line search, find the best steplength alpha
 * 5. Update solution --> new x, new lambdas
 */
//  SQPLineSearch2::State SQPLineSearch2::iterate(const State &currentState) const {
//    State newState = currentState;
//    newState.k += 1;
// We start at feasible point x with k = 1
// 1. Gradient Evaluation: Evaluate gradient information g and G then:
//   a. evaluate the error in the gradient of the Lagrangian from equation:
//      θ = g - G'λ - v
//   g is the gradient of the cost function
//   G is the Jacobian Matrix of the constraints
//   H is the Hessian Matrix
//
//   b. terminate if the KKT conditions are satisfied
//   c. Compute HL from equation HL = ∇²ₓF - Σ^{m}_{i=1}λ∇²ₓci
//      if this is the first iteration go to step 2
//      else continue
//  d. Levenberg Modification
//      i. compute the rate of change in the norm of the gradient of the Lagrangian from
//          Q3 =  || Θ^K ||∞ ÷ || Θ^{K-2} || ∞
//      ii. if Q1 <= 0.25Q2 set τ^k ← min(2τ^{k-1},1)
//      iii. else if Q1 >= 0.75 Q2 set τ^k= τ^(k-1)min(0.5,Q3)
//2. Search Direction. Construct the optimization search direction:
//   a. compute H from H = HL + τ(|σ|+1)I
//   b. compute p by solving the QP subproblem 2.14-2.15
//   c. Inertia control: if inertia K is incorrect and...
//   d. compute Δλ and Δv from 2.31 and 2.32
//   e. compute Δs and Δt from 2.34 and 2.35
//   f. compute penalty parameters to satisfy 2.6 and
//   g. initialize α = 1
// 3. Prediction
//    a. compute the predicted point for the variables, the multipliers, and the slacks from 2.30
//    b. evaluate the constraints cbar = c(xbar)
// 4. Line Search. Evaluate the merit function M(xbar, λbar, vbar, sbar, tbar) = Mbar and
//    a. if the merit function is sufficiently less than M, the xbar is an improved point
//       Terminate the line search and go to step 5
//    b. else change the steplength α to reduce M and return to step 3
// 5. Update. Update all quantities and set k = k + 1;
//    a. compute the actual reduction from 2.42;
//    b. compute the predicted reduction from 2.43 where Mbar^k is the predicted value of the merid function
//    c. return to step 1
//    return newState;
//  }
/* ************************************************************************* */
SQPLineSearch2::State SQPLineSearch2::iterate(
    const SQPLineSearch2::State& currentState) const {
  static const bool debug = false;
  static const bool useDamping = false;

  // Don't do anything if it's already converged!
  if (currentState.converged)
    return currentState;

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //1. Build the linearized graph
  QP linearizedProblem;
  // Add linearized constraints first, since we want nonlinear lambdas and
  // QP's linear lambdas have the same indices, based on the constraints' indices
  linearizedProblem.equalities = *program_.equalities.linearize(
      currentState.solution);
  linearizedProblem.inequalities = *program_.inequalities.linearize(
      currentState.solution);

  if (debug) {
    linearizedProblem.equalities.print("Linearized Equalities.");
    linearizedProblem.inequalities.print("Linearized Inequalities");
  }
  /*
   * Add unconstrained factors and Lagrangian-multiplied Hessian constraints
   * The Lagrange function is:
   *        L(x,lambda) = f(x) - \sum_j lambda_j*c_j(x)
   * Hence, the objective function of the SQP subproblem is the following:
   *   0.5 p'*(\hessian f - \sum_j lambda_j* (\hessian c_j(x)) )*p + \gradf'*x
   */
  linearizedProblem.cost = *program_.cost.linearize(currentState.solution);
  GaussianFactorGraph linearizedCost = linearizedProblem.cost;
  // Combine to a Lagrangian graph and add constraints' Hessian factors with multipliers
  linearizedProblem.cost += *multiplyConstrainedHessians(currentState.solution,
      currentState.lambdas, -1.0);

  // Try to solve the damped Lagrangian graph. Increase the damping factor if not ok.
  double newTau = currentState.tau;
  if (useDamping) {
    // Add damping factors
    GaussianFactorGraph::shared_ptr dampedSystem = buildDampedSystem(
        linearizedProblem.cost, currentState);
    *dampedSystem += linearizedProblem.cost;
    double dampingFactor = 10.0;
    try {
      dampedSystem->optimize();
      // Optimization succeeded: decrease damping factor
      if (newTau > 1e-10) {
        newTau /= dampingFactor;
        if (debug)
          cout << "Decrease tau: " << newTau << endl;
      }
    } catch (IndeterminantLinearSystemException& e) {
      // Optimization failed: increase damping factor
      newTau *= dampingFactor;
      if (debug)
        cout << "Increase tau: " << newTau << endl;
      return State(currentState.solution, currentState.lambdas, currentState.mu,
          newTau, currentState.converged, currentState.k + 1);
    }
  }
  if (debug)
    GTSAM_PRINT(linearizedProblem);

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //2. Solve the QP subproblem and compute dLambdas
  if (debug)
    cout << "Solving QP subproblem: " << endl;
  QPSolver qpSolver(linearizedProblem);
  VectorValues p;
  VectorValues lambdasHat;
  boost::tie(p, lambdasHat) = qpSolver.optimize();
  if (debug)
    p.print("p =  ");
  if (debug)
    lambdasHat.print("lambdasHat = ");

  //compute dLambdas
  VectorValues dLambdas = lambdasHat - currentState.lambdas;
  if (debug)
    dLambdas.print("dLambdas = ");

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //3. Choose a new penalty weight mu
  MeritFunction merit(program_, linearizedCost, linearizedProblem.cost,
      currentState.solution, p);
  double newMu = merit.computeNewMu(currentState.mu);
  if (debug)
    cout << "newMu: " << newMu << endl;

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
  if (debug)
    cout << "Final alpha: " << alpha << endl;
  Values newSolution = currentState.solution.retract(alpha * p);
  if (debug)
    newSolution.print("newSolution: ");
  VectorValues newLambdas = currentState.lambdas + alpha * dLambdas;
  if (debug)
    newLambdas.print("newLambdas: ");

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //6. Check convergence
  if (debug)
    currentState.print("currentState: ");
  bool newConvergence = (currentState.converged
      || checkConvergence(currentState.solution, currentState.lambdas));
  if (debug)
    cout << "newConvergence: " << (int) newConvergence << endl;
  return State(newSolution, newLambdas, newMu, newTau, newConvergence,
      currentState.k + 1);
}

/* ************************************************************************* */
VectorValues SQPLineSearch2::zeroFromConstraints(
    const NP& nonlinearProgram) const {
  VectorValues lambdas;
  for (size_t iFactor = 0; iFactor < nonlinearProgram.equalities.size();
      ++iFactor) {
    lambdas.insert(iFactor,
        zero(nonlinearProgram.equalities.at(iFactor)->dim()));
  }
  size_t offset = nonlinearProgram.equalities.size();
  for (size_t iFactor = 0; iFactor < nonlinearProgram.inequalities.size();
      ++iFactor) {
    lambdas.insert(iFactor + offset,
        zero(nonlinearProgram.inequalities.at(iFactor)->dim()));
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

/* ************************************************************************* */
MeritFunction::MeritFunction(const NP & program,
    const GaussianFactorGraph & linearizedCost,
    const GaussianFactorGraph & lagrangianGraph, const Values& x,
    const VectorValues& p) :
    program_(program), linearizedCost_(linearizedCost), lagrangianGraph_(
        lagrangianGraph), x_(x), p_(p) {
  gradf_ = linearizedCost_.gradientAtZero();
  if (gradf_.size() < p_.size()) {
    for (Key key : p_ | boost::adaptors::map_keys) {
      if (!gradf_.exists(key)) {
        gradf_.insert(key, zero(p_.at(key).size()));
      }
    }
  }
}

/* ************************************************************************* */
double MeritFunction::constraintNorm1(const Values x) const {
  double norm1 = 0.0;
  for (NonlinearInequalityConstraint::shared_ptr factor : program_.inequalities) {
    Vector error = factor->unwhitenedError(x);
    norm1 += error.cwiseAbs().sum();
  }
  for (NonlinearConstraint::shared_ptr factor : program_.equalities) {
    Vector error = factor->unwhitenedError(x);
    norm1 += error.cwiseAbs().sum();
  }
  return norm1;
}

/* ************************************************************************* */
double MeritFunction::phi(double alpha, double mu) const {
  static const bool debug = false;
  Values x2 = (fabs(alpha) > 1e-5) ? x_.retract(alpha * p_) : x_;
  double c2 = constraintNorm1(x2);

  double result = program_.cost.error(x2) + mu * c2;
  if (debug)
    cout << "phi(" << alpha << ") = " << result << endl;
  return result;
}

/* ************************************************************************* */
double MeritFunction::D(double mu) const {
  static const bool debug = false;
  double result = p_.dot(gradf_) - mu * constraintNorm1(x_);
  if (debug)
    cout << "D() = " << result << endl;
  return result;
}

/* ************************************************************************* */
double MeritFunction::computeNewMu(double currentMu) const {
  static const bool debug = false;
  static const double rho = 0.7;
  double muLowerBound = (p_.dot(gradf_) + 0.5 * ptHp(lagrangianGraph_, p_))
      / ((1 - rho) * constraintNorm1(x_));

  if (debug)
    cout << "gradfk'p = " << (p_.dot(gradf_)) << endl;
  if (debug)
    cout << "ptHp = " << ptHp(lagrangianGraph_, p_) << endl;
  if (debug)
    cout << "||c||_1 = " << constraintNorm1(x_) << endl;
  if (debug)
    cout << "mu lower bound = " << muLowerBound << endl;

  if (currentMu > muLowerBound)
    return currentMu;
  else
    return muLowerBound * 1.1;
}

/* ************************************************************************* */
double MeritFunction::ptHp(const GaussianFactorGraph& linear,
    const VectorValues& p) const {
  double result = 0.0;
  for (const GaussianFactor::shared_ptr& factor : linear) {
    VectorValues y = VectorValues::Zero(p);
    factor->multiplyHessianAdd(1.0, p, y); // y = Hx
    result += p.dot(y); // x'y = x'Hx
  }
  return result;
}

} /* namespace gtsam */
