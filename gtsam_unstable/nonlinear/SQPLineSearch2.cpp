/*
 * SQPLineSearch2.cpp
 * @brief:
 * @date: Apr 29, 2014
 * @author: Duy-Nguyen Ta
 */

#include <boost/range/adaptor/map.hpp>
#include <gtsam_unstable/linear/QPSolver.h>
#include <gtsam_unstable/nonlinear/SQPLineSearch2.h>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
GaussianFactorGraph::shared_ptr SQPLineSearch2::multiplyConstrainedHessians(
    const NonlinearFactorGraph& constrainedGraph, const Values& x,
    VectorValues lambdas, double alpha) const {
  GaussianFactorGraph::shared_ptr multipliedConstrainedHessians(
        new GaussianFactorGraph());
  for (size_t iFactor = 0; iFactor < constrainedGraph.size(); ++iFactor) {
//    NonlinearEqualityConstraint::shared_ptr constraint = toConstraint(
//          constrainedGraph.at(iFactor));
    //TODO: figure out was Duy is trying to do here.
//    VectorValues multipliedLambdas = lambdas.at(iFactor) * alpha;
//    multipliedConstrainedHessians->push_back(
//      *constraint->multipliedHessian(x, alpha*lambdas.at(iFactor)));
  }
  return multipliedConstrainedHessians;
}

/* ************************************************************************* */
bool SQPLineSearch2::checkConvergence(const Values& x, const VectorValues& lambdas) const {
  static const bool debug = true;
  if (debug) cout << "checkConvergence..." << endl;

  // Check constraints
//  double maxError = 0.0;
//  NoiseModelFactor::shared_ptr factor;
//  for (size_t iFactor = 0; iFactor<constrained_.size(); ++iFactor) {
//    factor = SQPLineSearch2::toNoiseModel(constrained_.at(iFactor));
//
//    // Check errors: 0 for eq, <=0 for ineq
//    Vector constrainedError = factor->unwhitenedError(x);
//    maxError = constrainedError.maxCoeff();
//    if (maxError > 1e-5)  {
//      if (debug) cout << "Violating constraint: " << iFactor << " with maxerror: " << maxError << endl;
//      return false;
//    }
//  }

//  for (size_t iFactor = 0; iFactor<constrained_.size(); ++iFactor) {
//    /*  Check Optimality with KKT conditions
//     *  Check KKT conditions:
//     *    (1) gradf = lambda*gradc, for all active [both eq and ineq] constraints
//     *        - We don't check this condition here. It should be satisfied since lambda
//     *        is solved with the dual graph
//     *        - TODO: unless we solve for a least square estimate of lambda, then we have to check!!!
//     *        - TODO: by default, QPSolver solve for lambda exactly, so we don't have to check
//     *        - TODO: gtsam's Constrained QR will have problems with linearly dependent constraints
//     *                so lambda solution might not satisfy this condition by default.
//     *                Assume no linearly dependent constraint in the dual graph.
//     *    (2) lambda <=0, for all ineq.
//     *    (3) lambda = 0, for inactive ineq
//     */
//    Vector lambda = lambdas.at(iFactor);
//    double maxLambda = lambda.maxCoeff();
//    // For active constraints: gradf = lambda*gradc,
//    if (fabs(maxError) < 1e-5) {
//      // For active ineq, we want lambda <= 0 (see explaination in QPSolver)
//      // so lambda > 0 is are bad
//      if (fabs(maxLambda) < 1e-5) {
//        if (debug) cout << "Violating constraint: " << iFactor
//             << " with maxlambda: " << maxLambda << endl;
//        return false;
//      }
//    }
//    else {
//      // The "error=0" check already passes, it can't be an eq constraint,
//      // It must be an inactive ineq constraints, so we want lambda = 0
//      if (fabs(maxLambda) > 1e-5) {
//        if (debug) cout << "Violating inactive ineq constraint: " << iFactor
//             << " with maxlambda: " << maxLambda << endl;
//        return false;
//      }
//    }
//
//    if (debug) factor->print("Satisfied constraint: ");
//    if (debug) cout << "error: " << maxError << endl;
//    if (debug) cout << "lambda: " << lambda << endl;
//  }

  if (debug) cout << "CONVERGED!!!" << endl;
  return true;
}

/* ************************************************************************* */
GaussianFactorGraph::shared_ptr SQPLineSearch2::buildDampedSystem(
    const GaussianFactorGraph& linear, const SQPLineSearch2::State& state) const {
  GaussianFactorGraph::shared_ptr damped(new GaussianFactorGraph());
  double sigma = 1.0 / std::sqrt(state.tau);
  // Straightforward damping:
  for(const Values::ConstKeyValuePair& key_value: state.solution) {
    size_t dim = key_value.value.dim();
    Matrix A = Matrix::Identity(dim, dim);
    Vector b = Vector::Zero(dim);
    SharedDiagonal model = noiseModel::Isotropic::Sigma(dim, sigma);
    *damped += boost::make_shared<JacobianFactor>(key_value.key, A, b, model);
  }
  return damped;
}


/* ************************************************************************* */
/**
 * Iterate 1 step
 * Given x, lambdas
 * 0. Check convergence
 * 1. Build the linear graph of the QP subproblem
 * 2. Solve QP --> p, lambdasHat --> p, dLambdas
 * 3. Update penalty weights mu
 * 4. Line search, find the best steplength alpha
 * 5. Update solution --> new x, new lambdas
 */
SQPLineSearch2::State SQPLineSearch2::iterate(
    const SQPLineSearch2::State& currentState) const {
  static const bool debug = false;
  static const bool useDamping = false;

  // Don't do anything if it's already converged!
  if (currentState.converged)
    return currentState;

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //1. Build the linear graph
  QP fullQP;

  // Add linearized constraints first, since we want nonlinear lambdas and
  // QP's linear lambdas have the same indices, based on the constraints' indices
//  GaussianFactorGraph::shared_ptr linearConstraints = constrained_.linearize(
//        currentState.solution);
//  if (debug) linearConstraints->print("Linear constraints: ");
    //TODO: this probably shouldn't be equalities. It should depend on the original linearized constraint.
//  for(auto linearConstraint: *linearConstraints){
//    fullQP.equalities.push_back(*linearConstraint);
//  }

  /*
   * Add unconstrained factors and Lagrangian-multiplied Hessian constraints
   * The Lagrange function is:
   *        L(x,lambda) = f(x) - \sum_j lambda_j*c_j(x)
   * Hence, the objective function of the SQP subproblem is the following:
   *   0.5 p'*(\hessian f - \sum_j lambda_j* (\hessian c_j(x)) )*p + \gradf'*x
   */
//  GaussianFactorGraph::shared_ptr linearUnconstraints = unconstrained_.linearize(
//        currentState.solution);

//  // Add damping factors
//  if (useDamping) {
//    GaussianFactorGraph::shared_ptr dampedSystem = buildDampedSystem(*linearUnconstraints, currentState);
//    *linearUnconstraints += *dampedSystem;
//  }
//
//  // Combine to a Lagrangian graph and add constraints' Hessian factors with multipliers
//  GaussianFactorGraph::shared_ptr lagrangianGraph(new GaussianFactorGraph());
//  *lagrangianGraph += *linearUnconstraints;
//  *lagrangianGraph += *multiplyConstrainedHessians(constrained_, currentState.solution,
//                                                   currentState.lambdas, -1.0);

  // Try to solve the damped Lagrangian graph. Increase the damping factor if not ok.
//  double newTau = currentState.tau;
//  if (useDamping) {
//    double dampingFactor = 10.0;
//    try {
//      lagrangianGraph->optimize();
//      // Optimization succeeded: decrease damping factor
//      if (newTau > 1e-10) {
//        newTau  /= dampingFactor;
//        if (debug) cout << "Decrease tau: " << newTau << endl;
//      }
//    } catch (IndeterminantLinearSystemException& e) {
//      // Optimization failed: increase damping factor
//      newTau *= dampingFactor;
//      if (debug) cout << "Increase tau: " << newTau << endl;
//      return currentState.newDamping(newTau);
//    }
//  }
//  //TODO: I am not sure this is supposed to be added like this.
//  fullQP.cost = *lagrangianGraph;
//  if (debug) GTSAM_PRINT(fullQP);

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //2. Solve the QP subproblem and compute dLambdas
//  if (debug) cout << "Solving QP subproblem: " << endl;
//  QPSolver qp(fullQP);
//  VectorValues p;
//  VectorValues lambdasHat;
//  boost::tie(p,lambdasHat) = qp.optimize();
//  if (debug) p.print("p =  ");
//  if (debug) lambdasHat.print("lambdasHat = ");
//
//  //compute dLambdas
//  VectorValues dLambdas = lambdasHat - currentState.lambdas;
//  if (debug) dLambdas.print("dLambdas = ");

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //3. Choose a new penalty weight mu
//  MeritFunction2 merit(unconstrained_, constrained_, linearUnconstraints,
//                       lagrangianGraph, currentState.solution, p);
//  double newMu = merit.computeNewMu(currentState.mu);
//  if (debug) cout << "newMu: " << newMu << endl;

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //4. Line search: find the steplength alpha
//  double tau = 0.9, eta = 0.3;
//  double alpha = 1.0;
////  while (alpha > 1e-2 && merit.phi(alpha, newMu) > merit.phi(0.0, newMu) + eta*alpha*merit.D(newMu)) {
//    alpha = tau*alpha;
//  }

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //5. Update solution
//  if (debug) cout << "Final alpha: " << alpha << endl;
//  Values newSolution = currentState.solution.retract(alpha * p);
//  if (debug) newSolution.print("newSolution: ");
//  VectorValues newLambdas = currentState.lambdas + alpha * dLambdas;
//  if (debug) newLambdas.print("newLambdas: ");

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //6. Check convergence
//  if (debug) currentState.print("currentState: ");
//  bool newConvergence = (currentState.converged || checkConvergence(currentState.solution, currentState.lambdas));
//  if (debug) cout << "newConvergence: " << (int)newConvergence << endl;
//  return State(newSolution, newLambdas, newMu, newTau, newConvergence);
  return State();
}

/* ************************************************************************* */
VectorValues SQPLineSearch2::zeroFromConstraints(
    const NonlinearFactorGraph& constrained) const {
  VectorValues lambdas;
  for (size_t iFactor = 0; iFactor < constrained.size(); ++iFactor) {
    //TODO: Update this for neww  types of constraints
//    NoiseModelFactor::shared_ptr factorC = toNoiseModel(constrained.at(iFactor));
//    lambdas.insert(iFactor, zero(factorC->get_noiseModel()->dim()));
  }
  return lambdas;
}

/* ************************************************************************* */
Values SQPLineSearch2::optimize(const Values& initials) const {
  return initials;
    
  SQPLineSearch2::State currentState;
//  currentState.solution = initials;
//  currentState.lambdas = zeroFromConstraints(constrained_);
//  currentState.mu = 0.0;
//  currentState.tau = 1e-5;
  int iter = 0;
  int maxIter = 200;
  while (!currentState.converged && iter < maxIter) {
    if (iter >= maxIter) {
      cout << "Not converged within " << maxIter << " iterations." << endl;
      break;
    }
    iter++;
    cout << "Iteration: " << iter << endl;
    currentState = iterate(currentState);
  }
  return currentState.solution;
}

/* ************************************************************************* */
MeritFunction2::MeritFunction2(const NP & program,
                               const GaussianFactorGraph::shared_ptr& linearUnconstrained,
    const GaussianFactorGraph::shared_ptr& lagrangianGraph, const Values& x,
                               const VectorValues& p) :
    program_(program), linearUnconstrained_(
                                                              linearUnconstrained), lagrangianGraph_(lagrangianGraph), x_(x), p_(p) {
  gradf_ = linearUnconstrained_->gradientAtZero();
  if (gradf_.size() < p_.size()) {
    for(Key key: p_ | boost::adaptors::map_keys) {
      if (!gradf_.exists(key)) {
        gradf_.insert(key, zero(p_.at(key).size()));
      }
    }
  }
}

/* ************************************************************************* */
double MeritFunction2::constraintNorm1(const Values x) const {
  double norm1 = 0.0;
//  for (size_t iFactor = 0; iFactor<constrained_.size(); ++iFactor) {
//    //TODO: update this for current version
////    NoiseModelFactor::shared_ptr factor = SQPLineSearch2::toNoiseModel(
////          constrained_.at(iFactor));
////    Vector error = factor->unwhitenedError(x);
////    norm1 += error.cwiseAbs().sum();
//  }
  return norm1;
}

/* ************************************************************************* */
double MeritFunction2::phi(double alpha, double mu) const {
  static const bool debug = false;
  Values x2 = (fabs(alpha)>1e-5) ? x_.retract(alpha*p_) : x_;
  double c2 = constraintNorm1(x2);
  double result;
//  double result = unconstrained_.error(x2) + mu*c2;
  if (debug) cout << "phi(" << alpha << ") = " << result << endl;
  return result;
}

/* ************************************************************************* */
double MeritFunction2::D(double mu) const {
  static const bool debug = false;
  double result = p_.dot(gradf_) - mu*constraintNorm1(x_);
  if (debug) cout << "D() = " << result << endl;
  return result;
}

/* ************************************************************************* */
double MeritFunction2::computeNewMu(double currentMu) const {
  static const bool debug = false;
  static const double rho = 0.7;
  double muLowerBound = (p_.dot(gradf_)
                         + 0.5 * ptHp(*lagrangianGraph_, p_)) / ((1 - rho) * constraintNorm1(x_));

  if (debug) cout << "gradfk'p = " << (p_.dot(gradf_)) << endl;
  if (debug) cout << "ptHp = " << ptHp(*lagrangianGraph_, p_) << endl;
  if (debug) cout << "||c||_1 = " << constraintNorm1(x_) << endl;
  if (debug) cout << "mu lower bound = " << muLowerBound << endl;

  if (currentMu > muLowerBound)
    return currentMu;
  else
    return muLowerBound*1.1;
}

/* ************************************************************************* */
double MeritFunction2::ptHp(const GaussianFactorGraph& linear,
                            const VectorValues& p) const {
  double result = 0.0;
  for(const GaussianFactor::shared_ptr& factor: linear) {
    VectorValues y = VectorValues::Zero(p);
    factor->multiplyHessianAdd(1.0, p, y); // y = Hx
    result += p.dot(y);// x'y = x'Hx
  }
  return result;
}

} /* namespace gtsam */
