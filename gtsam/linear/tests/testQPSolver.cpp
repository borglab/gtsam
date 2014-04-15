/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testQPSolver.cpp
 * @brief Test simple QP solver for a linear inequality constraint
 * @date Apr 10, 2014
 * @author Duy-Nguyen Ta
 */

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/base/Testable.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace gtsam::symbol_shorthand;

#define TEST_DISABLED(testGroup, testName)\
    void testGroup##testName##Test(TestResult& result_, const std::string& name_)

class QPSolver {
  const GaussianFactorGraph& graph_;   //!< the original graph, can't be modified!
  FastVector<size_t> constraintIndices_; //!< Indices of constrained factors in the original graph
  GaussianFactorGraph::shared_ptr freeHessians_; //!< unconstrained Hessians of constrained variables
  VariableIndex freeHessianFactorIndex_; //!< indices of unconstrained Hessian factors of constrained variables
                                        // gtsam calls it "VariableIndex", but I think FactorIndex
                                        // makes more sense, because it really stores factor indices.
  VariableIndex fullFactorIndices_; //!< indices of factors involving each variable.
                                    // gtsam calls it "VariableIndex", but I think FactorIndex
                                    // makes more sense, because it really stores factor indices.

public:
  /// Constructor
  QPSolver(const GaussianFactorGraph& graph) :
      graph_(graph), fullFactorIndices_(graph) {

    // Split the original graph into unconstrained and constrained part
    // and collect indices of constrained factors
    for (size_t i = 0; i < graph.nrFactors(); ++i) {
      // obtain the factor and its noise model
      JacobianFactor::shared_ptr jacobian = toJacobian(graph.at(i));
      if (jacobian && jacobian->get_model()
          && jacobian->get_model()->isConstrained()) {
        constraintIndices_.push_back(i);
      }
    }

    // Collect constrained variable keys
    KeySet constrainedVars;
    BOOST_FOREACH(size_t index, constraintIndices_) {
      KeyVector keys = graph[index]->keys();
      constrainedVars.insert(keys.begin(), keys.end());
    }

    // Collect unconstrained hessians of constrained vars to build dual graph
    freeHessians_ = unconstrainedHessiansOfConstrainedVars(graph, constrainedVars);
    freeHessianFactorIndex_ = VariableIndex(*freeHessians_);
  }

  /// Return indices of all constrained factors
  FastVector<size_t> constraintIndices() const { return constraintIndices_; }

  /// Convert a Gaussian factor to a jacobian. return empty shared ptr if failed
  JacobianFactor::shared_ptr toJacobian(const GaussianFactor::shared_ptr& factor) const {
    JacobianFactor::shared_ptr jacobian(
        boost::dynamic_pointer_cast<JacobianFactor>(factor));
    return jacobian;
  }

  /// Convert a Gaussian factor to a Hessian. Return empty shared ptr if failed
  HessianFactor::shared_ptr toHessian(const GaussianFactor::shared_ptr factor) const {
    HessianFactor::shared_ptr hessian(boost::dynamic_pointer_cast<HessianFactor>(factor));
    return hessian;
  }

  /// Return the Hessian factor graph of constrained variables
  GaussianFactorGraph::shared_ptr freeHessiansOfConstrainedVars() const {
    return freeHessians_;
  }

  /* ************************************************************************* */
  GaussianFactorGraph::shared_ptr unconstrainedHessiansOfConstrainedVars(
      const GaussianFactorGraph& graph, const KeySet& constrainedVars) const {
    VariableIndex variableIndex(graph);
    GaussianFactorGraph::shared_ptr hfg = boost::make_shared<GaussianFactorGraph>();

    // Collect all factors involving constrained vars
    FastSet<size_t> factors;
    BOOST_FOREACH(Key key, constrainedVars) {
      VariableIndex::Factors factorsOfThisVar = variableIndex[key];
      BOOST_FOREACH(size_t factorIndex, factorsOfThisVar) {
        factors.insert(factorIndex);
      }
    }

    // Convert each factor into Hessian
    BOOST_FOREACH(size_t factorIndex, factors) {
      if (!graph[factorIndex]) continue;
      // See if this is a Jacobian factor
      JacobianFactor::shared_ptr jf = toJacobian(graph[factorIndex]);
      if (jf) {
        // Dealing with mixed constrained factor
        if (jf->get_model() && jf->isConstrained()) {
          // Turn a mixed-constrained factor into a factor with 0 information on the constrained part
          Vector sigmas = jf->get_model()->sigmas();
          Vector newPrecisions(sigmas.size());
          bool mixed = false;
          for (size_t s=0; s<sigmas.size(); ++s) {
            if (sigmas[s] <= 1e-9) newPrecisions[s] = 0.0; // 0 info for constraints (both ineq and eq)
            else {
              newPrecisions[s] = 1.0/sigmas[s];
              mixed = true;
            }
          }
          if (mixed) {  // only add free hessians if it's mixed
            JacobianFactor::shared_ptr newJacobian = toJacobian(jf->clone());
            newJacobian->setModel(noiseModel::Diagonal::Precisions(newPrecisions));
            hfg->push_back(HessianFactor(*newJacobian));
          }
        }
        else {  // unconstrained Jacobian
          // Convert the original linear factor to Hessian factor
          hfg->push_back(HessianFactor(*graph[factorIndex]));
        }
      }
      else { // If it's not a Jacobian, it should be a hessian factor. Just add!
        hfg->push_back(graph[factorIndex]);
      }

    }
    return hfg;
  }

  /* ************************************************************************* */
  /**
   * Build the dual graph to solve for the Lagrange multipliers.
   *
   * The Lagrangian function is:
   *        L(X,lambdas) = f(X) - \sum_k lambda_k * c_k(X),
   * where the unconstrained part is
   *        f(X) = 0.5*X'*G*X - X'*g + 0.5*f0
   * and the linear equality constraints are
   *        c1(X), c2(X), ..., cm(X)
   *
   * Take the derivative of L wrt X at the solution and set it to 0, we have
   *    \grad f(X) = \sum_k lambda_k * \grad c_k(X)   (*)
   *
   * For each set of rows of (*) corresponding to a variable xi involving in some constraints
   * we have:
   *    \grad f(xi) = \frac{\partial f}{\partial xi}' = \sum_j G_ij*xj - gi
   *    \grad c_k(xi) = \frac{\partial c_k}{\partial xi}'
   *
   * Note: If xi does not involve in any constraint, we have the trivial condition
   * \grad f(Xi) = 0, which should be satisfied as a usual condition for unconstrained variables.
   *
   * So each variable xi involving in some constraints becomes a linear factor A*lambdas - b = 0
   * on the constraints' lambda multipliers, as follows:
   *    - The jacobian term A_k for each lambda_k is \grad c_k(xi)
   *    - The constant term b is \grad f(xi), which can be computed from all unconstrained
   *    Hessian factors connecting to xi: \grad f(xi) = \sum_j G_ij*xj - gi
   */
  GaussianFactorGraph buildDualGraph(const GaussianFactorGraph& graph,
      const VectorValues& x0) const {
    // The dual graph to return
    GaussianFactorGraph dualGraph;

    // For each variable xi involving in some constraint, compute the unconstrained gradient
    // wrt xi from the prebuilt freeHessian graph
    // \grad f(xi) = \frac{\partial f}{\partial xi}' = \sum_j G_ij*xj - gi
    BOOST_FOREACH(const VariableIndex::value_type& xiKey_factors, freeHessianFactorIndex_) {
      Key xiKey = xiKey_factors.first;
      VariableIndex::Factors xiFactors = xiKey_factors.second;

      // Find xi's dim from the first factor on xi
      if (xiFactors.size() == 0) continue;
      GaussianFactor::shared_ptr xiFactor0 = freeHessians_->at(0);
      size_t xiDim = xiFactor0->getDim(xiFactor0->find(xiKey));

      // Compute gradf(xi) = \frac{\partial f}{\partial xi}' = \sum_j G_ij*xj - gi
      Vector gradf_xi = zero(xiDim);
      BOOST_FOREACH(size_t factorIx, xiFactors) {
        HessianFactor::shared_ptr factor = toHessian(freeHessians_->at(factorIx));
        Factor::const_iterator xi = factor->find(xiKey);
        // Sum over Gij*xj for all xj connecting to xi
        for (Factor::const_iterator xj = factor->begin(); xj != factor->end();
            ++xj) {
          // Obtain Gij from the Hessian factor
          // Hessian factor only stores an upper triangular matrix, so be careful when i>j
          Matrix Gij;
          if (xi > xj) {
            Matrix Gji = factor->info(xj, xi);
            Gij = Gji.transpose();
          }
          else {
            Gij = factor->info(xi, xj);
          }
          // Accumulate Gij*xj to gradf
          Vector x0_j = x0.at(*xj);
          gradf_xi += Gij * x0_j;
        }
        // Subtract the linear term gi
        gradf_xi += -factor->linearTerm(xi);
      }

      // Obtain the jacobians for lambda variables from their corresponding constraints
      // gradc_k(xi) = \frac{\partial c_k}{\partial xi}'
      std::vector<std::pair<Key, Matrix> > lambdaTerms; // collection of lambda_k, and gradc_k
      BOOST_FOREACH(size_t factorIndex, fullFactorIndices_[xiKey]) {
        JacobianFactor::shared_ptr factor = toJacobian(graph.at(factorIndex));
        if (!factor || !factor->isConstrained()) continue;
        // Gradient is the transpose of the Jacobian: A_k = gradc_k(xi) = \frac{\partial c_k}{\partial xi}'
        // Each column for each lambda_k corresponds to [the transpose of] each constrained row factor
        Matrix A_k = factor->getA(factor->find(xiKey)).transpose();
        // Deal with mixed sigmas: no information if sigma != 0
        Vector sigmas = factor->get_model()->sigmas();
        for (size_t sigmaIx = 0; sigmaIx<sigmas.size(); ++sigmaIx) {
          // if it's either ineq (sigma<0) or unconstrained (sigma>0)
          // we have no information about it
          if (fabs(sigmas[sigmaIx]) > 1e-9) {
            A_k.col(sigmaIx) = zero(A_k.rows());
          }
        }
        // Use factorIndex as the lambda's key.
        lambdaTerms.push_back(make_pair(factorIndex, A_k));
      }
      // Enforce constrained noise model so lambdas are solved with QR
      // and should exactly satisfy all the equations
      dualGraph.push_back(JacobianFactor(lambdaTerms, gradf_xi,
          noiseModel::Constrained::All(gradf_xi.size())));

      // Add 0 priors on all lambdas to make sure the graph is solvable
      // TODO: Can we do for all lambdas like this, or only for those with no information?
      BOOST_FOREACH(size_t factorIndex, fullFactorIndices_[xiKey]) {
        JacobianFactor::shared_ptr factor = toJacobian(graph.at(factorIndex));
        if (!factor || !factor->isConstrained()) continue;
        size_t dim= factor->get_model()->dim();
        // Use factorIndex as the lambda's key.
        dualGraph.push_back(JacobianFactor(factorIndex, eye(dim), zero(dim)));
      }
    }
    return dualGraph;
  }

  /**
  * Find max lambda element.
  * For active ineq constraints (those that are enforced as eq constraints now
  * in the working set), we want lambda < 0.
  * This is because:
  *     - From the Lagrangian L = f - lambda*c, we know that the constraint force is
  *     (lambda * \grad c) = \grad f, because it cancels out the unconstrained
  *     unconstrained force (-\grad f), which is pulling x in the opposite direction
  *     of \grad f towards the unconstrained minimum point
  *     - We also know that  at the constraint surface \grad c points toward + (>= 0),
  *     while we are solving for - (<=0) constraint
  *     - So, we want the constraint force (lambda * \grad c) to to pull x
  *     towards the opposite direction of \grad c, i.e. towards the area
  *     where the ineq constraint <=0 is satisfied.
  *     - Hence, we want lambda < 0
  */
  std::pair<int, int> findWeakestViolationIneq(const VectorValues& lambdas) const {
    int worstFactorIx = -1, worstSigmaIx = -1;
    // preset the maxLambda to 0.0: if lambda is <= 0.0, the constraint is either
    // inactive or a good ineq constraint, so we don't care!
    double maxLambda = 0.0;
    BOOST_FOREACH(size_t factorIx, constraintIndices_) {
      Vector lambda = lambdas.at(factorIx);
      Vector orgSigmas = toJacobian(graph_.at(factorIx))->get_model()->sigmas();
      for (size_t j = 0; j<orgSigmas.size(); ++j)
        // If it is a BAD active inequality, and lambda is larger than the current max
        if (orgSigmas[j]<0 && lambda[j] > maxLambda) {
          worstFactorIx = factorIx;
          worstSigmaIx = j;
          maxLambda = lambda[j];
        }
    }
    return make_pair(worstFactorIx, worstSigmaIx);
  }

  /**
   * Deactivate or activate an ineq constraint in place
   * Warning: modify in-place to avoid copy/clone
   * @return true if update successful
   */
  bool updateWorkingSetInplace(GaussianFactorGraph& workingGraph,
      int factorIx, int sigmaIx, double newSigma) const {
    if (factorIx < 0 || sigmaIx < 0)
      return false;
    Vector sigmas = toJacobian(workingGraph.at(factorIx))->get_model()->sigmas();
    sigmas[sigmaIx] = newSigma; // removing it from the working set
    toJacobian(workingGraph.at(factorIx))->setModel(true, sigmas);
    return true;
  }

  /**
   * Compute step size alpha for the new solution x' = xk + alpha*p, where alpha \in [0,1]
   * We have to make sure the new solution with alpha satisfies all INACTIVE ineq constraints
   * If some inactive ineq constraints complain about the full step (alpha = 1),
   * we have to adjust alpha to stay within the ineq constraints' feasible regions.
   *
   * For each inactive ineq j:
   *  - We already have: aj'*xk - bj <= 0, since xk satisfies all ineq constraints
   *  - We want: aj'*(xk + alpha*p) - bj <= 0
   *  - If aj'*p <= 0, we have: aj'*(xk + alpha*p) <= aj'*xk <= bj, for all alpha>0
   *  it's good!
   *  - We only care when aj'*p > 0. In this case, we need to choose alpha so that
   *  aj'*xk + alpha*aj'*p - bj <= 0  --> alpha <= (bj - aj'*xk) / (aj'*p)
   *  We want to step as far as possible, so we should choose alpha = (bj - aj'*xk) / (aj'*p)
   *
   * We want the minimum of all those alphas among all inactive ineq.
   *
   *    @return a tuple of (alpha, factorIndex, sigmaIndex) where (factorIndex, sigmaIndex)
   *            is the constraint that has minimum alpha, or (-1,-1) if alpha = 1.
   *            This constraint will be added to the working set and become active
   *            in the next iteration
   */
  boost::tuple<double, int, int> computeStepSize(const GaussianFactorGraph& workingGraph,
      const VectorValues& xk, const VectorValues& p) const {
    static bool debug = true;

    double minAlpha = 1.0;
    int closestFactorIx = -1, closestSigmaIx = -1;
    BOOST_FOREACH(size_t factorIx, constraintIndices_) {
      JacobianFactor::shared_ptr jacobian = toJacobian(workingGraph.at(factorIx));
      Vector sigmas = jacobian->get_model()->sigmas();
      Vector b = jacobian->getb();
      for (size_t s = 0; s<sigmas.size(); ++s) {
        // If it is an inactive inequality, compute alpha and update min
        if (sigmas[s]<0) {
          // Compute aj'*p
          double ajTp = 0.0;
          for (Factor::const_iterator xj = jacobian->begin(); xj != jacobian->end(); ++xj) {
            Vector pj = p.at(*xj);
            Vector aj = jacobian->getA(xj).row(s);
            ajTp += aj.dot(pj);
          }
          if (debug) {
            cout << "s, ajTp: " << s << " " << ajTp << endl;
          }

          // Check if  aj'*p >0. Don't care if it's not.
          if (ajTp<=0) continue;

          // Compute aj'*xk
          double ajTx = 0.0;
          for (Factor::const_iterator xj = jacobian->begin(); xj != jacobian->end(); ++xj) {
            Vector xkj = xk.at(*xj);
            Vector aj = jacobian->getA(xj).row(s);
            ajTx += aj.dot(xkj);
          }
          if (debug) {
            cout << "b[s], ajTx: " << b[s] << " " << ajTx << " " << ajTp << endl;
          }

          // alpha = (bj - aj'*xk) / (aj'*p)
          double alpha = (b[s] - ajTx)/ajTp;
          if (debug) {
            cout << "alpha: " << alpha << endl;
          }

          // We want the minimum of all those max alphas
          if (alpha < minAlpha) {
            closestFactorIx = factorIx;
            closestSigmaIx = s;
            minAlpha = alpha;
          }
        }
      }
    }
    return boost::make_tuple(minAlpha, closestFactorIx, closestSigmaIx);
  }

  /** Iterate 1 step, modify workingGraph and currentSolution in place */
  bool iterateInPlace(GaussianFactorGraph& workingGraph, VectorValues& currentSolution) const {
    static bool debug = true;
    // Obtain the solution from the current working graph
    VectorValues newSolution = workingGraph.optimize();
    if (debug) newSolution.print("New solution:");

    // If we CAN'T move further
    if (newSolution.equals(currentSolution, 1e-5)) {
      // Compute lambda from the dual graph
      GaussianFactorGraph dualGraph = buildDualGraph(workingGraph, newSolution);
      if (debug) dualGraph.print("Dual graph: ");
      VectorValues lambdas = dualGraph.optimize();
      if (debug) lambdas.print("lambdas :");

      int factorIx, sigmaIx;
      boost::tie(factorIx, sigmaIx) = findWeakestViolationIneq(lambdas);

      // Try to disactivate the weakest violated ineq constraints
      // if not successful, i.e. all ineq constraints are satisfied: We have the solution!!
      if (!updateWorkingSetInplace(workingGraph, factorIx, sigmaIx, -1.0))
        return true;
    }
    else {
      // If we CAN make some progress
      // Adapt stepsize if some inactive inequality constraints complain about this move
      double alpha;
      int factorIx, sigmaIx;
      VectorValues p = newSolution - currentSolution;
      boost::tie(alpha, factorIx, sigmaIx) = computeStepSize(workingGraph, currentSolution, p);
      if (debug) {
        cout << "alpha, factorIx, sigmaIx: " << alpha << " " << factorIx << " " << sigmaIx << endl;
      }
      // also add to the working set the one that complains the most
      updateWorkingSetInplace(workingGraph, factorIx, sigmaIx, 0.0);
      // step!
      currentSolution = currentSolution + alpha * p;
    }

    return false;
  }

  VectorValues optimize(const VectorValues& initials) const {
    GaussianFactorGraph workingGraph = graph_.clone();
    VectorValues currentSolution = initials;
    bool converged = false;
    while (!converged) {
      converged = iterateInPlace(workingGraph, currentSolution);
    }
    return currentSolution;
  }

};

/* ************************************************************************* */
// Create test graph according to Forst10book_pg171Ex5
GaussianFactorGraph createTestCase() {
  GaussianFactorGraph graph;

  // Objective functions x1^2 - x1*x2 + x2^2 - 3*x1
  // Note the Hessian encodes:
  //        0.5*x1'*G11*x1 + x1'*G12*x2 + 0.5*x2'*G22*x2 - x1'*g1 - x2'*g2 + 0.5*f
  // Hence, we have G11=2, G12 = -1, g1 = +3, G22 = 2, g2 = 0, f = 0
  graph.push_back(
      HessianFactor(X(1), X(2), 2.0*ones(1, 1), -ones(1, 1), 3.0*ones(1),
          2.0*ones(1, 1), zero(1), 10.0));

  // Inequality constraints
  // Jacobian factors represent Ax-b, ehnce
  // x1 + x2 <= 2 --> x1 + x2 -2 <= 0, --> b=2
  Matrix A1 = (Matrix(4, 1)<<1, -1, 0, 1);
  Matrix A2 = (Matrix(4, 1)<<1, 0, -1, 0);
  Vector b =  (Vector(4)<<2, 0, 0, 1.5);
  // Special constrained noise model denoting <= inequalities with negative sigmas
  noiseModel::Constrained::shared_ptr noise =
      noiseModel::Constrained::MixedSigmas((Vector(4)<<-1, -1, -1, -1));
  graph.push_back(JacobianFactor(X(1), A1, X(2), A2, b, noise));

  return graph;
}

TEST_DISABLED(QPSolver, constraintsAux) {
  GaussianFactorGraph graph = createTestCase();
  QPSolver solver(graph);
  FastVector<size_t> constraintIx = solver.constraintIndices();
  LONGS_EQUAL(1, constraintIx.size());
  LONGS_EQUAL(1, constraintIx[0]);

  VectorValues lambdas;
  lambdas.insert(constraintIx[0], (Vector(4)<< -0.5, 0.0, 0.3, 0.1));
  int factorIx, lambdaIx;
  boost::tie(factorIx, lambdaIx) = solver.findWeakestViolationIneq(lambdas);
  LONGS_EQUAL(1, factorIx);
  LONGS_EQUAL(2, lambdaIx);

  VectorValues lambdas2;
  lambdas2.insert(constraintIx[0], (Vector(4)<< -0.5, 0.0, -0.3, -0.1));
  int factorIx2, lambdaIx2;
  boost::tie(factorIx2, lambdaIx2) = solver.findWeakestViolationIneq(lambdas2);
  LONGS_EQUAL(-1, factorIx2);
  LONGS_EQUAL(-1, lambdaIx2);

  GaussianFactorGraph::shared_ptr freeHessian = solver.freeHessiansOfConstrainedVars();
  GaussianFactorGraph expectedFreeHessian;
  expectedFreeHessian.push_back(
      HessianFactor(X(1), X(2), 2.0 * ones(1, 1), -ones(1, 1), 3.0 * ones(1),
          2.0 * ones(1, 1), zero(1), 1.0));
  EXPECT(expectedFreeHessian.equals(*freeHessian));
}

/* ************************************************************************* */
// Create a simple test graph with one equality constraint
GaussianFactorGraph createEqualityConstrainedTest() {
  GaussianFactorGraph graph;

  // Objective functions x1^2 + x2^2
  // Note the Hessian encodes:
  //        0.5*x1'*G11*x1 + x1'*G12*x2 + 0.5*x2'*G22*x2 - x1'*g1 - x2'*g2 + 0.5*f
  // Hence, we have G11=2, G12 = 0, g1 = 0, G22 = 2, g2 = 0, f = 0
  graph.push_back(
      HessianFactor(X(1), X(2), 2.0*ones(1, 1), zeros(1, 1), zero(1),
          2.0*ones(1, 1), zero(1), 0.0));

  // Equality constraints
  // x1 + x2 = 1 --> x1 + x2 -1 = 0, hence we negate the b vector
  Matrix A1 = (Matrix(1, 1)<<1);
  Matrix A2 = (Matrix(1, 1)<<1);
  Vector b = -(Vector(1)<<1);
  // Special constrained noise model denoting <= inequalities with negative sigmas
  noiseModel::Constrained::shared_ptr noise =
      noiseModel::Constrained::MixedSigmas((Vector(1)<<0.0));
  graph.push_back(JacobianFactor(X(1), A1, X(2), A2, b, noise));

  return graph;
}

TEST(QPSolver, dual) {
  GaussianFactorGraph graph = createEqualityConstrainedTest();

  // Initials values
  VectorValues initials;
  initials.insert(X(1), ones(1));
  initials.insert(X(2), ones(1));

  QPSolver solver(graph);

  GaussianFactorGraph dualGraph = solver.buildDualGraph(graph, initials);
  VectorValues dual = dualGraph.optimize();
  VectorValues expectedDual;
  expectedDual.insert(1, (Vector(1)<<2.0));
  CHECK(assert_equal(expectedDual, dual, 1e-100));
}

/* ************************************************************************* */

TEST(QPSolver, iterate) {
  GaussianFactorGraph graph = createTestCase();
  QPSolver solver(graph);

  GaussianFactorGraph workingGraph = graph.clone();

  VectorValues currentSolution;
  currentSolution.insert(X(1), zeros(1,1));
  currentSolution.insert(X(2), zeros(1,1));

  std::vector<VectorValues> expectedSolutions(3);
  expectedSolutions[0].insert(X(1), (Vector(1) << 4.0/3.0));
  expectedSolutions[0].insert(X(2), (Vector(1) << 2.0/3.0));
  expectedSolutions[1].insert(X(1), (Vector(1) << 1.5));
  expectedSolutions[1].insert(X(2), (Vector(1) << 0.5));
  expectedSolutions[2].insert(X(1), (Vector(1) << 1.5));
  expectedSolutions[2].insert(X(2), (Vector(1) << 0.5));

  bool converged = false;
  int it = 0;
  while (!converged) {
    converged = solver.iterateInPlace(workingGraph, currentSolution);
    CHECK(assert_equal(expectedSolutions[it], currentSolution, 1e-100));
    it++;
  }
}

/* ************************************************************************* */

TEST(QPSolver, optimize) {
  GaussianFactorGraph graph = createTestCase();
  QPSolver solver(graph);
  VectorValues initials;
  initials.insert(X(1), zeros(1,1));
  initials.insert(X(2), zeros(1,1));
  VectorValues solution = solver.optimize(initials);
  VectorValues expectedSolution;
  expectedSolution.insert(X(1), (Vector(1)<< 1.5));
  expectedSolution.insert(X(2), (Vector(1)<< 0.5));
  CHECK(assert_equal(expectedSolution, solution, 1e-100));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

