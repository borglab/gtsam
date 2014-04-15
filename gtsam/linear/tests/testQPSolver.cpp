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

#ifdef __CDT_PARSER__
#undef BOOST_FOREACH
#define BOOST_FOREACH(a, b) for(a; ; )
#endif

class QPSolver {
  const GaussianFactorGraph& graph_;   //!< the original graph, can't be modified!
  GaussianFactorGraph workingGraph_;  //!< working set
  VectorValues currentSolution_;
  FastVector<size_t> constraintIndices_; //!< Indices of constrained factors in the original graph
  GaussianFactorGraph::shared_ptr freeHessians_;
  VariableIndex freeHessianVarIndex_;
  VariableIndex fullFactorIndices_; //!< indices of factors involving each variable.
                                    // gtsam calls it "VariableIndex", but I think FactorIndex
                                    // makes more sense, because it really stores factor indices.

public:
  /// Constructor
  QPSolver(const GaussianFactorGraph& graph, const VectorValues& initials) :
      graph_(graph), workingGraph_(graph.clone()), currentSolution_(initials),
      fullFactorIndices_(graph) {

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
    freeHessianVarIndex_ = VariableIndex(*freeHessians_);
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
  GaussianFactorGraph::shared_ptr buildDualGraph(const VectorValues& x0) const {
    // The dual graph to return
    GaussianFactorGraph::shared_ptr dualGraph = boost::make_shared<GaussianFactorGraph>();

    // For each variable xi involving in some constraint, compute the unconstrained gradient
    // wrt xi from the prebuilt freeHessian graph
    // \grad f(xi) = \frac{\partial f}{\partial xi}' = \sum_j G_ij*xj - gi
    BOOST_FOREACH(const VariableIndex::value_type& xiKey_factors, freeHessianVarIndex_) {
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
        JacobianFactor::shared_ptr factor = toJacobian(workingGraph_.at(factorIndex));
        if (!factor || !factor->isConstrained()) continue;
        // Gradient is the transpose of the Jacobian: A_k = gradc_k(xi) = \frac{\partial c_k}{\partial xi}'
        // Each column for each lambda_k corresponds to [the transpose of] each constrained row factor
        Matrix A_k = factor->getA(factor->find(xiKey)).transpose();
        // Deal with mixed sigmas: no information if sigma != 0
        Vector sigmas = factor->get_model()->sigmas();
        for (size_t sigmaIx = 0; sigmaIx<sigmas.size(); ++sigmaIx) {
          if (fabs(sigmas[sigmaIx]) > 1e-9) {    // if it's either ineq (sigma<0) or unconstrained (sigma>0)
            A_k.col(sigmaIx) = zero(A_k.rows());
          }
        }
        // Use factorIndex as the lambda's key.
        lambdaTerms.push_back(make_pair(factorIndex, A_k));
      }
      // Enforce constrained noise model so lambda is solved with QR and exactly satisfies all the equation
      dualGraph->push_back(JacobianFactor(lambdaTerms, gradf_xi, noiseModel::Constrained::All(gradf_xi.size())));
    }
    return dualGraph;
  }

  /// Find max lambda element
  std::pair<int, int> findMostViolatedIneqConstraint(const VectorValues& lambdas) {
    int worstFactorIx = -1, worstSigmaIx = -1;
    double maxLambda = 0.0;
    BOOST_FOREACH(size_t factorIx, constraintIndices_) {
      Vector lambda = lambdas.at(factorIx);
      Vector orgSigmas = toJacobian(graph_.at(factorIx))->get_model()->sigmas();
      for (size_t j = 0; j<lambda.size(); ++j)
        // If it is an active inequality, and lambda is larger than the current max
        if (orgSigmas[j]<0 && lambda[j]>maxLambda) {
          worstFactorIx = factorIx;
          worstSigmaIx = j;
          maxLambda = lambda[j];
        }
    }
    return make_pair(worstFactorIx, worstSigmaIx);
  }

  /** Iterate 1 step */
//  bool iterate() {
//    // Obtain the solution from the current working graph
//    VectorValues newSolution = workingGraph_.optimize();
//
//    // If we CAN'T move further
//    if (newSolution == currentSolution) {
//      // Compute lambda from the dual graph
//      GaussianFactorGraph dualGraph = buildDualGraph(graph, newSolution);
//      VectorValues lambdas = dualGraph.optimize();
//
//      int factorIx, sigmaIx;
//      boost::tie(factorIx, sigmaIx) = findMostViolatedIneqConstraint(lambdas);
//      // If all constraints are satisfied: We have found the solution!!
//      if (factorIx < 0) {
//        return true;
//      }
//      else {
//        // If some constraints are violated!
//        Vector sigmas = toJacobian(workingGraph.at(factorIx))->get_model()->sigmas();
//        sigmas[sigmaIx] = 0.0;
//        toJacobian()->setModel(true, sigmas);
//        // No need to update the currentSolution, since we haven't moved anywhere
//      }
//    }
//    else {
//      // If we CAN make some progress
//      // Adapt stepsize if some inactive inequality constraints complain about this move
//      // also add to the working set the one that complains the most
//      VectorValues alpha = updateWorkingSetInplace();
//      currentSolution_ = (1 - alpha) * currentSolution_ + alpha * newSolution;
//    }
//
//    return false;
//  }
//
//  VectorValues optimize() const {
//    bool converged = false;
//    while (!converged) {
//      converged = iterate();
//    }
//  }

};

/* ************************************************************************* */
// Create test graph according to Forst10book_pg171Ex5
std::pair<GaussianFactorGraph, VectorValues> createTestCase() {
  GaussianFactorGraph graph;

  // Objective functions x1^2 - x1*x2 + x2^2 - 3*x1
  // Note the Hessian encodes:
  //        0.5*x1'*G11*x1 + x1'*G12*x2 + 0.5*x2'*G22*x2 - x1'*g1 - x2'*g2 + 0.5*f
  // Hence, we have G11=2, G12 = -1, g1 = +3, G22 = 2, g2 = 0, f = 0
  graph.push_back(
      HessianFactor(X(1), X(2), 2.0*ones(1, 1), -ones(1, 1), 3.0*ones(1),
          2.0*ones(1, 1), zero(1), 1.0));

  // Inequality constraints
  // x1 + x2 <= 2 --> x1 + x2 -2 <= 0, hence we negate the b vector
  Matrix A1 = (Matrix(4, 1)<<1, -1, 0, 1);
  Matrix A2 = (Matrix(4, 1)<<1, 0, -1, 0);
  Vector b = -(Vector(4)<<2, 0, 0, 1.5);
  // Special constrained noise model denoting <= inequalities with negative sigmas
  noiseModel::Constrained::shared_ptr noise =
      noiseModel::Constrained::MixedSigmas((Vector(4)<<-1, -1, -1, -1));
  graph.push_back(JacobianFactor(X(1), A1, X(2), A2, b, noise));

  // Initials values
  VectorValues initials;
  initials.insert(X(1), ones(1));
  initials.insert(X(2), ones(1));

  return make_pair(graph, initials);
}

TEST(QPSolver, constraintsAux) {
  GaussianFactorGraph graph;
  VectorValues initials;
  boost::tie(graph, initials)= createTestCase();
  QPSolver solver(graph, initials);
  FastVector<size_t> constraintIx = solver.constraintIndices();
  LONGS_EQUAL(1, constraintIx.size());
  LONGS_EQUAL(1, constraintIx[0]);

  VectorValues lambdas;
  lambdas.insert(constraintIx[0], (Vector(4)<< -0.5, 0.0, 0.3, 0.1));
  int factorIx, lambdaIx;
  boost::tie(factorIx, lambdaIx) = solver.findMostViolatedIneqConstraint(lambdas);
  LONGS_EQUAL(1, factorIx);
  LONGS_EQUAL(2, lambdaIx);

  VectorValues lambdas2;
  lambdas2.insert(constraintIx[0], (Vector(4)<< -0.5, 0.0, -0.3, -0.1));
  int factorIx2, lambdaIx2;
  boost::tie(factorIx2, lambdaIx2) = solver.findMostViolatedIneqConstraint(lambdas2);
  LONGS_EQUAL(-1, factorIx2);
  LONGS_EQUAL(-1, lambdaIx2);

  GaussianFactorGraph::shared_ptr freeHessian = solver.freeHessiansOfConstrainedVars();
  GaussianFactorGraph expectedFreeHessian;
  expectedFreeHessian.push_back(
      HessianFactor(X(1), X(2), 2.0 * ones(1, 1), -ones(1, 1), 3.0 * ones(1),
          2.0 * ones(1, 1), zero(1), 1.0));
  EXPECT(expectedFreeHessian.equals(*freeHessian));

  GaussianFactorGraph::shared_ptr dualGraph = solver.buildDualGraph(initials);
  dualGraph->print("Dual graph: ");
  VectorValues dual = dualGraph->optimize();
  dual.print("Dual: ");
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

