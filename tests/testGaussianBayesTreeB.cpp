/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testGaussianISAM.cpp
 * @brief   Unit tests for GaussianISAM
 * @author  Michael Kaess
 */

#include <tests/smallExample.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianDensity.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/geometry/Rot2.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace example;

using symbol_shorthand::X;
using symbol_shorthand::L;

/* ************************************************************************* */
// Some numbers that should be consistent among all smoother tests

static double sigmax1 = 0.786153, /*sigmax2 = 1.0/1.47292,*/ sigmax3 = 0.671512, sigmax4 =
    0.669534 /*, sigmax5 = sigmax3, sigmax6 = sigmax2*/, sigmax7 = sigmax1;

static const double tol = 1e-4;

/* ************************************************************************* *
 Bayes tree for smoother with "natural" ordering:
C1 x6 x7
C2   x5 : x6
C3     x4 : x5
C4       x3 : x4
C5         x2 : x3
C6           x1 : x2
**************************************************************************** */
TEST( GaussianBayesTree, linear_smoother_shortcuts )
{
  // Create smoother with 7 nodes
  GaussianFactorGraph smoother = createSmoother(7);

  GaussianBayesTree bayesTree = *smoother.eliminateMultifrontal();

  // Create the Bayes tree
  LONGS_EQUAL(6, (long)bayesTree.size());

  // Check the conditional P(Root|Root)
  GaussianBayesNet empty;
  GaussianBayesTree::sharedClique R = bayesTree.roots().front();
  GaussianBayesNet actual1 = R->shortcut(R);
  EXPECT(assert_equal(empty,actual1,tol));

  // Check the conditional P(C2|Root)
  GaussianBayesTree::sharedClique C2 = bayesTree[X(5)];
  GaussianBayesNet actual2 = C2->shortcut(R);
  EXPECT(assert_equal(empty,actual2,tol));

  // Check the conditional P(C3|Root)
  double sigma3 = 0.61808;
  Matrix A56 = (Matrix(2,2) << -0.382022,0.,0.,-0.382022).finished();
  GaussianBayesNet expected3;
  expected3.emplace_shared<GaussianConditional>(X(5), Z_2x1, I_2x2/sigma3, X(6), A56/sigma3);
  GaussianBayesTree::sharedClique C3 = bayesTree[X(4)];
  GaussianBayesNet actual3 = C3->shortcut(R);
  EXPECT(assert_equal(expected3,actual3,tol));

  // Check the conditional P(C4|Root)
  double sigma4 = 0.661968;
  Matrix A46 = (Matrix(2,2) << -0.146067,0.,0.,-0.146067).finished();
  GaussianBayesNet expected4;
  expected4.emplace_shared<GaussianConditional>(X(4), Z_2x1, I_2x2/sigma4, X(6), A46/sigma4);
  GaussianBayesTree::sharedClique C4 = bayesTree[X(3)];
  GaussianBayesNet actual4 = C4->shortcut(R);
  EXPECT(assert_equal(expected4,actual4,tol));
}

/* ************************************************************************* *
 Bayes tree for smoother with "nested dissection" ordering:

   Node[x1] P(x1 | x2)
   Node[x3] P(x3 | x2 x4)
   Node[x5] P(x5 | x4 x6)
   Node[x7] P(x7 | x6)
   Node[x2] P(x2 | x4)
   Node[x6] P(x6 | x4)
   Node[x4] P(x4)

 becomes

   C1     x5 x6 x4
   C2      x3 x2 : x4
   C3        x1 : x2
   C4      x7 : x6

************************************************************************* */
TEST(GaussianBayesTree, balanced_smoother_marginals) {
  // Create smoother with 7 nodes
  GaussianFactorGraph smoother = createSmoother(7);

  // Create the Bayes tree
  const Ordering ordering{X(1), X(3), X(5), X(7), X(2), X(6), X(4)};
  GaussianBayesTree bayesTree = *smoother.eliminateMultifrontal(ordering);

  VectorValues actualSolution = bayesTree.optimize();
  VectorValues expectedSolution = VectorValues::Zero(actualSolution);
  EXPECT(assert_equal(expectedSolution, actualSolution, tol));

  LONGS_EQUAL(4, bayesTree.size());

  double tol = 1e-5;

  // Check marginal on x1
  JacobianFactor actual1 = *bayesTree.marginalFactor(X(1));
  Matrix expectedCovX1 = I_2x2 * (sigmax1 * sigmax1);
  auto m = bayesTree.marginalFactor(X(1), EliminateCholesky);
  Matrix actualCovarianceX1 = m->information().inverse();
  EXPECT(assert_equal(expectedCovX1, actualCovarianceX1, tol));

  // Check marginal on x2
  double sigmax2 = 0.68712938;  // FIXME: this should be corrected analytically
  JacobianFactor actual2 = *bayesTree.marginalFactor(X(2));
  Matrix expectedCovX2 = I_2x2 * (sigmax2 * sigmax2);
  EXPECT(assert_equal(expectedCovX2, actual2.information().inverse(), tol));

  // Check marginal on x3
  JacobianFactor actual3 = *bayesTree.marginalFactor(X(3));
  Matrix expectedCovX3 = I_2x2 * (sigmax3 * sigmax3);
  EXPECT(assert_equal(expectedCovX3, actual3.information().inverse(), tol));

  // Check marginal on x4
  JacobianFactor actual4 = *bayesTree.marginalFactor(X(4));
  Matrix expectedCovX4 = I_2x2 * (sigmax4 * sigmax4);
  EXPECT(assert_equal(expectedCovX4, actual4.information().inverse(), tol));

  // Check marginal on x7 (should be equal to x1)
  JacobianFactor actual7 = *bayesTree.marginalFactor(X(7));
  Matrix expectedCovX7 = I_2x2 * (sigmax7 * sigmax7);
  EXPECT(assert_equal(expectedCovX7, actual7.information().inverse(), tol));
}

/* ************************************************************************* */
TEST( GaussianBayesTree, balanced_smoother_shortcuts )
{
  // Create smoother with 7 nodes
  GaussianFactorGraph smoother = createSmoother(7);

  // Create the Bayes tree
  const Ordering ordering{X(1), X(3), X(5), X(7), X(2), X(6), X(4)};
  GaussianBayesTree bayesTree = *smoother.eliminateMultifrontal(ordering);

  // Check the conditional P(Root|Root)
  GaussianBayesNet empty;
  GaussianBayesTree::sharedClique R = bayesTree.roots().front();
  GaussianBayesNet actual1 = R->shortcut(R);
  EXPECT(assert_equal(empty,actual1,tol));

  // Check the conditional P(C2|Root)
  GaussianBayesTree::sharedClique C2 = bayesTree[X(3)];
  GaussianBayesNet actual2 = C2->shortcut(R);
  EXPECT(assert_equal(empty,actual2,tol));

  // Check the conditional P(C3|Root), which should be equal to P(x2|x4)
  /** TODO: Note for multifrontal conditional:
   * p_x2_x4 is now an element conditional of the multifrontal conditional bayesTree[ordering[X(2)]]->conditional()
   * We don't know yet how to take it out.
   */
//  GaussianConditional::shared_ptr p_x2_x4 = bayesTree[ordering[X(2)]]->conditional();
//  p_x2_x4->print("Conditional p_x2_x4: ");
//  GaussianBayesNet expected3(p_x2_x4);
//  GaussianISAM::sharedClique C3 = isamTree[ordering[X(1)]];
//  GaussianBayesNet actual3 = GaussianISAM::shortcut(C3,R);
//  EXPECT(assert_equal(expected3,actual3,tol));
}

///* ************************************************************************* */
//TEST( BayesTree, balanced_smoother_clique_marginals )
//{
//  // Create smoother with 7 nodes
//  const Ordering ordering{X(1),X(3),X(5),X(7),X(2),X(6),X(4)};
//  GaussianFactorGraph smoother = createSmoother(7, ordering).first;
//
//  // Create the Bayes tree
//  GaussianBayesNet chordalBayesNet = *GaussianSequentialSolver(smoother).eliminate();
//  GaussianISAM bayesTree(chordalBayesNet);
//
//  // Check the clique marginal P(C3)
//  double sigmax2_alt = 1/1.45533; // THIS NEEDS TO BE CHECKED!
//  GaussianBayesNet expected = simpleGaussian(ordering[X(2)],Z_2x1,sigmax2_alt);
//  push_front(expected,ordering[X(1)], Z_2x1, eye(2)*sqrt(2), ordering[X(2)], -eye(2)*sqrt(2)/2, ones(2));
//  GaussianISAM::sharedClique R = bayesTree.root(), C3 = bayesTree[ordering[X(1)]];
//  GaussianFactorGraph marginal = C3->marginal(R);
//  GaussianVariableIndex varIndex(marginal);
//  Permutation toFront(Permutation::PullToFront(C3->keys(), varIndex.size()));
//  Permutation toFrontInverse(*toFront.inverse());
//  varIndex.permute(toFront);
//  for(const GaussianFactor::shared_ptr& factor: marginal) {
//    factor->permuteWithInverse(toFrontInverse); }
//  GaussianBayesNet actual = *inference::EliminateUntil(marginal, C3->keys().size(), varIndex);
//  actual.permuteWithInverse(toFront);
//  EXPECT(assert_equal(expected,actual,tol));
//}

/* ************************************************************************* */
TEST( GaussianBayesTree, balanced_smoother_joint )
{
  // Create smoother with 7 nodes
  const Ordering ordering{X(1), X(3), X(5), X(7), X(2), X(6), X(4)};
  GaussianFactorGraph smoother = createSmoother(7);

  // Create the Bayes tree, expected to look like:
  //   x5 x6 x4
  //     x3 x2 : x4
  //       x1 : x2
  //     x7 : x6
  GaussianBayesTree bayesTree = *smoother.eliminateMultifrontal(ordering);

  // Conditional density elements reused by both tests
  const Matrix I = I_2x2, A = -0.00429185*I;

  // Check the joint density P(x1,x7) factored as P(x1|x7)P(x7)
  GaussianBayesNet expected1;
    // Why does the sign get flipped on the prior?
  expected1.emplace_shared<GaussianConditional>(X(1), Z_2x1, I/sigmax7, X(7), A/sigmax7);
  expected1.emplace_shared<GaussianConditional>(X(7), Z_2x1, -1*I/sigmax7);
  GaussianBayesNet actual1 = *bayesTree.jointBayesNet(X(1),X(7));
  EXPECT(assert_equal(expected1, actual1, tol));

  //  // Check the joint density P(x7,x1) factored as P(x7|x1)P(x1)
  //  GaussianBayesNet expected2;
  //  GaussianConditional::shared_ptr
  //      parent2(new GaussianConditional(X(1), Z_2x1, -1*I/sigmax1, ones(2)));
  //    expected2.push_front(parent2);
  //  push_front(expected2,X(7), Z_2x1, I/sigmax1, X(1), A/sigmax1, sigma);
  //  GaussianBayesNet actual2 = *bayesTree.jointBayesNet(X(7),X(1));
  //  EXPECT(assert_equal(expected2,actual2,tol));

  // Check the joint density P(x1,x4), i.e. with a root variable
  double sig14 = 0.784465;
  Matrix A14 = -0.0769231*I;
  GaussianBayesNet expected3;
  expected3.emplace_shared<GaussianConditional>(X(1), Z_2x1, I/sig14, X(4), A14/sig14);
  expected3.emplace_shared<GaussianConditional>(X(4), Z_2x1, I/sigmax4);
  GaussianBayesNet actual3 = *bayesTree.jointBayesNet(X(1),X(4));
  EXPECT(assert_equal(expected3,actual3,tol));

  //  // Check the joint density P(x4,x1), i.e. with a root variable, factored the other way
  //  GaussianBayesNet expected4;
  //  GaussianConditional::shared_ptr
  //      parent4(new GaussianConditional(X(1), Z_2x1, -1.0*I/sigmax1, ones(2)));
  //    expected4.push_front(parent4);
  //  double sig41 = 0.668096;
  //  Matrix A41 = -0.055794*I;
  //  push_front(expected4,X(4), Z_2x1, I/sig41, X(1), A41/sig41, sigma);
  //  GaussianBayesNet actual4 = *bayesTree.jointBayesNet(X(4),X(1));
  //  EXPECT(assert_equal(expected4,actual4,tol));
}

/* ************************************************************************* */
TEST(GaussianBayesTree, shortcut_overlapping_separator)
{
  // Test computing shortcuts when the separator overlaps.  This previously
  // would have highlighted a problem where information was duplicated.

  // Create factor graph:
  // f(1,2,5)
  // f(3,4,5)
  // f(5,6)
  // f(6,7)
  GaussianFactorGraph fg;
  noiseModel::Diagonal::shared_ptr model = noiseModel::Unit::Create(1);
  fg.add(1, (Matrix(1, 1) <<  1.0).finished(), 3, (Matrix(1, 1) <<  2.0).finished(), 5, (Matrix(1, 1) <<  3.0).finished(), (Vector(1) << 4.0).finished(), model);
  fg.add(1, (Matrix(1, 1) <<  5.0).finished(), (Vector(1) << 6.0).finished(), model);
  fg.add(2, (Matrix(1, 1) <<  7.0).finished(), 4, (Matrix(1, 1) <<  8.0).finished(), 5, (Matrix(1, 1) <<  9.0).finished(), (Vector(1) << 10.0).finished(), model);
  fg.add(2, (Matrix(1, 1) <<  11.0).finished(), (Vector(1) << 12.0).finished(), model);
  fg.add(5, (Matrix(1, 1) <<  13.0).finished(), 6, (Matrix(1, 1) <<  14.0).finished(), (Vector(1) << 15.0).finished(), model);
  fg.add(6, (Matrix(1, 1) <<  17.0).finished(), 7, (Matrix(1, 1) <<  18.0).finished(), (Vector(1) << 19.0).finished(), model);
  fg.add(7, (Matrix(1, 1) <<  20.0).finished(), (Vector(1) << 21.0).finished(), model);

  // Eliminate into BayesTree
  // c(6,7)
  // c(5|6)
  //   c(1,2|5)
  //   c(3,4|5)
  Ordering ordering(fg.keys());
  GaussianBayesTree bt = *fg.eliminateMultifrontal(ordering); // eliminate in increasing key order, fg.keys() is sorted.

  GaussianFactorGraph joint = *bt.joint(1,2, EliminateQR);

  Matrix expectedJointJ = (Matrix(2,3) <<
    5, 0, 6,
    0, -11, -12
    ).finished();

  Matrix actualJointJ = joint.augmentedJacobian();

  // PR 315: sign of rows in joint are immaterial
  if (signbit(expectedJointJ(0, 2)) != signbit(actualJointJ(0, 2)))
    expectedJointJ.row(0) = -expectedJointJ.row(0);

  if (signbit(expectedJointJ(1, 2)) != signbit(actualJointJ(1, 2)))
    expectedJointJ.row(1) = -expectedJointJ.row(1);

  EXPECT(assert_equal(expectedJointJ, actualJointJ));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
