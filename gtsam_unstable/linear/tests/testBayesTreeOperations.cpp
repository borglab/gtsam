/**
 * @file testLinearTools.cpp
 *
 * @brief 
 *
 * @date Aug 27, 2012
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam_unstable/linear/bayesTreeOperations.h>
#include <gtsam/linear/GaussianMultifrontalSolver.h>

#include <tests/smallExample.h>

using namespace gtsam;

SharedDiagonal model2 = noiseModel::Diagonal::Sigmas(ones(2));
SharedDiagonal model4 = noiseModel::Diagonal::Sigmas(ones(4));
SharedDiagonal model6 = noiseModel::Diagonal::Sigmas(ones(6));

using namespace std;

using symbol_shorthand::X;
using symbol_shorthand::L;

static const double tol = 1e-4;

/* ************************************************************************* */
TEST( testBayesTreeOperations, splitFactor1 ) {

  // Build upper-triangular system
  JacobianFactor initFactor(
       0,Matrix_(4, 2,
           1.0, 2.0,
           0.0, 3.0,
           0.0, 0.0,
           0.0, 0.0),
       1,Matrix_(4, 2,
           1.0, 2.0,
           9.0, 3.0,
           6.0, 8.0,
           0.0, 7.0),
       Vector_(4, 0.1, 0.2, 0.3, 0.4),
       model4);

  GaussianFactorGraph actSplit = splitFactor(initFactor.clone());
  GaussianFactorGraph expSplit;

  expSplit.add(
       0,Matrix_(2, 2,
           1.0, 2.0,
           0.0, 3.0),
       1,Matrix_(2, 2,
           1.0, 2.0,
           9.0, 3.0),
       Vector_(2, 0.1, 0.2),
       model2);
  expSplit.add(
       1,Matrix_(2, 2,
           6.0, 8.0,
           0.0, 7.0),
       Vector_(2, 0.3, 0.4),
       model2);

  EXPECT(assert_equal(expSplit, actSplit));
}

/* ************************************************************************* */
TEST( testBayesTreeOperations, splitFactor2 ) {

  // Build upper-triangular system
  JacobianFactor initFactor(
       0,Matrix_(6, 2,
           1.0, 2.0,
           0.0, 3.0,
           0.0, 0.0,
           0.0, 0.0,
           0.0, 0.0,
           0.0, 0.0),
       1,Matrix_(6, 2,
           1.0, 2.0,
           9.0, 3.0,
           6.0, 8.0,
           0.0, 7.0,
           0.0, 0.0,
           0.0, 0.0),
       2,Matrix_(6, 2,
           1.1, 2.2,
           9.1, 3.2,
           6.1, 8.2,
           0.1, 7.2,
           0.1, 3.2,
           0.0, 1.2),
       Vector_(6, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6),
       model6);

  GaussianFactorGraph actSplit = splitFactor(initFactor.clone());
  GaussianFactorGraph expSplit;

  expSplit.add(
       0,Matrix_(2, 2,
           1.0, 2.0,
           0.0, 3.0),
       1,Matrix_(2, 2,
           1.0, 2.0,
           9.0, 3.0),
       2,Matrix_(2, 2,
           1.1, 2.2,
           9.1, 3.2),
       Vector_(2, 0.1, 0.2),
       model2);
  expSplit.add(
       1,Matrix_(2, 2,
           6.0, 8.0,
           0.0, 7.0),
       2,Matrix_(2, 2,
           6.1, 8.2,
           0.1, 7.2),
       Vector_(2, 0.3, 0.4),
       model2);
  expSplit.add(
      2,Matrix_(2, 2,
           0.1, 3.2,
           0.0, 1.2),
       Vector_(2, 0.5, 0.6),
       model2);

  EXPECT(assert_equal(expSplit, actSplit));
}

/* ************************************************************************* */
TEST( testBayesTreeOperations, splitFactor3 ) {

  // Build upper-triangular system
  JacobianFactor initFactor(
       0,Matrix_(4, 2,
           1.0, 2.0,
           0.0, 3.0,
           0.0, 0.0,
           0.0, 0.0),
       1,Matrix_(4, 2,
           1.0, 2.0,
           9.0, 3.0,
           6.0, 8.0,
           0.0, 7.0),
       2,Matrix_(4, 2,
           1.1, 2.2,
           9.1, 3.2,
           6.1, 8.2,
           0.1, 7.2),
       Vector_(4, 0.1, 0.2, 0.3, 0.4),
       model4);

  GaussianFactorGraph actSplit = splitFactor(initFactor.clone());
  GaussianFactorGraph expSplit;

  expSplit.add(
       0,Matrix_(2, 2,
           1.0, 2.0,
           0.0, 3.0),
       1,Matrix_(2, 2,
           1.0, 2.0,
           9.0, 3.0),
       2,Matrix_(2, 2,
           1.1, 2.2,
           9.1, 3.2),
       Vector_(2, 0.1, 0.2),
       model2);
  expSplit.add(
       1,Matrix_(2, 2,
           6.0, 8.0,
           0.0, 7.0),
       2,Matrix_(2, 2,
           6.1, 8.2,
           0.1, 7.2),
       Vector_(2, 0.3, 0.4),
       model2);

  EXPECT(assert_equal(expSplit, actSplit));
}

/* ************************************************************************* */
// Some numbers that should be consistent among all smoother tests

//static double sigmax1 = 0.786153, /*sigmax2 = 1.0/1.47292,*/ sigmax3 = 0.671512, sigmax4 =
//    0.669534 /*, sigmax5 = sigmax3, sigmax6 = sigmax2*/, sigmax7 = sigmax1;

/* ************************************************************************* */
TEST( testBayesTreeOperations, liquefy ) {
  using namespace example;

  // Create smoother with 7 nodes
  Ordering ordering;
  ordering += X(1),X(3),X(5),X(7),X(2),X(6),X(4);
  GaussianFactorGraph smoother = createSmoother(7, ordering).first;

  // Create the Bayes tree
  GaussianBayesTree bayesTree = *GaussianMultifrontalSolver(smoother).eliminate();
//  bayesTree.print("Full tree");

  SharedDiagonal unit6 = noiseModel::Diagonal::Sigmas(Vector_(ones(6)));
  SharedDiagonal unit4 = noiseModel::Diagonal::Sigmas(Vector_(ones(4)));
  SharedDiagonal unit2 = noiseModel::Diagonal::Sigmas(Vector_(ones(2)));

  // Liquefy the tree back into a graph
  {
    GaussianFactorGraph actGraph = liquefy(bayesTree, false); // Doesn't split conditionals
    GaussianFactorGraph expGraph;

    Matrix A12 = Matrix_(6, 2,
        1.73205081,          0.0,
               0.0,   1.73205081,
               0.0,          0.0,
               0.0,          0.0,
               0.0,          0.0,
               0.0,          0.0);

    Matrix A15 = Matrix_(6, 2,
       -0.577350269,          0.0,
                0.0, -0.577350269,
         1.47196014,          0.0,
                0.0,   1.47196014,
                0.0,          0.0,
                0.0,          0.0);

    Matrix A16 = Matrix_(6, 2,
      -0.577350269,          0.0,
               0.0, -0.577350269,
      -0.226455407,          0.0,
               0.0, -0.226455407,
        1.49357599,          0.0,
               0.0,   1.49357599);
    expGraph.add(2, A12, 5, A15, 6, A16, zeros(6,1), unit6);

    Matrix A21 = Matrix_(4, 2,
        1.73205081,          0.0,
               0.0,   1.73205081,
               0.0,          0.0,
               0.0,          0.0);

    Matrix A24 = Matrix_(4, 2,
      -0.577350269,          0.0,
               0.0, -0.577350269,
        1.47196014,          0.0,
               0.0,   1.47196014);

    Matrix A26 = Matrix_(4, 2,
      -0.577350269,          0.0,
               0.0, -0.577350269,
      -0.226455407,          0.0,
               0.0, -0.226455407);

    expGraph.add(1, A21, 4, A24, 6, A26, zeros(4,1), unit4);

    Matrix A30 = Matrix_(2, 2,
        1.41421356,          0.0,
               0.0,   1.41421356);

    Matrix A34 = Matrix_(2, 2,
      -0.707106781,          0.0,
               0.0, -0.707106781);

    expGraph.add(0, A30, 4, A34, zeros(2,1), unit2);

    Matrix A43 = Matrix_(2, 2,
        1.41421356,          0.0,
               0.0,   1.41421356);
    Matrix A45 = Matrix_(2, 2,
      -0.707106781,          0.0,
               0.0, -0.707106781);

    expGraph.add(3, A43, 5, A45, zeros(2,1), unit2);

    EXPECT(assert_equal(expGraph, actGraph, tol));
  }

  // Liquefy the tree back into a graph, splitting factors
  {
    GaussianFactorGraph actGraph = liquefy(bayesTree, true);
    GaussianFactorGraph expGraph;

    // Conditional 1
    {
      Matrix A12 = Matrix_(2, 2,
          1.73205081,          0.0,
                 0.0,   1.73205081);

      Matrix A15 = Matrix_(2, 2,
         -0.577350269,          0.0,
                  0.0, -0.577350269);

      Matrix A16 = Matrix_(2, 2,
        -0.577350269,          0.0,
                 0.0, -0.577350269);
      expGraph.add(2, A12, 5, A15, 6, A16, zeros(2,1), unit2);
    }

    {
      Matrix A15 = Matrix_(2, 2,
           1.47196014,          0.0,
                  0.0,   1.47196014);

      Matrix A16 = Matrix_(2, 2,
        -0.226455407,          0.0,
                 0.0, -0.226455407);
      expGraph.add(5, A15, 6, A16, zeros(2,1), unit2);
    }

    {
      Matrix A16 = Matrix_(2, 2,
          1.49357599,          0.0,
                 0.0,   1.49357599);
      expGraph.add(6, A16, zeros(2,1), unit2);
    }

    // Conditional 2
    {
      Matrix A21 = Matrix_(2, 2,
          1.73205081,          0.0,
                 0.0,   1.73205081);

      Matrix A24 = Matrix_(2, 2,
        -0.577350269,          0.0,
                 0.0, -0.577350269);

      Matrix A26 = Matrix_(2, 2,
        -0.577350269,          0.0,
                 0.0, -0.577350269);

      expGraph.add(1, A21, 4, A24, 6, A26, zeros(2,1), unit2);
    }

    {
      Matrix A24 = Matrix_(2, 2,
          1.47196014,          0.0,
                 0.0,   1.47196014);

      Matrix A26 = Matrix_(2, 2,
        -0.226455407,          0.0,
                 0.0, -0.226455407);

      expGraph.add(4, A24, 6, A26, zeros(2,1), unit2);
    }

    // Conditional 3
    Matrix A30 = Matrix_(2, 2,
        1.41421356,          0.0,
               0.0,   1.41421356);

    Matrix A34 = Matrix_(2, 2,
      -0.707106781,          0.0,
               0.0, -0.707106781);

    expGraph.add(0, A30, 4, A34, zeros(2,1), unit2);

    // Conditional 4
    Matrix A43 = Matrix_(2, 2,
        1.41421356,          0.0,
               0.0,   1.41421356);
    Matrix A45 = Matrix_(2, 2,
      -0.707106781,          0.0,
               0.0, -0.707106781);

    expGraph.add(3, A43, 5, A45, zeros(2,1), unit2);

    EXPECT(assert_equal(expGraph, actGraph, tol));
  }
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
