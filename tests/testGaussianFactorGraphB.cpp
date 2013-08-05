/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testGaussianFactorGraphB.cpp
 *  @brief  Unit tests for Linear Factor Graph
 *  @author Christian Potthast
 **/

#include <tests/smallExample.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/inference/SymbolicFactorGraph.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign/std/set.hpp> // for operator +=
#include <boost/assign/std/vector.hpp> // for operator +=
using namespace boost::assign;

#include <string.h>
#include <iostream>

using namespace std;
using namespace gtsam;
using namespace example;

double tol=1e-5;

using symbol_shorthand::X;
using symbol_shorthand::L;

/* ************************************************************************* */
TEST( GaussianFactorGraphOrdered, equals ) {

  OrderingOrdered ordering; ordering += X(1),X(2),L(1);
  GaussianFactorGraphOrdered fg = createGaussianFactorGraph(ordering);
  GaussianFactorGraphOrdered fg2 = createGaussianFactorGraph(ordering);
  EXPECT(fg.equals(fg2));
}

/* ************************************************************************* */
TEST( GaussianFactorGraphOrdered, error ) {
  OrderingOrdered ordering; ordering += X(1),X(2),L(1);
  GaussianFactorGraphOrdered fg = createGaussianFactorGraph(ordering);
  VectorValuesOrdered cfg = createZeroDelta(ordering);

  // note the error is the same as in testNonlinearFactorGraph as a
  // zero delta config in the linear graph is equivalent to noisy in
  // non-linear, which is really linear under the hood
  double actual = fg.error(cfg);
  DOUBLES_EQUAL( 5.625, actual, 1e-9 );
}

/* ************************************************************************* */
TEST( GaussianFactorGraphOrdered, eliminateOne_x1 )
{
  OrderingOrdered ordering; ordering += X(1),L(1),X(2);
  GaussianFactorGraphOrdered fg = createGaussianFactorGraph(ordering);

  GaussianConditionalOrdered::shared_ptr conditional;
  GaussianFactorGraphOrdered remaining;
  boost::tie(conditional,remaining) = fg.eliminateOne(0, EliminateQROrdered);

  // create expected Conditional Gaussian
  Matrix I = 15*eye(2), R11 = I, S12 = -0.111111*I, S13 = -0.444444*I;
  Vector d = Vector_(2, -0.133333, -0.0222222), sigma = ones(2);
  GaussianConditionalOrdered expected(ordering[X(1)],15*d,R11,ordering[L(1)],S12,ordering[X(2)],S13,sigma);

  EXPECT(assert_equal(expected,*conditional,tol));
}

/* ************************************************************************* */
TEST( GaussianFactorGraphOrdered, eliminateOne_x2 )
{
  OrderingOrdered ordering; ordering += X(2),L(1),X(1);
  GaussianFactorGraphOrdered fg = createGaussianFactorGraph(ordering);
  GaussianConditionalOrdered::shared_ptr actual = fg.eliminateOne(0, EliminateQROrdered).first;

  // create expected Conditional Gaussian
  double sig = 0.0894427;
  Matrix I = eye(2)/sig, R11 = I, S12 = -0.2*I, S13 = -0.8*I;
  Vector d = Vector_(2, 0.2, -0.14)/sig, sigma = ones(2);
  GaussianConditionalOrdered expected(ordering[X(2)],d,R11,ordering[L(1)],S12,ordering[X(1)],S13,sigma);

  EXPECT(assert_equal(expected,*actual,tol));
}

/* ************************************************************************* */
TEST( GaussianFactorGraphOrdered, eliminateOne_l1 )
{
  OrderingOrdered ordering; ordering += L(1),X(1),X(2);
  GaussianFactorGraphOrdered fg = createGaussianFactorGraph(ordering);
  GaussianConditionalOrdered::shared_ptr actual = fg.eliminateOne(0, EliminateQROrdered).first;

  // create expected Conditional Gaussian
  double sig = sqrt(2.0)/10.;
  Matrix I = eye(2)/sig, R11 = I, S12 = -0.5*I, S13 = -0.5*I;
  Vector d = Vector_(2, -0.1, 0.25)/sig, sigma = ones(2);
  GaussianConditionalOrdered expected(ordering[L(1)],d,R11,ordering[X(1)],S12,ordering[X(2)],S13,sigma);

  EXPECT(assert_equal(expected,*actual,tol));
}

/* ************************************************************************* */
TEST( GaussianFactorGraphOrdered, eliminateOne_x1_fast )
{
  OrderingOrdered ordering; ordering += X(1),L(1),X(2);
  GaussianFactorGraphOrdered fg = createGaussianFactorGraph(ordering);
  GaussianConditionalOrdered::shared_ptr conditional;
  GaussianFactorGraphOrdered remaining;
  boost::tie(conditional,remaining) = fg.eliminateOne(ordering[X(1)], EliminateQROrdered);

  // create expected Conditional Gaussian
  Matrix I = 15*eye(2), R11 = I, S12 = -0.111111*I, S13 = -0.444444*I;
  Vector d = Vector_(2, -0.133333, -0.0222222), sigma = ones(2);
  GaussianConditionalOrdered expected(ordering[X(1)],15*d,R11,ordering[L(1)],S12,ordering[X(2)],S13,sigma);

  // Create expected remaining new factor
  JacobianFactorOrdered expectedFactor(1, Matrix_(4,2,
             4.714045207910318,                   0.,
                             0.,   4.714045207910318,
                             0.,                   0.,
                             0.,                   0.),
     2, Matrix_(4,2,
           -2.357022603955159,                   0.,
                            0.,  -2.357022603955159,
            7.071067811865475,                   0.,
                            0.,   7.071067811865475),
     Vector_(4, -0.707106781186547, 0.942809041582063, 0.707106781186547, -1.414213562373094), noiseModel::Unit::Create(4));

  EXPECT(assert_equal(expected,*conditional,tol));
  EXPECT(assert_equal((const GaussianFactorOrdered&)expectedFactor,*remaining.back(),tol));
}

/* ************************************************************************* */
TEST( GaussianFactorGraphOrdered, eliminateOne_x2_fast )
{
  OrderingOrdered ordering; ordering += X(1),L(1),X(2);
  GaussianFactorGraphOrdered fg = createGaussianFactorGraph(ordering);
  GaussianConditionalOrdered::shared_ptr actual = fg.eliminateOne(ordering[X(2)], EliminateQROrdered).first;

  // create expected Conditional Gaussian
  double sig = 0.0894427;
  Matrix I = eye(2)/sig, R11 = I, S12 = -0.2*I, S13 = -0.8*I;
  Vector d = Vector_(2, 0.2, -0.14)/sig, sigma = ones(2);
  GaussianConditionalOrdered expected(ordering[X(2)],d,R11,ordering[X(1)],S13,ordering[L(1)],S12,sigma);

  EXPECT(assert_equal(expected,*actual,tol));
}

/* ************************************************************************* */
TEST( GaussianFactorGraphOrdered, eliminateOne_l1_fast )
{
  OrderingOrdered ordering; ordering += X(1),L(1),X(2);
  GaussianFactorGraphOrdered fg = createGaussianFactorGraph(ordering);
  GaussianConditionalOrdered::shared_ptr actual = fg.eliminateOne(ordering[L(1)], EliminateQROrdered).first;

  // create expected Conditional Gaussian
  double sig = sqrt(2.0)/10.;
  Matrix I = eye(2)/sig, R11 = I, S12 = -0.5*I, S13 = -0.5*I;
  Vector d = Vector_(2, -0.1, 0.25)/sig, sigma = ones(2);
  GaussianConditionalOrdered expected(ordering[L(1)],d,R11,ordering[X(1)],S12,ordering[X(2)],S13,sigma);

  EXPECT(assert_equal(expected,*actual,tol));
}

/* ************************************************************************* */
TEST( GaussianFactorGraphOrdered, eliminateAll )
{
  // create expected Chordal bayes Net
  Matrix I = eye(2);

  OrderingOrdered ordering;
  ordering += X(2),L(1),X(1);

  Vector d1 = Vector_(2, -0.1,-0.1);
  GaussianBayesNetOrdered expected = simpleGaussian(ordering[X(1)],d1,0.1);

  double sig1 = 0.149071;
  Vector d2 = Vector_(2, 0.0, 0.2)/sig1, sigma2 = ones(2);
  push_front(expected,ordering[L(1)],d2, I/sig1,ordering[X(1)], (-1)*I/sig1,sigma2);

  double sig2 = 0.0894427;
  Vector d3 = Vector_(2, 0.2, -0.14)/sig2, sigma3 = ones(2);
  push_front(expected,ordering[X(2)],d3, I/sig2,ordering[L(1)], (-0.2)*I/sig2, ordering[X(1)], (-0.8)*I/sig2, sigma3);

  // Check one ordering
  GaussianFactorGraphOrdered fg1 = createGaussianFactorGraph(ordering);
  GaussianBayesNetOrdered actual = *GaussianSequentialSolver(fg1).eliminate();
  EXPECT(assert_equal(expected,actual,tol));

  GaussianBayesNetOrdered actualQR = *GaussianSequentialSolver(fg1, true).eliminate();
  EXPECT(assert_equal(expected,actualQR,tol));
}

/* ************************************************************************* */
TEST( GaussianFactorGraphOrdered, copying )
{
  // Create a graph
  OrderingOrdered ordering; ordering += X(2),L(1),X(1);
  GaussianFactorGraphOrdered actual = createGaussianFactorGraph(ordering);

  // Copy the graph !
  GaussianFactorGraphOrdered copy = actual;

  // now eliminate the copy
  GaussianBayesNetOrdered actual1 = *GaussianSequentialSolver(copy).eliminate();

  // Create the same graph, but not by copying
  GaussianFactorGraphOrdered expected = createGaussianFactorGraph(ordering);

  // and check that original is still the same graph
  EXPECT(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( GaussianFactorGraphOrdered, CONSTRUCTOR_GaussianBayesNet )
{
  OrderingOrdered ord;
  ord += X(2),L(1),X(1);
  GaussianFactorGraphOrdered fg = createGaussianFactorGraph(ord);

  // render with a given ordering
  GaussianBayesNetOrdered CBN = *GaussianSequentialSolver(fg).eliminate();

  // True GaussianFactorGraph
  GaussianFactorGraphOrdered fg2(CBN);
  GaussianBayesNetOrdered CBN2 = *GaussianSequentialSolver(fg2).eliminate();
  EXPECT(assert_equal(CBN,CBN2));
}

/* ************************************************************************* */
TEST( GaussianFactorGraphOrdered, getOrdering)
{
  OrderingOrdered original; original += L(1),X(1),X(2);
  FactorGraphOrdered<IndexFactorOrdered> symbolic(createGaussianFactorGraph(original));
  Permutation perm(*inference::PermutationCOLAMD(VariableIndexOrdered(symbolic)));
  OrderingOrdered actual = original; actual.permuteInPlace(perm);
  OrderingOrdered expected; expected += L(1),X(2),X(1);
  EXPECT(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( GaussianFactorGraphOrdered, optimize_Cholesky )
{
  // create an ordering
  OrderingOrdered ord; ord += X(2),L(1),X(1);

  // create a graph
  GaussianFactorGraphOrdered fg = createGaussianFactorGraph(ord);

  // optimize the graph
  VectorValuesOrdered actual = *GaussianSequentialSolver(fg, false).optimize();

  // verify
  VectorValuesOrdered expected = createCorrectDelta(ord);

  EXPECT(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( GaussianFactorGraphOrdered, optimize_QR )
{
  // create an ordering
  OrderingOrdered ord; ord += X(2),L(1),X(1);

  // create a graph
  GaussianFactorGraphOrdered fg = createGaussianFactorGraph(ord);

  // optimize the graph
  VectorValuesOrdered actual = *GaussianSequentialSolver(fg, true).optimize();

  // verify
  VectorValuesOrdered expected = createCorrectDelta(ord);

  EXPECT(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( GaussianFactorGraphOrdered, combine)
{
  // create an ordering
  OrderingOrdered ord; ord += X(2),L(1),X(1);

  // create a test graph
  GaussianFactorGraphOrdered fg1 = createGaussianFactorGraph(ord);

  // create another factor graph
  GaussianFactorGraphOrdered fg2 = createGaussianFactorGraph(ord);

  // get sizes
  size_t size1 = fg1.size();
  size_t size2 = fg2.size();

  // combine them
  fg1.combine(fg2);

  EXPECT(size1+size2 == fg1.size());
}

/* ************************************************************************* */
TEST( GaussianFactorGraphOrdered, combine2)
{
  // create an ordering
  OrderingOrdered ord; ord += X(2),L(1),X(1);

  // create a test graph
  GaussianFactorGraphOrdered fg1 = createGaussianFactorGraph(ord);

  // create another factor graph
  GaussianFactorGraphOrdered fg2 = createGaussianFactorGraph(ord);

  // get sizes
  size_t size1 = fg1.size();
  size_t size2 = fg2.size();

  // combine them
  GaussianFactorGraphOrdered fg3 = GaussianFactorGraphOrdered::combine2(fg1, fg2);

  EXPECT(size1+size2 == fg3.size());
}

/* ************************************************************************* */
// print a vector of ints if needed for debugging
void print(vector<int> v) {
  for (size_t k = 0; k < v.size(); k++)
    cout << v[k] << " ";
  cout << endl;
}

/* ************************************************************************* */
TEST(GaussianFactorGraphOrdered, createSmoother)
{
  GaussianFactorGraphOrdered fg1 = createSmoother(2).first;
  LONGS_EQUAL(3,fg1.size());
  GaussianFactorGraphOrdered fg2 = createSmoother(3).first;
  LONGS_EQUAL(5,fg2.size());
}

/* ************************************************************************* */
double error(const VectorValuesOrdered& x) {
  // create an ordering
  OrderingOrdered ord; ord += X(2),L(1),X(1);

  GaussianFactorGraphOrdered fg = createGaussianFactorGraph(ord);
  return fg.error(x);
}

/* ************************************************************************* */
TEST( GaussianFactorGraphOrdered, multiplication )
{
  // create an ordering
  OrderingOrdered ord; ord += X(2),L(1),X(1);

  GaussianFactorGraphOrdered A = createGaussianFactorGraph(ord);
  VectorValuesOrdered x = createCorrectDelta(ord);
  Errors actual = A * x;
  Errors expected;
  expected += Vector_(2,-1.0,-1.0);
  expected += Vector_(2, 2.0,-1.0);
  expected += Vector_(2, 0.0, 1.0);
  expected += Vector_(2,-1.0, 1.5);
  EXPECT(assert_equal(expected,actual));
}

/* ************************************************************************* */
// Extra test on elimination prompted by Michael's email to Frank 1/4/2010
TEST( GaussianFactorGraphOrdered, elimination )
{
  OrderingOrdered ord;
  ord += X(1), X(2);
  // Create Gaussian Factor Graph
  GaussianFactorGraphOrdered fg;
  Matrix Ap = eye(1), An = eye(1) * -1;
  Vector b = Vector_(1, 0.0);
  SharedDiagonal sigma = noiseModel::Isotropic::Sigma(1,2.0);
  fg.add(ord[X(1)], An, ord[X(2)], Ap, b, sigma);
  fg.add(ord[X(1)], Ap, b, sigma);
  fg.add(ord[X(2)], Ap, b, sigma);

  // Eliminate
  GaussianBayesNetOrdered bayesNet = *GaussianSequentialSolver(fg).eliminate();

  // Check sigma
  EXPECT_DOUBLES_EQUAL(1.0,bayesNet[ord[X(2)]]->get_sigmas()(0),1e-5);

  // Check matrix
  Matrix R;Vector d;
  boost::tie(R,d) = matrix(bayesNet);
  Matrix expected = Matrix_(2,2,
      0.707107,  -0.353553,
      0.0,   0.612372);
  Matrix expected2 = Matrix_(2,2,
      0.707107,  -0.353553,
      0.0,   -0.612372);
  EXPECT(equal_with_abs_tol(expected, R, 1e-6) || equal_with_abs_tol(expected2, R, 1e-6));
}

 /* ************************************************************************* */
// Tests ported from ConstrainedGaussianFactorGraph
/* ************************************************************************* */
TEST( GaussianFactorGraphOrdered, constrained_simple )
{
  // get a graph with a constraint in it
  GaussianFactorGraphOrdered fg = createSimpleConstraintGraph();
  EXPECT(hasConstraints(fg));


  // eliminate and solve
  VectorValuesOrdered actual = *GaussianSequentialSolver(fg).optimize();

  // verify
  VectorValuesOrdered expected = createSimpleConstraintValues();
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( GaussianFactorGraphOrdered, constrained_single )
{
  // get a graph with a constraint in it
  GaussianFactorGraphOrdered fg = createSingleConstraintGraph();
  EXPECT(hasConstraints(fg));

  // eliminate and solve
  VectorValuesOrdered actual = *GaussianSequentialSolver(fg).optimize();

  // verify
  VectorValuesOrdered expected = createSingleConstraintValues();
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( GaussianFactorGraphOrdered, constrained_multi1 )
{
  // get a graph with a constraint in it
  GaussianFactorGraphOrdered fg = createMultiConstraintGraph();
  EXPECT(hasConstraints(fg));

  // eliminate and solve
  VectorValuesOrdered actual = *GaussianSequentialSolver(fg).optimize();

  // verify
  VectorValuesOrdered expected = createMultiConstraintValues();
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */

static SharedDiagonal model = noiseModel::Isotropic::Sigma(2,1);

/* ************************************************************************* */
TEST(GaussianFactorGraphOrdered, replace)
{
  OrderingOrdered ord; ord += X(1),X(2),X(3),X(4),X(5),X(6);
  SharedDiagonal noise(noiseModel::Isotropic::Sigma(3, 1.0));

  GaussianFactorGraphOrdered::sharedFactor f1(new JacobianFactorOrdered(
      ord[X(1)], eye(3,3), ord[X(2)], eye(3,3), zero(3), noise));
  GaussianFactorGraphOrdered::sharedFactor f2(new JacobianFactorOrdered(
      ord[X(2)], eye(3,3), ord[X(3)], eye(3,3), zero(3), noise));
  GaussianFactorGraphOrdered::sharedFactor f3(new JacobianFactorOrdered(
      ord[X(3)], eye(3,3), ord[X(4)], eye(3,3), zero(3), noise));
  GaussianFactorGraphOrdered::sharedFactor f4(new JacobianFactorOrdered(
      ord[X(5)], eye(3,3), ord[X(6)], eye(3,3), zero(3), noise));

  GaussianFactorGraphOrdered actual;
  actual.push_back(f1);
  actual.push_back(f2);
  actual.push_back(f3);
  actual.replace(0, f4);

  GaussianFactorGraphOrdered expected;
  expected.push_back(f4);
  expected.push_back(f2);
  expected.push_back(f3);

  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(GaussianFactorGraphOrdered, createSmoother2)
{
  using namespace example;
  GaussianFactorGraphOrdered fg2;
  OrderingOrdered ordering;
  boost::tie(fg2,ordering) = createSmoother(3);
  LONGS_EQUAL(5,fg2.size());

  // eliminate
  vector<Index> x3var; x3var.push_back(ordering[X(3)]);
  vector<Index> x1var; x1var.push_back(ordering[X(1)]);
  GaussianBayesNetOrdered p_x3 = *GaussianSequentialSolver(
      *GaussianSequentialSolver(fg2).jointFactorGraph(x3var)).eliminate();
  GaussianBayesNetOrdered p_x1 = *GaussianSequentialSolver(
      *GaussianSequentialSolver(fg2).jointFactorGraph(x1var)).eliminate();
  CHECK(assert_equal(*p_x1.back(),*p_x3.front())); // should be the same because of symmetry
}

/* ************************************************************************* */
TEST(GaussianFactorGraphOrdered, hasConstraints)
{
  FactorGraphOrdered<GaussianFactorOrdered> fgc1 = createMultiConstraintGraph();
  EXPECT(hasConstraints(fgc1));

  FactorGraphOrdered<GaussianFactorOrdered> fgc2 = createSimpleConstraintGraph() ;
  EXPECT(hasConstraints(fgc2));

  OrderingOrdered ordering; ordering += X(1), X(2), L(1);
  GaussianFactorGraphOrdered fg = createGaussianFactorGraph(ordering);
  EXPECT(!hasConstraints(fg));
}

#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/RangeFactor.h>
#include <gtsam/linear/GaussianMultifrontalSolver.h>

/* ************************************************************************* */
TEST( GaussianFactorGraphOrdered, conditional_sigma_failure) {
  // This system derives from a failure case in DDF in which a Bayes Tree
  // has non-unit sigmas for conditionals in the Bayes Tree, which
  // should never happen by construction

  // Reason for the failure: using Vector_() is dangerous as having a non-float gets set to zero, resulting in constraints
  gtsam::Key xC1 = 0, l32 = 1, l41 = 2;

  // noisemodels at nonlinear level
  gtsam::SharedNoiseModel priorModel = noiseModel::Diagonal::Sigmas(Vector_(6, 0.05, 0.05, 3.0, 0.2, 0.2, 0.2));
  gtsam::SharedNoiseModel measModel = noiseModel::Unit::Create(2);
  gtsam::SharedNoiseModel elevationModel = noiseModel::Isotropic::Sigma(1, 3.0);

  double fov = 60; // degrees
  double imgW = 640; // pixels
  double imgH = 480; // pixels
  gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(fov, imgW, imgH));

  typedef GenericProjectionFactor<Pose3, Point3> ProjectionFactor;

  double relElevation = 6;

  Values initValues;
  initValues.insert(xC1,
      Pose3(Rot3(
          -1.,           0.0,  1.2246468e-16,
          0.0,             1.,           0.0,
          -1.2246468e-16,           0.0,            -1.),
          Point3(0.511832102, 8.42819594, 5.76841725)));
  initValues.insert(l32,  Point3(0.364081507, 6.89766221, -0.231582751) );
  initValues.insert(l41,  Point3(1.61051523, 6.7373052, -0.231582751)   );

  NonlinearFactorGraph factors;
  factors.add(PriorFactor<Pose3>(xC1,
      Pose3(Rot3(
          -1.,           0.0,  1.2246468e-16,
          0.0,             1.,           0.0,
          -1.2246468e-16,           0.0,            -1),
          Point3(0.511832102, 8.42819594, 5.76841725)), priorModel));
  factors.add(ProjectionFactor(Point2(333.648615, 98.61535), measModel, xC1, l32, K));
  factors.add(ProjectionFactor(Point2(218.508, 83.8022039), measModel, xC1, l41, K));
  factors.add(RangeFactor<Pose3,Point3>(xC1, l32, relElevation, elevationModel));
  factors.add(RangeFactor<Pose3,Point3>(xC1, l41, relElevation, elevationModel));

  OrderingOrdered orderingC; orderingC += xC1, l32, l41;

  // Check that sigmas are correct (i.e., unit)
  GaussianFactorGraphOrdered lfg = *factors.linearize(initValues, orderingC);

  GaussianMultifrontalSolver solver(lfg, false);
  GaussianBayesTreeOrdered actBT = *solver.eliminate();

  // Check that all sigmas in an unconstrained bayes tree are set to one
  BOOST_FOREACH(const GaussianBayesTreeOrdered::sharedClique& clique, actBT.nodes()) {
    GaussianConditionalOrdered::shared_ptr conditional = clique->conditional();
    size_t dim = conditional->dim();
    EXPECT(assert_equal(gtsam::ones(dim), conditional->get_sigmas(), tol));
  }
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
