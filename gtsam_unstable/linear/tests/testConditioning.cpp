/**
 * @file testConditioning.cpp
 *
 * @brief Experiments using backsubstitution for conditioning (not summarization, it turns out)
 *
 * @date Sep 3, 2012
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/TestableAssertions.h>

#include <boost/assign/std/set.hpp>
#include <boost/assign/std/list.hpp>
#include <boost/assign/std/vector.hpp>

#include <gtsam_unstable/linear/conditioning.h>

using namespace std;
using namespace boost::assign;
using namespace gtsam;

const double tol = 1e-5;

// Simple example
Matrix R = Matrix_(3,3,
    1.0,-2.0,-3.0,
    0.0, 3.0,-5.0,
    0.0, 0.0, 6.0);
Vector d = Vector_(3,
    0.1, 0.2, 0.3);
Vector x = Vector_(3,
    0.55,
    0.15,
    0.05);

/* ************************************************************************* */
TEST( testConditioning, directed_elimination_example ) {

  // create a 3-variable system from which to eliminate variables
  // Scalar variables, pre-factorized into R,d system
  // Use multifrontal representation
  // Variables 0, 1, 2 - want to summarize out 1
  Vector expx = R.triangularView<Eigen::Upper>().solve(d);
  EXPECT(assert_equal(x, expx, tol));
  EXPECT(assert_equal(Vector(R*x), d, tol));

  // backsub-summarized version
  Matrix Rprime = Matrix_(2,2,
      1.0,-3.0,
      0.0, 6.0);
  Vector dprime = Vector_(2,
      d(0) - R(0,1)*x(1),
      d(2));
  Vector xprime = Vector_(2,
      x(0), // Same solution, just smaller
      x(2));
  EXPECT(assert_equal(Vector(Rprime*xprime), dprime, tol));
}

/* ************************************************************************* */
TEST( testConditioning, directed_elimination_singlefrontal ) {
  // Gaussian conditional with a single frontal variable, parent is to be removed
  // Top row from above example

  Index root_key = 0, removed_key = 1, remaining_parent = 2;
  Matrix R11 = Matrix_(1,1, 1.0), R22 = Matrix_(1,1, 3.0), S = Matrix_(1,1,-2.0), T = Matrix_(1,1,-3.0);
  Vector d0 = d.segment(0,1), d1 = d.segment(1,1);
  SharedDiagonal sigmas = noiseModel::Unit::Create(1);
  GaussianConditional::shared_ptr initConditional(new
      GaussianConditional(root_key, d0, R11, removed_key, S, remaining_parent, T, sigmas));

  VectorValues solution;
  solution.insert(0, x.segment(0,1));
  solution.insert(1, x.segment(1,1));
  solution.insert(2, x.segment(2,1));

  std::set<Index> saved_indices;
  saved_indices += root_key, remaining_parent;

  GaussianConditional::shared_ptr actSummarized = conditionDensity(initConditional, saved_indices, solution);
  GaussianConditional::shared_ptr expSummarized(new
      GaussianConditional(root_key, d0 - S*x(1), R11, remaining_parent, T, sigmas));

  CHECK(actSummarized);
  EXPECT(assert_equal(*expSummarized, *actSummarized, tol));

  // Simple test of base case: if target index isn't present, return clone
  GaussianConditional::shared_ptr actSummarizedSimple = conditionDensity(expSummarized, saved_indices, solution);
  CHECK(actSummarizedSimple);
  EXPECT(assert_equal(*expSummarized, *actSummarizedSimple, tol));

  // case where frontal variable is to be eliminated - return null
  GaussianConditional::shared_ptr removeFrontalInit(new
        GaussianConditional(removed_key, d1, R22, remaining_parent, T, sigmas));
  GaussianConditional::shared_ptr actRemoveFrontal = conditionDensity(removeFrontalInit, saved_indices, solution);
  EXPECT(!actRemoveFrontal);
}

///* ************************************************************************* */
//TEST( testConditioning, directed_elimination_multifrontal ) {
//  // Use top two rows from the previous example
//  Index root_key = 0, removed_key = 1, remaining_parent = 2;
//  Matrix R11 = R.topLeftCorner(2,2), S = R.block(0,2,2,1),
//      Sprime = Matrix_(1,1,-2.0), R11prime = Matrix_(1,1, 1.0);
//  Vector d1 = d.segment(0,2);
//  SharedDiagonal sigmas1 = noiseModel::Unit::Create(1), sigmas2 = noiseModel::Unit::Create(2);
//
//
//  std::list<std::pair<Index, Matrix> > terms;
//  terms += make_pair(root_key, Matrix(R11.col(0)));
//  terms += make_pair(removed_key, Matrix(R11.col(1)));
//  terms += make_pair(remaining_parent, S);
//  GaussianConditional::shared_ptr initConditional(new GaussianConditional(terms, 2, d1, sigmas2));
//
//  VectorValues solution;
//  solution.insert(0, x.segment(0,1));
//  solution.insert(1, x.segment(1,1));
//  solution.insert(2, x.segment(2,1));
//
//  std::set<Index> saved_indices;
//  saved_indices += root_key, remaining_parent;
//
//  GaussianConditional::shared_ptr actSummarized = conditionDensity(initConditional, saved_indices, solution);
//  GaussianConditional::shared_ptr expSummarized(new
//      GaussianConditional(root_key, d.segment(0,1) - Sprime*x(1), R11prime, remaining_parent, R.block(0,2,1,1), sigmas1));
//
//  CHECK(actSummarized);
//  EXPECT(assert_equal(*expSummarized, *actSummarized, tol));
//}
//
///* ************************************************************************* */
//TEST( testConditioning, directed_elimination_multifrontal_multidim ) {
//  // use larger example, three frontal variables, dim = 2 each, two parents (one removed)
//  // Vars: 0, 1, 2, 3, 4; frontal: 0, 1, 2. parents: 3, 4;
//  // Remove 1, 3
//  Matrix Rinit = Matrix_(6, 11,
//      1.0, 0.0,  2.0, 0.0,  3.0, 0.0,  1.0, 0.0, -1.0, 0.0,  0.1,
//      0.0, 1.0,  0.0, 2.0,  0.0, 3.0,  0.0, 1.0,  0.0, 1.0,  0.2,
//      0.0, 0.0,  3.0, 0.0,  4.0, 0.0,  0.0,-1.0,  1.0, 0.0,  0.3,
//      0.0, 0.0,  0.0, 4.0,  0.0, 4.0,  3.0, 2.0,  0.0, 9.0,  0.4,
//      0.0, 0.0,  0.0, 0.0,  5.0, 0.0,  7.0, 0.0,  3.0, 0.0,  0.5,
//      0.0, 0.0,  0.0, 0.0,  0.0, 4.0,  0.0, 8.0,  0.0, 6.0,  0.6);
//
//  vector<size_t> init_dims; init_dims += 2, 2, 2, 2, 2, 1;
//  VerticalBlockMatrix init_matrices(init_dims, Rinit);
//  SharedDiagonal sigmas = noiseModel::Unit::Create(6);
//  vector<size_t> init_keys; init_keys += 0, 1, 2, 3, 4;
//  GaussianConditional::shared_ptr initConditional(new
//      GaussianConditional(init_keys, 3, init_matrices, sigmas));
//
//  // Construct a solution vector
//  VectorValues solution;
//  solution.insert(0, zero(2));
//  solution.insert(1, zero(2));
//  solution.insert(2, zero(2));
//  solution.insert(3, Vector_(2, 1.0, 2.0));
//  solution.insert(4, Vector_(2, 3.0, 4.0));
//
//  solution = initConditional->solve(solution);
//
//  std::set<Index> saved_indices;
//  saved_indices += 0, 2, 4;
//
//  GaussianConditional::shared_ptr actSummarized = conditionDensity(initConditional, saved_indices, solution);
//  CHECK(actSummarized);
//
//  Matrix Rexp = Matrix_(4, 7,
//      1.0, 0.0,  3.0, 0.0,  -1.0, 0.0,  0.1,
//      0.0, 1.0,  0.0, 3.0,   0.0, 1.0,  0.2,
//      0.0, 0.0,  5.0, 0.0,   3.0, 0.0,  0.5,
//      0.0, 0.0,  0.0, 4.0,   0.0, 6.0,  0.6);
//
//  // Update rhs
//  Rexp.block(0, 6, 2, 1) -= Rinit.block(0, 2, 2, 2) * solution.at(1) + Rinit.block(0, 6, 2, 2) * solution.at(3);
//  Rexp.block(2, 6, 2, 1) -= Rinit.block(4, 6, 2, 2) * solution.at(3);
//
//  vector<size_t> exp_dims; exp_dims += 2, 2, 2, 1;
//  VerticalBlockMatrix exp_matrices(exp_dims, Rexp);
//  SharedDiagonal exp_sigmas = noiseModel::Unit::Create(4);
//  vector<size_t> exp_keys; exp_keys += 0, 2, 4;
//  GaussianConditional expSummarized(exp_keys, 2, exp_matrices, exp_sigmas);
//
//  EXPECT(assert_equal(expSummarized, *actSummarized, tol));
//}
//
///* ************************************************************************* */
//TEST( testConditioning, directed_elimination_multifrontal_multidim2 ) {
//  // Example from LinearAugmentedSystem
//  // 4 variables, last two in ordering kept - should be able to do this with no computation.
//
//  vector<size_t> init_dims; init_dims += 3, 3, 2, 2, 1;
//
//  //Full initial conditional: density on [3] [4] [5] [6]
//  Matrix Rinit = Matrix_(10, 11,
//      8.78422312,  -0.0375455118,  -0.0387376278,     -5.059576,           0.0,           0.0,  -0.0887200041,  0.00429643583,  -0.130078263,  0.0193260727,  0.0,
//      0.0,    8.46951839,    9.51456887,  -0.0224291821,   -5.24757636,           0.0,  0.0586258904,  -0.173455825,    0.11090295,  -0.330696013,        0.0,
//      0.0,           0.0,    16.5539485,  0.00105159359,   -2.35354497,   -6.04085484,  -0.0212095105,  0.0978729072,  0.00471054272,  0.0694956367,    0.0,
//      0.0,           0.0,           0.0,    10.9015885,  -0.0105694572,  0.000582715469,  -0.0410535006,  0.00162772139,  -0.0601433772,  0.0082824087,0.0,
//      0.0,           0.0,           0.0,           0.0,    10.5531086,   -1.34722553,    0.02438072,  -0.0644224578,  0.0561372492,  -0.148932792,        0.0,
//      0.0,           0.0,           0.0,           0.0,           0.0,    21.4870439,  -0.00443305851,  0.0234766354,  0.00484572411,  0.0101997356,      0.0,
//      0.0,           0.0,           0.0,           0.0,           0.0,           0.0,    2.73892865,  0.0242046766,  -0.0459727048,  0.0445071938,        0.0,
//      0.0,           0.0,           0.0,           0.0,           0.0,           0.0,           0.0,    2.61246954,    0.02287419,  -0.102870789,          0.0,
//      0.0,           0.0,           0.0,           0.0,           0.0,           0.0,           0.0,           0.0,    2.04823446,  -0.302033014,          0.0,
//      0.0,           0.0,           0.0,           0.0,           0.0,           0.0,           0.0,           0.0,           0.0,    2.24068986,          0.0);
//  Vector dinit = Vector_(10,
//      -0.00186915, 0.00318554, 0.000592421, -0.000861, 0.00171528, 0.000274123, -0.0284011, 0.0275465, 0.0439795, -0.0222134);
//  Rinit.rightCols(1) = dinit;
//  SharedDiagonal sigmas = noiseModel::Unit::Create(10);
//
//  VerticalBlockMatrix init_matrices(init_dims, Rinit);
//  vector<size_t> init_keys; init_keys += 3, 4, 5, 6;
//  GaussianConditional::shared_ptr initConditional(new
//      GaussianConditional(init_keys, 4, init_matrices, sigmas));
//
//  // Calculate a solution
//  VectorValues solution;
//  solution.insert(0, zero(3));
//  solution.insert(1, zero(3));
//  solution.insert(2, zero(3));
//  solution.insert(3, zero(3));
//  solution.insert(4, zero(3));
//  solution.insert(5, zero(2));
//  solution.insert(6, zero(2));
//
//  solution = initConditional->solve(solution);
//
//  // Perform summarization
//  std::set<Index> saved_indices;
//  saved_indices += 5, 6;
//
//  GaussianConditional::shared_ptr actSummarized = conditionDensity(initConditional, saved_indices, solution);
//  CHECK(actSummarized);
//
//  // Create expected value on [5], [6]
//  Matrix Rexp = Matrix_(4, 5,
//      2.73892865,  0.0242046766,  -0.0459727048,  0.0445071938,  -0.0284011,
//             0.0,    2.61246954,    0.02287419,  -0.102870789,     0.0275465,
//             0.0,           0.0,    2.04823446,  -0.302033014,     0.0439795,
//             0.0,           0.0,           0.0,    2.24068986,    -0.0222134);
//  SharedDiagonal expsigmas = noiseModel::Unit::Create(4);
//
//  vector<size_t> exp_dims; exp_dims += 2, 2, 1;
//  VerticalBlockMatrix exp_matrices(exp_dims, Rexp);
//  vector<size_t> exp_keys; exp_keys += 5, 6;
//  GaussianConditional expConditional(exp_keys, 2, exp_matrices, expsigmas);
//
//  EXPECT(assert_equal(expConditional, *actSummarized, tol));
//}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
