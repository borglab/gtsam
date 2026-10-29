/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testGradients.cpp
 * @brief Independent algebra, manifold and input-contract checks for Gradients.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/linear/FixedJacobianFactor.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/nonlinear/CustomFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Gradients.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>

#include <array>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

using namespace gtsam;

/* ************************************************************************* */
namespace linear_fixture {

// Dense packing is confined to small independent algebra references in tests.
Errors factorBlocks(const GaussianFactorGraph& factors, const Vector& dense) {
  Errors blocks;
  DenseIndex offset = 0;
  for (const auto& factor : factors) {
    const DenseIndex rows = factor ? static_cast<const JacobianFactor&>(*factor).rows() : 0;
    blocks.push_back(dense.segment(offset, rows));
    offset += rows;
  }
  return blocks;
}

Vector stack(const Errors& blocks) {
  DenseIndex rows = 0;
  for (const Vector& block : blocks) rows += block.size();
  Vector dense(rows);
  DenseIndex offset = 0;
  for (const Vector& block : blocks) {
    dense.segment(offset, block.size()) = block;
    offset += block.size();
  }
  return dense;
}

GaussianFactorGraph graph() {
  GaussianFactorGraph result;
  result.emplace_shared<JacobianFactor>(
      19, Matrix2::Identity(), Vector2{.2, -.1},
      noiseModel::Diagonal::Sigmas(Vector2{2., 3.}));
  result.push_back(GaussianFactor::shared_ptr{});
  result.emplace_shared<JacobianFactor>(
      19, Matrix2::Identity(), 4, Matrix21{{1.}, {2.}}, Vector2{.5, -.3},
      noiseModel::Diagonal::Sigmas(Vector2{.7, 1.2}));
  result.emplace_shared<JacobianFactor>(
      19, -Matrix2::Identity(), 91, Matrix2::Identity(), Vector2{.1, .4});
  result.emplace_shared<JacobianFactor>(
      4, Matrix21{{2.}, {-1.}}, 91, Matrix2::Identity(), Vector2{-.2, .8},
      noiseModel::Diagonal::Sigmas(Vector2{.8, 1.5}));
  return result;
}

VectorValues source() {
  return {{4, Vector1{1.}}, {19, Vector2{-.3, .2}},
          {91, Vector2{.5, -.8}}};
}

bool matchesDense(const Gradients& gradients, const GaussianFactorGraph& factors,
                  const VectorValues& seed, const Ordering& ordering) {
  const Matrix jacobian = factors.jacobian(ordering).first;
  const Matrix hessian = jacobian.transpose() * jacobian;
  const Vector expectedAdjoint = hessian.ldlt().solve(seed.vector(ordering));
  const Vector direction = Vector::LinSpaced(jacobian.rows(), -.4, .8);
  const Vector expectedMotion =
      -hessian.ldlt().solve(jacobian.transpose() * direction);
  const Errors pullback = gradients.pullback(seed);
  const Vector motion = gradients.pushforward(factorBlocks(factors, direction)).vector(ordering);
  return assert_equal(expectedAdjoint, gradients.adjoint(seed).vector(ordering),
                      1e-10) &&
         assert_equal(expectedMotion, motion, 1e-10) &&
         assert_equal(Vector(-jacobian * expectedAdjoint), stack(pullback), 1e-10) &&
         std::abs(seed.vector(ordering).dot(motion) - dot(factorBlocks(factors, direction), pullback)) <
             1e-10;
}

// Both elimination methods and nontrivial key orders give the dense response.
TEST(Gradients, GaussianConstructorsAndOrdering) {
  const auto factors = graph();
  for (const auto method : {Gradients::CHOLESKY, Gradients::QR}) {
    EXPECT(matchesDense(Gradients(factors, method), factors, source(),
                        Ordering{4, 19, 91}));
    for (const Ordering& ordering : {Ordering{4, 19, 91}, Ordering{91, 19, 4}}) {
      const Gradients gradients(graph(), ordering, method);
      EXPECT(matchesDense(gradients, factors, source(), ordering));
    }
  }
}

// Each original factor slot has one block, including empty slots.
TEST(Gradients, FactorResidualBlocksAndSparseSeeds) {
  const Gradients gradients(graph());
  const VectorValues sparse{{4, Vector1{2.}}};
  const VectorValues complete{{4, Vector1{2.}}, {19, Vector2::Zero()},
                              {91, Vector2::Zero()}};
  const Errors factorCotangents = gradients.pullback(sparse);
  EXPECT_LONGS_EQUAL(5, factorCotangents.size());
  auto block = factorCotangents.begin();
  for (size_t dimension : {2, 0, 2, 2, 2}) {
    EXPECT_LONGS_EQUAL(dimension, block->size());
    ++block;
  }
  EXPECT(assert_equal(gradients.pullback(complete), factorCotangents));
  EXPECT_LONGS_EQUAL(3, gradients.adjoint(sparse).size());
  const Errors zero = gradients.pullback(VectorValues{});
  EXPECT_DOUBLES_EQUAL(0., dot(zero, zero), 0.);
}

// Fresh construction owns its data and supports repeated concurrent queries.
TEST(Gradients, RepeatedConcurrentQueries) {
  auto factors = graph();
  const Gradients gradients(factors);
  const Gradients copied = gradients;
  const Errors expected = gradients.pullback(source());
  std::dynamic_pointer_cast<JacobianFactor>(factors[0])->getA().setConstant(17.);
  factors.resize(0);
  auto first = std::async(std::launch::async, [&] { return gradients.pullback(source()); });
  auto second = std::async(std::launch::async, [&] { return copied.pullback(source()); });
  EXPECT(assert_equal(expected, first.get(), 1e-12));
  EXPECT(assert_equal(expected, second.get(), 1e-12));
}

// Affine offsets do not enter products, even when cancellation would be severe.
TEST(Gradients, HugeResidualOffsetDoesNotAffectDerivatives) {
  GaussianFactorGraph factors;
  factors.emplace_shared<JacobianFactor>(
      8, Matrix11{{2.}}, Vector1{1e25}, noiseModel::Isotropic::Sigma(1, 3.));
  const Gradients gradients(factors);
  EXPECT(assert_equal(Errors{Vector1{-1.5}}, gradients.pullback({{8, Vector1{1.}}})));
  EXPECT(assert_equal(Vector1{-1.5}, gradients.pushforward(Errors{Vector1{1.}}).at(8)));
}

// Parameter columns, shared parameters and direct loss terms obey the chain rule.
TEST(Gradients, SharedParameterGradientAndDirectTerm) {
  const auto factors = graph();
  const Gradients gradients(factors);
  const Matrix jacobian = factors.jacobian(Ordering{4, 19, 91}).first;
  Matrix parameter = Matrix::Zero(8, 2);
  parameter.block<2, 2>(2, 0) = Matrix2{{1., 2.}, {-1., .5}};
  parameter.block<2, 2>(6, 0) = Matrix2{{.5, 1.}, {1., -1.}};
  const Vector2 direct{.3, -.2}, direction{.7, -.4};
  const Vector parameterGradient = parameter.transpose() * stack(gradients.pullback(source())) + direct;
  const Vector expected =
      -parameter.transpose() * jacobian *
          (jacobian.transpose() * jacobian).ldlt().solve(source().vector()) + direct;
  EXPECT(assert_equal(expected, parameterGradient, 1e-10));
  const auto motion = gradients.pushforward(factorBlocks(factors, parameter * direction));
  EXPECT_DOUBLES_EQUAL(parameterGradient.dot(direction),
                       source().dot(motion) + direct.dot(direction), 1e-10);
}

}  // namespace linear_fixture
/* ************************************************************************* */

namespace tree_fixture {

using Clique = GaussianBayesTreeClique;

GaussianFactorGraph branchedGraph() {
  GaussianFactorGraph result;
  for (Key key = 0; key < 7; ++key)
    result.emplace_shared<JacobianFactor>(key, Matrix11{{1.}}, Vector1::Zero());
  const std::array<std::pair<Key, Key>, 9> edges{
      {{0, 2}, {0, 5}, {1, 2}, {2, 4}, {3, 4}, {3, 6}, {4, 5}, {4, 6}, {5, 6}}};
  for (const auto& [first, second] : edges)
    result.emplace_shared<JacobianFactor>(first, Matrix11{{-1.}}, second,
                                        Matrix11{{1.}}, Vector1::Zero());
  return result;
}

Clique::shared_ptr clique(const GaussianConditional& conditional) {
  return std::make_shared<Clique>(std::make_shared<GaussianConditional>(conditional));
}

void connect(const Clique::shared_ptr& parent, const Clique::shared_ptr& child) {
  parent->children.push_back(child);
  child->parent_ = parent;
}

// Sparse supervision crosses deeper separators and reaches an unsupervised leaf.
TEST(Gradients, BranchedMultifrontalTreeAndDisconnectedRoot) {
  auto factors = branchedGraph();
  factors.emplace_shared<JacobianFactor>(
      88, Matrix11{{1.}}, Vector1::Zero(), noiseModel::Isotropic::Sigma(1, 3.));
  const Ordering ordering{0, 1, 2, 3, 4, 5, 6, 88};
  const auto tree = factors.eliminateMultifrontal(ordering);
  EXPECT_LONGS_EQUAL(2, tree->roots().size());
  EXPECT_LONGS_EQUAL(3, tree->clique(4)->conditional()->nrFrontals());
  const Gradients gradients(factors, *tree);
  VectorValues complete;
  for (const Key key : ordering) complete.insert(key, Vector1::Zero());
  complete.at(0)[0] = 1.;
  EXPECT(linear_fixture::matchesDense(gradients, factors, complete, ordering));
  const auto adjoint = gradients.adjoint({{0, Vector1{1.}}});
  EXPECT(std::abs(adjoint.at(3)[0]) > .01);
  EXPECT_DOUBLES_EQUAL(0., adjoint.at(88)[0], 1e-12);
  EXPECT_DOUBLES_EQUAL(18., gradients.adjoint({{88, Vector1{2.}}}).at(88)[0], 1e-12);
}

struct NonunitTree {
  Matrix2 childR{{2., .7}, {0., 3.}}, childS{{1., -.2}, {.5, 2.}};
  Matrix2 rootR{{1., .3}, {0., 1.5}};
  SharedDiagonal childNoise = noiseModel::Diagonal::Sigmas(Vector2{2., 4.});
  SharedDiagonal rootNoise = noiseModel::Diagonal::Sigmas(Vector2{.6, 1.3});
  GaussianFactorGraph factors;
  GaussianBayesTree tree;
  Clique::shared_ptr child, root;

  NonunitTree() {
    factors.emplace_shared<JacobianFactor>(20, rootR, Vector2{9., -8.}, rootNoise);
    factors.emplace_shared<JacobianFactor>(10, childR, 20, childS,
                                          Vector2{1e25, -1e25}, childNoise);
    root = clique(GaussianConditional(20, Vector2{9., -8.}, rootR, rootNoise));
    child = clique(GaussianConditional(10, Vector2{1e25, -1e25}, childR, 20,
                                       childS, childNoise));
    connect(root, child);
    tree.insertRoot(root);
  }
};

// Ordinary tree copying shares conditional/noise ownership beyond source life.
TEST(Gradients, ImportedNonunitConditionalsAndLifetime) {
  const VectorValues seed{{10, Vector2{1., -.5}}, {20, Vector2{.2, 1.4}}};
  std::weak_ptr<GaussianConditional> conditional;
  std::weak_ptr<noiseModel::Diagonal> noise;
  std::unique_ptr<Gradients> gradients;
  Errors expected;
  {
    NonunitTree data;
    conditional = data.child->conditional();
    noise = data.childNoise;
    gradients = std::make_unique<Gradients>(data.factors, data.tree);
    EXPECT(linear_fixture::matchesDense(*gradients, data.factors, seed,
                                       Ordering{10, 20}));
    expected = gradients->pullback(seed);
  }
  EXPECT(!conditional.expired());
  EXPECT(!noise.expired());
  for (size_t repeat = 0; repeat < 3; ++repeat)
    EXPECT(assert_equal(expected, gradients->pullback(seed), 1e-12));
  gradients.reset();
  EXPECT(conditional.expired());
  EXPECT(noise.expired());
}

// Ordinary imported conditionals retain small positive Gaussian sigmas.
TEST(Gradients, ImportedSmallPositiveNoise) {
  const double sigma = 1e-12;
  const auto noise = noiseModel::Diagonal::Sigmas(Vector1{sigma}, false);
  GaussianFactorGraph graph;
  graph.emplace_shared<JacobianFactor>(10, Matrix11{{sigma}}, Vector1{9.}, noise);
  GaussianBayesTree tree;
  tree.addClique(clique(GaussianConditional(
      10, Vector1{9.}, Matrix11{{sigma}}, noise)));
  const Gradients gradients(graph, tree);
  const VectorValues seed{{10, Vector1{2.}}};
  EXPECT(assert_equal(Vector1{2.}, gradients.adjoint(seed).at(10), 1e-12));
  EXPECT(assert_equal(Errors{Vector1{-2.}}, gradients.pullback(seed), 1e-12));
}

// Constructor checks reject incompatible state keys and singular conditionals.
TEST(Gradients, ImportedTreeCompatibilityErrors) {
  NonunitTree data;
  GaussianBayesTree incomplete;
  incomplete.insertRoot(clique(GaussianConditional(20, Vector2::Zero(), Matrix2::Identity())));
  CHECK_EXCEPTION(Gradients(data.factors, incomplete), std::invalid_argument);
  data.child->conditional()->getA()(0, 0) = 0.;
  CHECK_EXCEPTION(Gradients(data.factors, data.tree), IndeterminateSystemException);
}

// Native triangular solves must not silently discard lower-triangular input entries.
TEST(Gradients, ImportedNontriangularConditional) {
  NonunitTree data;
  data.child->conditional()->getA()(1, 0) = .2;
  CHECK_EXCEPTION(Gradients(data.factors, data.tree), std::invalid_argument);
}

}  // namespace tree_fixture
/* ************************************************************************* */

namespace nonlinear_fixture {

class ExplicitFactor : public NoiseModelFactor {
 public:
  Matrix matrix;
  Vector offset;

  ExplicitFactor(const Matrix& matrix, const Vector& offset,
                 const SharedNoiseModel& noise)
      : NoiseModelFactor(noise, KeyVector{42}), matrix(matrix), offset(offset) {}

  Vector unwhitenedError(const Values& values,
                        OptionalMatrixVecType jacobians = nullptr) const override {
    if (jacobians) (*jacobians)[0] = matrix;
    return matrix * values.at<Vector>(42) - offset;
  }

  std::shared_ptr<GaussianFactor> linearize(const Values&) const override {
    // Deliberately unrelated reduced representation: the constructor must use
    // this factor's explicit residual and Jacobians, as its API documents.
    return std::make_shared<HessianFactor>(42, Matrix2::Identity(), Vector2::Zero(), 0.);
  }
};

class InactivePrior : public PriorFactor<Vector> {
 public:
  using PriorFactor<Vector>::PriorFactor;
  bool active(const Values&) const override { return false; }
};

Values values() {
  Values result;
  result.insert(42, Vector(Vector2{.2, -.3}));
  result.insert(999, Pose2(1., 2., .4));  // Extra Values create no state blocks.
  return result;
}

// Full covariance whitening preserves original residual rows despite linearize overrides.
TEST(Gradients, ExplicitResidualsBypassReducedLinearize) {
  const Matrix2 matrix{{2., .4}, {-.5, 1.3}};
  const auto noise = noiseModel::Gaussian::Covariance(Matrix2{{2., .3}, {.3, .7}});
  auto factor = std::make_shared<ExplicitFactor>(matrix, Vector2{1., -.7}, noise);
  NonlinearFactorGraph graph;
  graph.push_back(factor);
  GaussianFactorGraph expected;
  expected.emplace_shared<JacobianFactor>(42, noise->Whiten(matrix), Vector2::Zero());
  for (const auto method : {Gradients::CHOLESKY, Gradients::QR}) {
    EXPECT(linear_fixture::matchesDense(Gradients(graph, values(), method), expected,
                                        {{42, Vector2{1., -.2}}}, Ordering{42}));
    EXPECT(linear_fixture::matchesDense(Gradients(graph, values(), Ordering{42}, method),
                                        expected, {{42, Vector2{1., -.2}}}, Ordering{42}));
  }
}

// Nonlinear construction owns its numerical rows and supports null Gaussian noise.
TEST(Gradients, NonlinearOwnershipAndUnitNoise) {
  auto factor = std::make_shared<ExplicitFactor>(Matrix2::Identity(), Vector2::Zero(),
                                                SharedNoiseModel{});
  NonlinearFactorGraph graph;
  graph.push_back(factor);
  auto solution = values();
  const Gradients gradients(graph, solution);
  const VectorValues seed{{42, Vector2{1., -.2}}};
  const Errors expected = gradients.pullback(seed);
  factor->matrix.setConstant(9.);
  solution.update(42, Vector(Vector2{10., 20.}));
  graph.resize(0);
  EXPECT(assert_equal(Errors{Vector2{-1., .2}}, expected, 1e-12));
  EXPECT(assert_equal(expected, gradients.pullback(seed), 1e-12));
}

// Null and inactive slots consume no rows, and inactive keys require no Values.
TEST(Gradients, InactiveNonlinearFactorSlots) {
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Vector>>(42, Vector(Vector2::Zero()),
                                            noiseModel::Unit::Create(2));
  graph.push_back(NonlinearFactor::shared_ptr{});
  graph.emplace_shared<InactivePrior>(88, Vector(Vector1{0.}),
                                      noiseModel::Unit::Create(1));
  const Gradients gradients(graph, values());
  const Errors factorCotangents = gradients.pullback(VectorValues{});
  EXPECT_LONGS_EQUAL(3, factorCotangents.size());
  EXPECT_LONGS_EQUAL(2, factorCotangents.front().size());
  EXPECT_LONGS_EQUAL(0, std::next(factorCotangents.begin())->size());
  EXPECT_LONGS_EQUAL(0, factorCotangents.back().size());
  EXPECT_LONGS_EQUAL(1, gradients.adjoint(VectorValues{}).size());
}

// Empty, null-only and constant-factor graphs preserve their factor blocks.
TEST(Gradients, EmptyAndConstantResidualGraphs) {
  const Gradients empty{GaussianFactorGraph{}};
  EXPECT_LONGS_EQUAL(0, empty.pushforward(Errors{}).size());
  EXPECT_LONGS_EQUAL(0, empty.pullback(VectorValues{}).size());
  GaussianFactorGraph nulls;
  nulls.resize(2);
  const Gradients nullGradients(nulls);
  const Errors emptyBlocks{Vector{}, Vector{}};
  EXPECT(assert_equal(emptyBlocks, nullGradients.pullback(VectorValues{})));
  EXPECT_LONGS_EQUAL(0, nullGradients.pushforward(emptyBlocks).size());
  GaussianFactorGraph constants;
  constants.emplace_shared<JacobianFactor>(Vector(Vector2{4., -8.}));
  const Gradients gradients(constants);
  EXPECT_LONGS_EQUAL(0, gradients.pushforward(Errors{Vector2{2., 3.}}).size());
  EXPECT(assert_equal(Errors{Vector2::Zero()}, gradients.pullback(VectorValues{})));
}

}  // namespace nonlinear_fixture
/* ************************************************************************* */

namespace invalid_fixture {

class ProjectedJacobian : public JacobianFactor {
 public:
  ProjectedJacobian() : JacobianFactor(42, Matrix2::Identity(), Vector2::Zero()) {}
};

// Unknown keys, dimensions and nonfinite query entries are rejected before solving.
TEST(Gradients, InvalidQueryInputs) {
  const Gradients gradients(linear_fixture::graph());
  CHECK_EXCEPTION(gradients.adjoint({{100, Vector1{1.}}}), std::invalid_argument);
  CHECK_EXCEPTION(gradients.pullback({{4, Vector2{1., 2.}}}), std::invalid_argument);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  CHECK_EXCEPTION(gradients.adjoint({{4, Vector1{nan}}}), std::invalid_argument);
  CHECK_EXCEPTION(gradients.pushforward(Errors{}), std::invalid_argument);
  Errors bad = gradients.pullback(VectorValues{});
  bad.front() = Vector1::Zero();
  CHECK_EXCEPTION(gradients.pushforward(bad), std::invalid_argument);
  bad = gradients.pullback(VectorValues{});
  std::next(bad.begin())->resize(1);  // A null factor slot requires an empty block.
  CHECK_EXCEPTION(gradients.pushforward(bad), std::invalid_argument);
  bad = gradients.pullback(VectorValues{});
  bad.front()[0] = std::numeric_limits<double>::infinity();
  CHECK_EXCEPTION(gradients.pushforward(bad), std::invalid_argument);
  bad = gradients.pullback(VectorValues{});
  bad.push_back(Vector{});
  CHECK_EXCEPTION(gradients.pushforward(bad), std::invalid_argument);
}

// Explicit derived Jacobian rows retain their caller-defined residual coordinates.
TEST(Gradients, DerivedJacobianFactors) {
  GaussianFactorGraph fixed, projected;
  fixed.emplace_shared<FixedJacobianFactor<2, 2>>(
      KeyVector{42}, std::vector<Matrix>{Matrix2{{2., .4}, {-.3, 1.}}},
      Vector2{.1, -.4}, noiseModel::Diagonal::Sigmas(Vector2{.7, 1.3}));
  projected.emplace_shared<ProjectedJacobian>();
  for (const auto& graph : {fixed, projected}) {
    EXPECT(linear_fixture::matchesDense(Gradients(graph), graph,
                                        {{42, Vector2{.2, -.7}}}, Ordering{42}));
  }
}

// Hessian factors cannot define residual cotangents; hard constraints are unsupported.
TEST(Gradients, UnsupportedImportedFactors) {
  GaussianFactorGraph hessian, constrained;
  hessian.emplace_shared<HessianFactor>(42, Matrix2::Identity(), Vector2::Zero(), 0.);
  constrained.emplace_shared<JacobianFactor>(
      42, Matrix2::Identity(), Vector2::Zero(), noiseModel::Constrained::All(2));
  CHECK_EXCEPTION((Gradients(hessian)), std::invalid_argument);
  CHECK_EXCEPTION((Gradients(constrained)), std::invalid_argument);
}

// Robust, hard-constrained and non-residual nonlinear factors are unsupported.
TEST(Gradients, UnsupportedNonlinearModels) {
  const auto robust = noiseModel::Robust::Create(
      noiseModel::mEstimator::Huber::Create(1.345), noiseModel::Unit::Create(2));
  for (const SharedNoiseModel& noise :
       {SharedNoiseModel(robust), SharedNoiseModel(noiseModel::Constrained::All(2))}) {
    NonlinearFactorGraph graph;
    graph.emplace_shared<PriorFactor<Vector>>(42, Vector(Vector2::Zero()), noise);
    CHECK_EXCEPTION(Gradients(graph, nonlinear_fixture::values()), std::invalid_argument);
  }
  NonlinearFactorGraph container;
  container.emplace_shared<LinearContainerFactor>(
      JacobianFactor(42, Matrix2::Identity(), Vector2::Zero()));
  CHECK_EXCEPTION(Gradients(container, nonlinear_fixture::values()), std::invalid_argument);
}

// Invalid orderings, missing Values and nonfinite coefficients fail explicitly.
TEST(Gradients, InvalidConstructionInputs) {
  const auto graph = linear_fixture::graph();
  CHECK_EXCEPTION(Gradients(graph, Ordering{4, 19}), std::invalid_argument);
  CHECK_EXCEPTION(Gradients(graph, Ordering{4, 19, 19}), std::invalid_argument);
  NonlinearFactorGraph nonlinear;
  nonlinear.emplace_shared<PriorFactor<Vector>>(42, Vector(Vector2::Zero()),
                                                noiseModel::Unit::Create(2));
  CHECK_EXCEPTION(Gradients(nonlinear, Values{}), std::exception);
  GaussianFactorGraph nonfinite;
  Matrix11 matrix{{std::numeric_limits<double>::quiet_NaN()}};
  nonfinite.emplace_shared<JacobianFactor>(0, matrix, Vector1::Zero());
  CHECK_EXCEPTION((Gradients(nonfinite)), std::invalid_argument);
}

// Malformed callbacks cannot resize away Jacobian slots before validation indexes them.
TEST(Gradients, MalformedCallbackJacobianCount) {
  NonlinearFactorGraph graph;
  graph.emplace_shared<CustomFactor>(noiseModel::Unit::Create(2), KeyVector{42},
      [](const CustomFactor&, const Values&, OptionalJacobianVector jacobians) -> Vector {
        if (jacobians) jacobians->get().clear();
        return Vector2::Zero();
      });
  CHECK_EXCEPTION(Gradients(graph, nonlinear_fixture::values()), std::invalid_argument);
}

// Gauge freedoms are errors; neither factorization adds damping or a pseudoinverse.
TEST(Gradients, SingularGraph) {
  GaussianFactorGraph graph;
  graph.emplace_shared<JacobianFactor>(0, Matrix11{{-1.}}, 1, Matrix11{{1.}}, Vector1::Zero());
  CHECK_EXCEPTION(Gradients(graph, Gradients::CHOLESKY), IndeterminateSystemException);
  CHECK_EXCEPTION(Gradients(graph, Gradients::QR), IndeterminateSystemException);
  // Enough residual rows do not imply full column rank.
  GaussianFactorGraph square;
  square.emplace_shared<JacobianFactor>(
      0, Matrix2{{1., 0.}, {0., 0.}}, Vector2::Zero());
  CHECK_EXCEPTION(Gradients(square, Gradients::CHOLESKY), IndeterminateSystemException);
  CHECK_EXCEPTION(Gradients(square, Gradients::QR), IndeterminateSystemException);
}

}  // namespace invalid_fixture
/* ************************************************************************* */

namespace mathematical_fixture {

enum class Curvature { State, Mixed, ZeroResidual };

NonlinearFactorGraph scalarGraph(Curvature kind, double parameter) {
  const size_t rows = kind == Curvature::ZeroResidual ? 1 : 2;
  NonlinearFactorGraph graph;
  graph.emplace_shared<CustomFactor>(noiseModel::Unit::Create(rows), KeyVector{0},
      [kind, parameter](const CustomFactor&, const Values& values,
                        OptionalJacobianVector jacobians) -> Vector {
        const double x = values.at<double>(0);
        if (jacobians) {
          auto& matrix = jacobians->get()[0];
          if (kind == Curvature::State) matrix = Matrix21{{1.}, {2 * x}};
          else if (kind == Curvature::Mixed) matrix = Matrix21{{parameter}, {1.}};
          else matrix = Matrix11{{2 * x}};
        }
        if (kind == Curvature::State) return Vector2{x - parameter, x * x};
        if (kind == Curvature::Mixed) return Vector2{parameter * x - 1., x - 3.};
        return Vector1{x * x - parameter};
      });
  return graph;
}

double stationaryState(Curvature kind, double parameter) {
  if (kind == Curvature::Mixed) return (parameter + 3.) / (parameter * parameter + 1.);
  if (kind == Curvature::ZeroResidual) return std::sqrt(parameter);
  double x = 1.;
  for (size_t iteration = 0; iteration < 12; ++iteration)
    x -= (x + 2 * x * x * x - parameter) / (1 + 6 * x * x);
  return x;
}

// State curvature yields GN 1/5 while differentiating nonlinear stationarity gives 1/7.
TEST(Gradients, NonzeroResidualStateCurvature) {
  Values solution;
  solution.insert(0, 1.);
  const auto graph = scalarGraph(Curvature::State, 3.);
  const Gradients gradients(graph, solution);
  EXPECT_DOUBLES_EQUAL(.2, gradients.pushforward(Errors{Vector2{-1., 0.}}).at(0)[0], 1e-12);
  EXPECT_DOUBLES_EQUAL(0., graph.linearize(solution)->gradientAtZero().norm(), 1e-12);
  for (double step : {1e-4, 1e-5, 1e-6}) {
    const double finite = (stationaryState(Curvature::State, 3. + step) -
                           stationaryState(Curvature::State, 3. - step)) / (2 * step);
    EXPECT_DOUBLES_EQUAL(1. / 7., finite, 1e-8);
    EXPECT(std::abs(finite - .2) > .05);
  }
}

// A changing state Jacobian contributes a mixed term even for state-affine residuals.
TEST(Gradients, NonzeroResidualMixedCurvature) {
  Values solution;
  solution.insert(0, 2.);
  const auto graph = scalarGraph(Curvature::Mixed, 1.);
  const Gradients gradients(graph, solution);
  EXPECT_DOUBLES_EQUAL(-1., gradients.pushforward(Errors{Vector2{2., 0.}}).at(0)[0], 1e-12);
  EXPECT_DOUBLES_EQUAL(0., graph.linearize(solution)->gradientAtZero().norm(), 1e-12);
  for (double step : {1e-4, 1e-5, 1e-6}) {
    const double finite = (stationaryState(Curvature::Mixed, 1. + step) -
                           stationaryState(Curvature::Mixed, 1. - step)) / (2 * step);
    EXPECT_DOUBLES_EQUAL(-1.5, finite, 1e-8);
    EXPECT(std::abs(finite + 1.) > .4);
  }
}

// Zero residual removes both omitted curvature terms, recovering the exact response.
TEST(Gradients, ZeroResidualNonlinearResponse) {
  Values solution;
  solution.insert(0, 2.);
  const Gradients gradients(scalarGraph(Curvature::ZeroResidual, 4.), solution);
  EXPECT_DOUBLES_EQUAL(.25, gradients.pushforward(Errors{Vector1{-1.}}).at(0)[0], 1e-12);
  EXPECT_DOUBLES_EQUAL(.25, -gradients.pullback({{0, Vector1{1.}}}).front()[0], 1e-12);
  for (double step : {1e-4, 1e-5, 1e-6}) {
    const double finite = (std::sqrt(4. + step) - std::sqrt(4. - step)) / (2 * step);
    EXPECT_DOUBLES_EQUAL(.25, finite, 1e-8);
  }
}

}  // namespace mathematical_fixture
/* ************************************************************************* */

namespace manifold_fixture {

const Pose2 base(.4, -.2, .3), reference(-.3, .7, -.6);
const Vector3 direction{.3, -.4, .7}, sigmas{.4, .8, 1.1};
const Matrix3 weight = (Vector3{.7, 2., 1.3}).asDiagonal();

Pose2 measurement(double parameter) {
  return base.compose(Pose2::Expmap(parameter * direction));
}

std::pair<NonlinearFactorGraph, Values> solve(double parameter) {
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Pose2>>(
      89, measurement(parameter), noiseModel::Diagonal::Sigmas(sigmas));
  Values initial;
  initial.insert(89, base);
  GaussNewtonParams settings;
  settings.setMaxIterations(100);
  settings.setAbsoluteErrorTol(1e-18);
  settings.setRelativeErrorTol(1e-14);
  return {graph, GaussNewtonOptimizer(graph, initial, settings).optimize()};
}

double loss(const Pose2& state) {
  const Vector3 error = reference.localCoordinates(state);
  return .5 * error.dot(weight * error);
}

// Tangent bases and the reference-to-solution Local Jacobian agree with reoptimization.
TEST(Gradients, ManifoldLossAndParameterFiniteDifferences) {
  const double parameter = .35;
  const auto [graph, solution] = solve(parameter);
  const auto& state = solution.at<Pose2>(89);
  const Gradients gradients(graph, solution);
  Matrix3 comparison;
  const Vector3 error = reference.localCoordinates(state, {}, comparison);
  const Vector3 source = comparison.transpose() * weight * error;
  const Vector3 parameterJacobian = -direction.cwiseQuotient(sigmas);
  const Vector motion = gradients.pushforward(Errors{parameterJacobian}).at(89);
  const double gradient = parameterJacobian.dot(gradients.pullback({{89, source}}).front());
  EXPECT(assert_equal(direction, motion, 1e-12));
  EXPECT((source - weight * error).norm() > .1);
  for (double step : {1e-4, 1e-5, 1e-6}) {
    const Pose2 positive = solve(parameter + step).second.at<Pose2>(89);
    const Pose2 negative = solve(parameter - step).second.at<Pose2>(89);
    const Vector3 finiteMotion =
        (state.localCoordinates(positive) - state.localCoordinates(negative)) / (2 * step);
    EXPECT(assert_equal(direction, finiteMotion, 1e-8));
    EXPECT_DOUBLES_EQUAL(gradient, (loss(positive) - loss(negative)) / (2 * step), 1e-8);
  }
}

}  // namespace manifold_fixture
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
