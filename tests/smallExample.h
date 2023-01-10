/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    smallExample.h
 * @brief   Create small example with two poses and one landmark
 * @brief   smallExample
 * @author  Carlos Nieto
 */

// \callgraph


#pragma once

#include <tests/simulated2D.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>

namespace gtsam {
namespace example {

/**
 * Create small example for non-linear factor graph
 */
// inline boost::shared_ptr<const NonlinearFactorGraph> sharedNonlinearFactorGraph();
// inline NonlinearFactorGraph createNonlinearFactorGraph();

/**
 * Create values structure to go with it
 * The ground truth values structure for the example above
 */
// inline Values createValues();

/** Vector Values equivalent */
// inline VectorValues createVectorValues();

/**
 * create a noisy values structure for a nonlinear factor graph
 */
// inline boost::shared_ptr<const Values> sharedNoisyValues();
// inline Values createNoisyValues();

/**
 * Zero delta config
 */
// inline VectorValues createZeroDelta();

/**
 * Delta config that, when added to noisyValues, returns the ground truth
 */
// inline VectorValues createCorrectDelta();

/**
 * create a linear factor graph
 * The non-linear graph above evaluated at NoisyValues
 */
// inline GaussianFactorGraph createGaussianFactorGraph();

/**
 * create small Chordal Bayes Net x <- y
 */
// inline GaussianBayesNet createSmallGaussianBayesNet();

/**
 * Create really non-linear factor graph (cos/sin)
 */
// inline boost::shared_ptr<const NonlinearFactorGraph>
//sharedReallyNonlinearFactorGraph();
// inline NonlinearFactorGraph createReallyNonlinearFactorGraph();

/**
 * Create a full nonlinear smoother
 * @param T number of time-steps
 */
// inline std::pair<NonlinearFactorGraph, Values> createNonlinearSmoother(int T);

/**
 * Create a Kalman smoother by linearizing a non-linear factor graph
 * @param T number of time-steps
 */
// inline GaussianFactorGraph createSmoother(int T);

/* ******************************************************* */
// Linear Constrained Examples
/* ******************************************************* */

/**
 * Creates a simple constrained graph with one linear factor and
 * one binary equality constraint that sets x = y
 */
// inline GaussianFactorGraph createSimpleConstraintGraph();
// inline VectorValues createSimpleConstraintValues();

/**
 * Creates a simple constrained graph with one linear factor and
 * one binary constraint.
 */
// inline GaussianFactorGraph createSingleConstraintGraph();
// inline VectorValues createSingleConstraintValues();

/**
 * Creates a constrained graph with a linear factor and two
 * binary constraints that share a node
 */
// inline GaussianFactorGraph createMultiConstraintGraph();
// inline VectorValues createMultiConstraintValues();

/* ******************************************************* */
// Planar graph with easy subtree for SubgraphPreconditioner
/* ******************************************************* */

/*
 * Create factor graph with N^2 nodes, for example for N=3
 *  x13-x23-x33
 *   |   |   |
 *  x12-x22-x32
 *   |   |   |
 * -x11-x21-x31
 * with x11 clamped at (1,1), and others related by 2D odometry.
 */
// inline std::pair<GaussianFactorGraph, VectorValues> planarGraph(size_t N);

/*
 * Create canonical ordering for planar graph that also works for tree
 * With x11 the root, e.g. for N=3
 * x33 x23 x13 x32 x22 x12 x31 x21 x11
 */
// inline Ordering planarOrdering(size_t N);

/*
 * Split graph into tree and loop closing constraints, e.g., with N=3
 *  x13-x23-x33
 *   |
 *  x12-x22-x32
 *   |
 * -x11-x21-x31
 */
// inline std::pair<GaussianFactorGraph, GaussianFactorGraph > splitOffPlanarTree(
//    size_t N, const GaussianFactorGraph& original);



// Implementations

//  using namespace gtsam::noiseModel;

namespace impl {
typedef boost::shared_ptr<NonlinearFactor> shared_nlf;

static SharedDiagonal kSigma1_0 = noiseModel::Isotropic::Sigma(2,1.0);
static SharedDiagonal kSigma0_1 = noiseModel::Isotropic::Sigma(2,0.1);
static SharedDiagonal kSigma0_2 = noiseModel::Isotropic::Sigma(2,0.2);
static SharedDiagonal kConstrainedModel = noiseModel::Constrained::All(2);

static const Key _l1_=0, _x1_=1, _x2_=2;
static const Key _x_=0, _y_=1, _z_=2;
} // \namespace impl


/* ************************************************************************* */
inline boost::shared_ptr<const NonlinearFactorGraph>
sharedNonlinearFactorGraph(const SharedNoiseModel &noiseModel1 = impl::kSigma0_1,
                           const SharedNoiseModel &noiseModel2 = impl::kSigma0_2) {
  using namespace impl;
  using symbol_shorthand::L;
  using symbol_shorthand::X;
  // Create
  boost::shared_ptr<NonlinearFactorGraph> nlfg(new NonlinearFactorGraph);

  // prior on x1
  Point2 mu(0, 0);
  shared_nlf f1(new simulated2D::Prior(mu, noiseModel1, X(1)));
  nlfg->push_back(f1);

  // odometry between x1 and x2
  Point2 z2(1.5, 0);
  shared_nlf f2(new simulated2D::Odometry(z2, noiseModel1, X(1), X(2)));
  nlfg->push_back(f2);

  // measurement between x1 and l1
  Point2 z3(0, -1);
  shared_nlf f3(new simulated2D::Measurement(z3, noiseModel2, X(1), L(1)));
  nlfg->push_back(f3);

  // measurement between x2 and l1
  Point2 z4(-1.5, -1.);
  shared_nlf f4(new simulated2D::Measurement(z4, noiseModel2, X(2), L(1)));
  nlfg->push_back(f4);

  return nlfg;
}

/* ************************************************************************* */
inline NonlinearFactorGraph
createNonlinearFactorGraph(const SharedNoiseModel &noiseModel1 = impl::kSigma0_1,
                           const SharedNoiseModel &noiseModel2 = impl::kSigma0_2) {
  return *sharedNonlinearFactorGraph(noiseModel1, noiseModel2);
}

/* ************************************************************************* */
inline Values createValues() {
  using symbol_shorthand::X;
  using symbol_shorthand::L;
  Values c;
  c.insert(X(1), Point2(0.0, 0.0));
  c.insert(X(2), Point2(1.5, 0.0));
  c.insert(L(1), Point2(0.0, -1.0));
  return c;
}

/* ************************************************************************* */
inline VectorValues createVectorValues() {
  using namespace impl;
  VectorValues c {{_l1_, Vector2(0.0, -1.0)},
                  {_x1_, Vector2(0.0, 0.0)},
                  {_x2_, Vector2(1.5, 0.0)}};
  return c;
}

/* ************************************************************************* */
inline boost::shared_ptr<const Values> sharedNoisyValues() {
  using symbol_shorthand::X;
  using symbol_shorthand::L;
  boost::shared_ptr<Values> c(new Values);
  c->insert(X(1), Point2(0.1, 0.1));
  c->insert(X(2), Point2(1.4, 0.2));
  c->insert(L(1), Point2(0.1, -1.1));
  return c;
}

/* ************************************************************************* */
inline Values createNoisyValues() {
  return *sharedNoisyValues();
}

/* ************************************************************************* */
inline VectorValues createCorrectDelta() {
  using symbol_shorthand::X;
  using symbol_shorthand::L;
  VectorValues c;
  c.insert(L(1), Vector2(-0.1, 0.1));
  c.insert(X(1), Vector2(-0.1, -0.1));
  c.insert(X(2), Vector2(0.1, -0.2));
  return c;
}

/* ************************************************************************* */
inline VectorValues createZeroDelta() {
  using symbol_shorthand::X;
  using symbol_shorthand::L;
  VectorValues c;
  c.insert(L(1), Z_2x1);
  c.insert(X(1), Z_2x1);
  c.insert(X(2), Z_2x1);
  return c;
}

/* ************************************************************************* */
inline GaussianFactorGraph createGaussianFactorGraph() {
  using symbol_shorthand::X;
  using symbol_shorthand::L;
  // Create empty graph
  GaussianFactorGraph fg;

  // linearized prior on x1: c[_x1_]+x1=0 i.e. x1=-c[_x1_]
  fg += JacobianFactor(X(1), 10*I_2x2, -1.0*Vector::Ones(2));

  // odometry between x1 and x2: x2-x1=[0.2;-0.1]
  fg += JacobianFactor(X(1), -10*I_2x2, X(2), 10*I_2x2, Vector2(2.0, -1.0));

  // measurement between x1 and l1: l1-x1=[0.0;0.2]
  fg += JacobianFactor(X(1), -5*I_2x2, L(1), 5*I_2x2, Vector2(0.0, 1.0));

  // measurement between x2 and l1: l1-x2=[-0.2;0.3]
  fg += JacobianFactor(X(2), -5*I_2x2, L(1), 5*I_2x2, Vector2(-1.0, 1.5));

  return fg;
}

/* ************************************************************************* */
/** create small Chordal Bayes Net x <- y
 * x y d
 * 1 1 9
 *   1 5
 */
inline GaussianBayesNet createSmallGaussianBayesNet() {
  using namespace impl;
  Matrix R11 = (Matrix(1, 1) << 1.0).finished(), S12 = (Matrix(1, 1) << 1.0).finished();
  Matrix R22 = (Matrix(1, 1) << 1.0).finished();
  Vector d1(1), d2(1);
  d1(0) = 9;
  d2(0) = 5;

  // define nodes and specify in reverse topological sort (i.e. parents last)
  GaussianConditional::shared_ptr Px_y(new GaussianConditional(_x_, d1, R11, _y_, S12));
  GaussianConditional::shared_ptr Py(new GaussianConditional(_y_, d2, R22));
  GaussianBayesNet cbn;
  cbn.push_back(Px_y);
  cbn.push_back(Py);

  return cbn;
}

/* ************************************************************************* */
// Some nonlinear functions to optimize
/* ************************************************************************* */
namespace smallOptimize {

inline Point2 h(const Point2& v) {
  return Point2(cos(v.x()), sin(v.y()));
}

inline Matrix H(const Point2& v) {
  return (Matrix(2, 2) <<
      -sin(v.x()), 0.0,
      0.0, cos(v.y())).finished();
}

struct UnaryFactor: public gtsam::NoiseModelFactorN<Point2> {

  Point2 z_;

  UnaryFactor(const Point2& z, const SharedNoiseModel& model, Key key) :
    gtsam::NoiseModelFactorN<Point2>(model, key), z_(z) {
  }

  Vector evaluateError(const Point2& x, boost::optional<Matrix&> A = boost::none) const override {
    if (A) *A = H(x);
    return (h(x) - z_);
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new UnaryFactor(*this))); }
};

}

/* ************************************************************************* */
inline NonlinearFactorGraph nonlinearFactorGraphWithGivenSigma(const double sigma) {
  using symbol_shorthand::X;
  using symbol_shorthand::L;
  boost::shared_ptr<NonlinearFactorGraph> fg(new NonlinearFactorGraph);
  Point2 z(1.0, 0.0);
  boost::shared_ptr<smallOptimize::UnaryFactor> factor(
      new smallOptimize::UnaryFactor(z, noiseModel::Isotropic::Sigma(2,sigma), X(1)));
  fg->push_back(factor);
  return *fg;
}

/* ************************************************************************* */
inline boost::shared_ptr<const NonlinearFactorGraph> sharedReallyNonlinearFactorGraph() {
  using symbol_shorthand::X;
  using symbol_shorthand::L;
  boost::shared_ptr<NonlinearFactorGraph> fg(new NonlinearFactorGraph);
  Point2 z(1.0, 0.0);
  double sigma = 0.1;
  boost::shared_ptr<smallOptimize::UnaryFactor> factor(
      new smallOptimize::UnaryFactor(z, noiseModel::Isotropic::Sigma(2,sigma), X(1)));
  fg->push_back(factor);
  return fg;
}

inline NonlinearFactorGraph createReallyNonlinearFactorGraph() {
  return *sharedReallyNonlinearFactorGraph();
}

/* ************************************************************************* */
inline NonlinearFactorGraph sharedNonRobustFactorGraphWithOutliers() {
  using symbol_shorthand::X;
  boost::shared_ptr<NonlinearFactorGraph> fg(new NonlinearFactorGraph);
  Point2 z(0.0, 0.0);
  double sigma = 0.1;

  boost::shared_ptr<PriorFactor<Point2>> factor(
      new PriorFactor<Point2>(X(1), z, noiseModel::Isotropic::Sigma(2,sigma)));
  // 3 noiseless inliers
  fg->push_back(factor);
  fg->push_back(factor);
  fg->push_back(factor);

  // 1 outlier
  Point2 z_out(1.0, 0.0);
  boost::shared_ptr<PriorFactor<Point2>> factor_out(
      new PriorFactor<Point2>(X(1), z_out, noiseModel::Isotropic::Sigma(2,sigma)));
  fg->push_back(factor_out);

  return *fg;
}

/* ************************************************************************* */
inline NonlinearFactorGraph sharedRobustFactorGraphWithOutliers() {
  using symbol_shorthand::X;
  boost::shared_ptr<NonlinearFactorGraph> fg(new NonlinearFactorGraph);
  Point2 z(0.0, 0.0);
  double sigma = 0.1;
  auto gmNoise = noiseModel::Robust::Create(
            noiseModel::mEstimator::GemanMcClure::Create(1.0), noiseModel::Isotropic::Sigma(2,sigma));
  boost::shared_ptr<PriorFactor<Point2>> factor(
      new PriorFactor<Point2>(X(1), z, gmNoise));
  // 3 noiseless inliers
  fg->push_back(factor);
  fg->push_back(factor);
  fg->push_back(factor);

  // 1 outlier
  Point2 z_out(1.0, 0.0);
  boost::shared_ptr<PriorFactor<Point2>> factor_out(
      new PriorFactor<Point2>(X(1), z_out, gmNoise));
  fg->push_back(factor_out);

  return *fg;
}


/* ************************************************************************* */
inline std::pair<NonlinearFactorGraph, Values> createNonlinearSmoother(int T) {
  using namespace impl;
  using symbol_shorthand::X;
  using symbol_shorthand::L;

  // Create
  NonlinearFactorGraph nlfg;
  Values poses;

  // prior on x1
  Point2 x1(1.0, 0.0);
  shared_nlf prior(new simulated2D::Prior(x1, kSigma1_0, X(1)));
  nlfg.push_back(prior);
  poses.insert(X(1), x1);

  for (int t = 2; t <= T; t++) {
    // odometry between x_t and x_{t-1}
    Point2 odo(1.0, 0.0);
    shared_nlf odometry(new simulated2D::Odometry(odo, kSigma1_0, X(t - 1), X(t)));
    nlfg.push_back(odometry);

    // measurement on x_t is like perfect GPS
    Point2 xt(t, 0);
    shared_nlf measurement(new simulated2D::Prior(xt, kSigma1_0, X(t)));
    nlfg.push_back(measurement);

    // initial estimate
    poses.insert(X(t), xt);
  }

  return std::make_pair(nlfg, poses);
}

/* ************************************************************************* */
inline GaussianFactorGraph createSmoother(int T) {
  NonlinearFactorGraph nlfg;
  Values poses;
  std::tie(nlfg, poses) = createNonlinearSmoother(T);

  return *nlfg.linearize(poses);
}

/* ************************************************************************* */
inline GaussianFactorGraph createSimpleConstraintGraph() {
  using namespace impl;
  // create unary factor
  // prior on _x_, mean = [1,-1], sigma=0.1
  Matrix Ax = I_2x2;
  Vector b1(2);
  b1(0) = 1.0;
  b1(1) = -1.0;
  JacobianFactor::shared_ptr f1(new JacobianFactor(_x_, Ax, b1, kSigma0_1));

  // create binary constraint factor
  // between _x_ and _y_, that is going to be the only factor on _y_
  // |1 0||x_1| + |-1  0||y_1| = |0|
  // |0 1||x_2|   | 0 -1||y_2|   |0|
  Matrix Ax1 = I_2x2;
  Matrix Ay1 = I_2x2 * -1;
  Vector b2 = Vector2(0.0, 0.0);
  JacobianFactor::shared_ptr f2(new JacobianFactor(_x_, Ax1, _y_, Ay1, b2,
      kConstrainedModel));

  // construct the graph
  GaussianFactorGraph fg;
  fg.push_back(f1);
  fg.push_back(f2);

  return fg;
}

/* ************************************************************************* */
inline VectorValues createSimpleConstraintValues() {
  using namespace impl;
  using symbol_shorthand::X;
  using symbol_shorthand::L;
  VectorValues config;
  Vector v = Vector2(1.0, -1.0);
  config.insert(_x_, v);
  config.insert(_y_, v);
  return config;
}

/* ************************************************************************* */
inline GaussianFactorGraph createSingleConstraintGraph() {
  using namespace impl;
  // create unary factor
  // prior on _x_, mean = [1,-1], sigma=0.1
  Matrix Ax = I_2x2;
  Vector b1(2);
  b1(0) = 1.0;
  b1(1) = -1.0;
  //GaussianFactor::shared_ptr f1(new JacobianFactor(_x_, kSigma0_1->Whiten(Ax), kSigma0_1->whiten(b1), kSigma0_1));
  JacobianFactor::shared_ptr f1(new JacobianFactor(_x_, Ax, b1, kSigma0_1));

  // create binary constraint factor
  // between _x_ and _y_, that is going to be the only factor on _y_
  // |1 2||x_1| + |10 0||y_1| = |1|
  // |2 1||x_2|   |0 10||y_2|   |2|
  Matrix Ax1(2, 2);
  Ax1(0, 0) = 1.0;
  Ax1(0, 1) = 2.0;
  Ax1(1, 0) = 2.0;
  Ax1(1, 1) = 1.0;
  Matrix Ay1 = I_2x2 * 10;
  Vector b2 = Vector2(1.0, 2.0);
  JacobianFactor::shared_ptr f2(new JacobianFactor(_x_, Ax1, _y_, Ay1, b2,
      kConstrainedModel));

  // construct the graph
  GaussianFactorGraph fg;
  fg.push_back(f1);
  fg.push_back(f2);

  return fg;
}

/* ************************************************************************* */
inline VectorValues createSingleConstraintValues() {
  using namespace impl;
  VectorValues config{{_x_, Vector2(1.0, -1.0)}, {_y_, Vector2(0.2, 0.1)}};
  return config;
}

/* ************************************************************************* */
inline GaussianFactorGraph createMultiConstraintGraph() {
  using namespace impl;
  // unary factor 1
  Matrix A = I_2x2;
  Vector b = Vector2(-2.0, 2.0);
  JacobianFactor::shared_ptr lf1(new JacobianFactor(_x_, A, b, kSigma0_1));

  // constraint 1
  Matrix A11(2, 2);
  A11(0, 0) = 1.0;
  A11(0, 1) = 2.0;
  A11(1, 0) = 2.0;
  A11(1, 1) = 1.0;

  Matrix A12(2, 2);
  A12(0, 0) = 10.0;
  A12(0, 1) = 0.0;
  A12(1, 0) = 0.0;
  A12(1, 1) = 10.0;

  Vector b1(2);
  b1(0) = 1.0;
  b1(1) = 2.0;
  JacobianFactor::shared_ptr lc1(new JacobianFactor(_x_, A11, _y_, A12, b1,
      kConstrainedModel));

  // constraint 2
  Matrix A21(2, 2);
  A21(0, 0) = 3.0;
  A21(0, 1) = 4.0;
  A21(1, 0) = -1.0;
  A21(1, 1) = -2.0;

  Matrix A22(2, 2);
  A22(0, 0) = 1.0;
  A22(0, 1) = 1.0;
  A22(1, 0) = 1.0;
  A22(1, 1) = 2.0;

  Vector b2(2);
  b2(0) = 3.0;
  b2(1) = 4.0;
  JacobianFactor::shared_ptr lc2(new JacobianFactor(_x_, A21, _z_, A22, b2,
      kConstrainedModel));

  // construct the graph
  GaussianFactorGraph fg;
  fg.push_back(lf1);
  fg.push_back(lc1);
  fg.push_back(lc2);

  return fg;
}

/* ************************************************************************* */
inline VectorValues createMultiConstraintValues() {
  using namespace impl;
  VectorValues config{{_x_, Vector2(-2.0, 2.0)},
                      {_y_, Vector2(-0.1, 0.4)},
                      {_z_, Vector2(-4.0, 5.0)}};
  return config;
}

/* ************************************************************************* */
// Create key for simulated planar graph
namespace impl {
inline Symbol key(size_t x, size_t y) {
  using symbol_shorthand::X;
  return X(1000*x+y);
}
} // \namespace impl

/* ************************************************************************* */
inline std::pair<GaussianFactorGraph, VectorValues> planarGraph(size_t N) {
  using namespace impl;

  // create empty graph
  NonlinearFactorGraph nlfg;

  // Create almost hard constraint on x11, sigma=0 will work for PCG not for normal
  shared_nlf constraint(new simulated2D::Prior(Point2(1.0, 1.0), noiseModel::Isotropic::Sigma(2,1e-3), key(1,1)));
  nlfg.push_back(constraint);

  // Create horizontal constraints, 1...N*(N-1)
  Point2 z1(1.0, 0.0); // move right
  for (size_t x = 1; x < N; x++)
    for (size_t y = 1; y <= N; y++) {
      shared_nlf f(new simulated2D::Odometry(z1, noiseModel::Isotropic::Sigma(2,0.01), key(x, y), key(x + 1, y)));
      nlfg.push_back(f);
    }

  // Create vertical constraints, N*(N-1)+1..2*N*(N-1)
  Point2 z2(0.0, 1.0); // move up
  for (size_t x = 1; x <= N; x++)
    for (size_t y = 1; y < N; y++) {
      shared_nlf f(new simulated2D::Odometry(z2, noiseModel::Isotropic::Sigma(2,0.01), key(x, y), key(x, y + 1)));
      nlfg.push_back(f);
    }

  // Create linearization and ground xtrue config
  Values zeros;
  for (size_t x = 1; x <= N; x++)
    for (size_t y = 1; y <= N; y++)
      zeros.insert(key(x, y), Point2(0,0));
  VectorValues xtrue;
  for (size_t x = 1; x <= N; x++)
    for (size_t y = 1; y <= N; y++)
      xtrue.insert(key(x, y), Point2((double)x, (double)y));

  // linearize around zero
  boost::shared_ptr<GaussianFactorGraph> gfg = nlfg.linearize(zeros);
  return std::make_pair(*gfg, xtrue);
}

/* ************************************************************************* */
inline Ordering planarOrdering(size_t N) {
  Ordering ordering;
  for (size_t y = N; y >= 1; y--)
    for (size_t x = N; x >= 1; x--)
      ordering.push_back(impl::key(x, y));
  return ordering;
}

/* ************************************************************************* */
inline std::pair<GaussianFactorGraph, GaussianFactorGraph> splitOffPlanarTree(
    size_t N, const GaussianFactorGraph& original) {
  GaussianFactorGraph T, C;

  // Add the x11 constraint to the tree
  T.push_back(original[0]);

  // Add all horizontal constraints to the tree
  size_t i = 1;
  for (size_t x = 1; x < N; x++)
    for (size_t y = 1; y <= N; y++, i++) T.push_back(original[i]);

  // Add first vertical column of constraints to T, others to C
  for (size_t x = 1; x <= N; x++)
    for (size_t y = 1; y < N; y++, i++)
      if (x == 1)
        T.push_back(original[i]);
      else
        C.push_back(original[i]);

  return std::make_pair(T, C);
}

/* ************************************************************************* */

} // \namespace example
} // \namespace gtsam
