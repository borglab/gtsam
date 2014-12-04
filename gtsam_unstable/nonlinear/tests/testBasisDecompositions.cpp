/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testBasisDecompositions.cpp
 * @date November 23, 2014
 * @author Frank Dellaert
 * @brief unit tests for Basis Decompositions w Expressions
 */

#include <gtsam/base/Matrix.h>

namespace gtsam {

/// Fourier
template<int N>
class Fourier {

public:

  typedef Eigen::Matrix<double, N, 1> Coefficients;
  typedef Eigen::Matrix<double, 1, N> Jacobian;

private:

  double x_;
  Jacobian H_;

public:

  /// Constructor
  Fourier(double x) :
      x_(x) {
    H_(0, 0) = 1;
    for (size_t i = 1; i < N; i += 2) {
      H_(0, i) = cos(i * x);
      H_(0, i + 1) = sin(i * x);
    }
  }

  /// Given coefficients c, predict value for x
  double operator()(const Coefficients& c, OptionalJacobian<1, N> H) {
    if (H)
      (*H) = H_;
    return H_ * c;
  }
};

}

#include <gtsam_unstable/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>

namespace gtsam {

/// For now, this is our sequence representation
typedef std::map<double, double> Sequence;

/**
 * Class that does Fourier Decomposition via least squares
 */
class FourierDecomposition {
public:

  typedef Vector3 Coefficients; ///< Fourier coefficients

private:
  Coefficients c_;

public:

  /// Create nonlinear FG from Sequence
  static NonlinearFactorGraph NonlinearGraph(const Sequence& sequence,
      const SharedNoiseModel& model) {
    NonlinearFactorGraph graph;
    Expression<Coefficients> c(0);
    typedef std::pair<double, double> Sample;
    BOOST_FOREACH(Sample sample, sequence) {
      Expression<double> expression(Fourier<3>(sample.first), c);
      ExpressionFactor<double> factor(model, sample.second, expression);
      graph.add(factor);
    }
    return graph;
  }

  /// Create linear FG from Sequence
  static GaussianFactorGraph::shared_ptr LinearGraph(const Sequence& sequence,
      const SharedNoiseModel& model) {
    NonlinearFactorGraph graph = NonlinearGraph(sequence, model);
    Values values;
    values.insert<Coefficients>(0, Coefficients::Zero()); // does not matter
    GaussianFactorGraph::shared_ptr gfg = graph.linearize(values);
    return gfg;
  }

  /// Constructor
  FourierDecomposition(const Sequence& sequence,
      const SharedNoiseModel& model) {
    GaussianFactorGraph::shared_ptr gfg = LinearGraph(sequence, model);
    VectorValues solution = gfg->optimize();
    c_ = solution.at(0);
  }

  /// Return Fourier coefficients
  Coefficients coefficients() {
    return c_;
  }
};

}

#include <gtsam_unstable/nonlinear/expressionTesting.h>
#include <gtsam/base/Testable.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

noiseModel::Diagonal::shared_ptr model = noiseModel::Unit::Create(1);

//******************************************************************************
TEST(BasisDecompositions, Fourier) {
  Fourier<3> fx(0);
  Eigen::Matrix<double, 1, 3> expectedH, actualH;
  Vector3 c(1.5661, 1.2717, 1.2717);
  expectedH = numericalDerivative11<double, Vector3>(
      boost::bind(&Fourier<3>::operator(), fx, _1, boost::none), c);
  EXPECT_DOUBLES_EQUAL(c[0]+c[1], fx(c,actualH), 1e-9);
  EXPECT(assert_equal((Matrix)expectedH, actualH));
}

//******************************************************************************
TEST(BasisDecompositions, ManualFourier) {

  // Create linear factor graph
  GaussianFactorGraph g;
  Key key(1);
  Expression<Vector3> c(key);
  Values values;
  values.insert<Vector3>(key, Vector3::Zero()); // does not matter
  for (size_t i = 0; i < 16; i++) {
    double x = i * M_PI / 8, y = exp(sin(x) + cos(x));

    // Manual JacobianFactor
    Matrix A(1, 3);
    A << 1, cos(x), sin(x);
    Vector b(1);
    b << y;
    JacobianFactor f1(key, A, b);
    g.add(f1);

    // With ExpressionFactor
    Expression<double> expression(Fourier<3>(x), c);
    EXPECT_CORRECT_EXPRESSION_JACOBIANS(expression, values, 1e-5, 1e-9);
    {
      ExpressionFactor<double> f2(model, y, expression);
      boost::shared_ptr<GaussianFactor> gf = f2.linearize(values);
      boost::shared_ptr<JacobianFactor> jf = //
          boost::dynamic_pointer_cast<JacobianFactor>(gf);
      CHECK(jf);
      EXPECT( assert_equal(f1, *jf, 1e-9));
    }
    {
      ExpressionFactor<double> f2(model, y, expression);
      boost::shared_ptr<GaussianFactor> gf = f2.linearize(values);
      boost::shared_ptr<JacobianFactor> jf = //
          boost::dynamic_pointer_cast<JacobianFactor>(gf);
      CHECK(jf);
      EXPECT( assert_equal(f1, *jf, 1e-9));
    }
    {
      ExpressionFactor<double> f2(model, y, expression);
      boost::shared_ptr<GaussianFactor> gf = f2.linearize(values);
      boost::shared_ptr<JacobianFactor> jf = //
          boost::dynamic_pointer_cast<JacobianFactor>(gf);
      CHECK(jf);
      EXPECT( assert_equal(f1, *jf, 1e-9));
    }
    {
      ExpressionFactor<double> f2(model, y, expression);
      boost::shared_ptr<GaussianFactor> gf = f2.linearize(values);
      boost::shared_ptr<JacobianFactor> jf = //
          boost::dynamic_pointer_cast<JacobianFactor>(gf);
      CHECK(jf);
      EXPECT( assert_equal(f1, *jf, 1e-9));
    }
  }

  // Solve
  VectorValues actual = g.optimize();

  // Check
  Vector3 expected(1.5661, 1.2717, 1.2717);
  EXPECT(assert_equal((Vector) expected, actual.at(key),1e-4));
}

//******************************************************************************
TEST(BasisDecompositions, FourierDecomposition) {

  // Create example sequence
  Sequence sequence;
  for (size_t i = 0; i < 16; i++) {
    double x = i * M_PI / 8, y = exp(sin(x) + cos(x));
    sequence[x] = y;
  }

  // Do Fourier Decomposition
  FourierDecomposition actual(sequence, model);

  // Check
  Vector3 expected(1.5661, 1.2717, 1.2717);
  EXPECT(assert_equal((Vector) expected, actual.coefficients(),1e-4));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

