/**
 * @file 	 testSQPSolver.cpp
 * @brief
 * @author Ivan Dario Jimenez
 * @date 	 Oct 26, 2013
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam_unstable/nonlinear/SQPLineSearch2.h>
#include <gtsam_unstable/nonlinear/NonlinearConstraint.h>
#include <gtsam_unstable/nonlinear/NonlinearInequalityConstraint.h>
#include <gtsam/inference/Symbol.h>

using namespace gtsam;

TEST(SQPLineSearch2, FailingQPWorkingGraph){
  Key X(Symbol('X',1)) , Y(Symbol('Y',1)), D(Symbol('D',1));
  GaussianFactorGraph::shared_ptr workingGraph(new GaussianFactorGraph());
  workingGraph->push_back(HessianFactor(X, Y, 37.9117*I_1x1, Z_1x1, -22.4673*I_1x1, 341.037*I_1x1, I_1x1*-606.025,1e6));
  workingGraph->push_back(HessianFactor(X, Y, -37.9054 *I_1x1, Z_1x1, Z_1x1, Z_1x1, Z_1x1,1e6));
//  workingGraph->push_back(JacobianFactor(X, -9.47942*I_1x1, Y, I_1x1, 0.285808*I_1x1));
  workingGraph->push_back(JacobianFactor(X, -9.47942*I_1x1, Y, I_1x1, 0.285808*I_1x1, noiseModel::Unit::Create(1)));
//  workingGraph->push_back(JacobianFactor(X, -9.47942*I_1x1, Y, I_1x1, 0.285808*I_1x1, noiseModel::Constrained::All(1)));
//  workingGraph->push_back(LinearEquality(X, -9.47942*I_1x1, Y, I_1x1, 0.285808*I_1x1,D));
  VectorValues result = workingGraph->optimize();
  GTSAM_PRINT(result);
  std::cout << JacobianFactor(X, -9.47942*I_1x1, Y, I_1x1, 0.285808*I_1x1, noiseModel::Unit::Create(1)).error(result) << std::endl;
}


/**
 * Nocedal06 Example 15.2 page 427
 * F(X) = x^2 + y^2
 * s.t. (x-1)^3 - y^2 = 0
 * with only solution and feasible point at (1,0)
 */

namespace Nocedal152 {

struct costError {
  Vector operator()(const double &x1, const double &x2,
      boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 =
          boost::none) const {
    return I_1x1 * (std::pow(x1, 2) + std::pow(x2, 2));
  }
};

struct constraintError {
  Vector operator()(const double &x1, const double &x2,
      boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 =
          boost::none) const {
    return I_1x1 * (std::pow(x1 - 1, 2) + std::pow(x2, 2));
  }
};

typedef NonlinearEqualityConstraint2<double, double, costError> cost;
typedef NonlinearEqualityConstraint2<double, double, constraintError> constraint;

}

TEST(SQPLineSearch2, TrivialNonlinearConstraintWithEqualities) {
  Key X(Symbol('X',1)) , Y(Symbol('Y',1)), D(Symbol('D',1));
  NP problem;
  problem.cost->push_back(Nocedal152::cost(X,Y,D));
  problem.equalities->push_back(Nocedal152::constraint(X,Y,D));
  Values expected;
  expected.insert(X, 1.0);
  expected.insert(Y, 0.0);
  SQPLineSearch2 solver(problem);
  Values actual = solver.optimize(expected);
  CHECK(assert_equal(expected, actual));
}

/**
 * min (x-1)^4 + (y-1)^4
 * s.t.  y - (x-1)^3 -1 = 0
 * start at (3,9) end at (1,1)
 */
namespace TrivialExample{
  
  struct costError{
    Vector operator()(const double & x1, const double & x2,
                      boost::optional<Matrix&> H1 = boost::none,
                      boost::optional<Matrix&> H2 = boost::none){
      if(H1){
        *H1 = 4*std::pow((x1 -1),3)*I_1x1;
      }
      if(H2){
        *H2 = 4*std::pow((x2 -1),3)*I_1x1;
      }
      return (std::pow(x1 -1, 4) + std::pow(x2-1, 4))*I_1x1;
    }
  };
  
  struct constraintError{
    Vector operator()(const double & x1, const double & x2,
      boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none){
      if(H1){
        *H1 = (-3*std::pow(x1-1,2))*I_1x1;
      }
      if(H2){
        *H2 = I_1x1;
      }
      return (x2 - std::pow(x1 - 1, 3) -1)*I_1x1;
    }
  };
  typedef NonlinearEqualityConstraint2<double,double, costError> cost;
  typedef NonlinearEqualityConstraint2<double,double, constraintError> constraint;
}

TEST(SQPLineSearch2, TrivialTest){
  Key X(Symbol('X',1)) , Y(Symbol('Y',1)), D(Symbol('D',1));
  NP problem;
  problem.cost->push_back(TrivialExample::cost(noiseModel::Unit::Create(1), X, Y, D));
  problem.equalities->push_back(TrivialExample::constraint(X,Y,D));
  Values initial, expected;
  initial.insert(X, 3.0);
  initial.insert(Y, 9.0);
  expected.insert(X, 1.0);
  expected.insert(Y, 1.0);
  SQPLineSearch2 solver(problem);
  Values actual = solver.optimize(initial);
  CHECK(assert_equal(expected, actual, 1e-7));
}

/**
 * Circle Example From AR-DRONE MPC
 * min x^2 + y^2
 * s.t (x-2)^2 + y^2 - 1 = 0
 * Start point x=2, y=1
 * With Solution x=1 y = 0
 */
namespace CircleExample {
struct costError {
  Vector operator()(const double & x1, const double & x2,
      boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 =
          boost::none) {
    if (H1) {
      *H1 = I_1x1 * 2 * x1;
    }
    if (H2) {
      *H2 = I_1x1 * 2 * x2;
    }
    return I_1x1 * (x1 * x1 + x2 * x2);
  }
};

struct constraintError {
  Vector operator()(const double & x1, const double & x2,
      boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 =
          boost::none) {
    if (H1) {
      *H1 = I_1x1 * 2 * (x1 - 2);
    }
    if (H2) {
      *H2 = I_1x1 * 2 * x2;
    }
    return I_1x1 * (std::pow((x1 - 2), 2) + x2 * x2 - 1);
  }
};
typedef NonlinearEqualityConstraint2<double, double, costError> cost;
typedef NonlinearEqualityConstraint2<double, double, constraintError> constraint;
}

TEST(SQPLineSearch2, CircleTest) {
  Key X(Symbol('X',1)) , Y(Symbol('Y',1)), D(Symbol('D',1));
  NP problem;
  problem.cost->push_back(CircleExample::cost(noiseModel::Unit::Create(1), X,Y,D));
  problem.equalities->push_back(CircleExample::constraint(X,Y,D));
  Values initial, expected;
  initial.insert(X, 2.0);
  initial.insert(Y, 1.0);
  expected.insert(X, 1.0);
  expected.insert(Y, 0.0);
  SQPLineSearch2 solver(problem);
  Values actual = solver.optimize(initial);
  CHECK(assert_equal(expected, actual, 1e-7))
}

/**
 * Trivial Equality Example
 * min x ^4 + y^4 - x^3 - 10*y^3 - 10*x^2 - y^2 - x - y
 * s.t.
 * y >= 1
 * y <= Exp[x]
 * x  <= 2
 *
 * Start point (1, 1.5) Solution at (2., 7.38906)
 */

namespace EasyEasyExample{
  
}

TEST_DISABLED(SQPLineSearch2, EasyEasyTest){
  
}

/**
 * Nocedal06 Problem 18.3 page 562
 * F(X) = e^(x1x2x3x4x5) + 0.5(x1^3 + x2^3 + 1)^2
 * s.t
 *  
 * x1^2 + x2^2 + x3^2 + x4^2 + x5^2 − 10 = 0
 * x2x3 − 5x4x5 = 0
 * x1^3 + x2^3 + 1 = 0
 * starting point on (−1.71, 1.59, 1.82, −0.763, −0.763)
 * with solution at (−1.8, 1.7, 1.9, −0.8, −0.8)
 */

namespace Nocedal183 {

struct costError {
  Vector operator()(const Vector5 &x,
      boost::optional<Matrix &> H1 = boost::none) const {
    if (H1) {
      /** Jacobian
       * [
       * 3*x1^2*(x1^3 + x2^3 + 1) + x2*x3*x4*x5*exp(x1*x2*x3*x4*x5),
       * 3*x2^2*(x1^3 + x2^3 + 1) + x1*x3*x4*x5*exp(x1*x2*x3*x4*x5),
       * x1*x2*x4*x5*exp(x1*x2*x3*x4*x5),
       * x1*x2*x3*x5*exp(x1*x2*x3*x4*x5),
       * x1*x2*x3*x4*exp(x1*x2*x3*x4*x5)
       * ]
       */
      *H1 = (Matrix(1, 5)
          << 3 * std::pow(x[0], 2) * (std::pow(x[0], 3) + std::pow(x[1], 3) + 1)
              + std::exp(x[0] * x[1] * x[2] * x[3] * x[4]) * x[1] * x[2] * x[3]
                  * x[4], 3 * std::pow(x[1], 2)
          * (std::pow(x[0], 3) + std::pow(x[1], 3) + 1)
          + std::exp(x[0] * x[1] * x[2] * x[3] * x[4]) * x[0] * x[2] * x[3]
              * x[4], std::exp(x[0] * x[1] * x[2] * x[3] * x[4]) * x[0] * x[1]
          * x[2] * x[4], std::exp(x[0] * x[1] * x[2] * x[3] * x[4]) * x[0]
          * x[1] * x[3] * x[4], std::exp(x[0] * x[1] * x[2] * x[3] * x[4])
          * x[0] * x[1] * x[2] * x[3]).finished();
    }
    return I_1x1
        * (std::exp(x[0] * x[1] * x[2] * x[3] * x[4])
            + 0.5 * std::pow(std::pow(x[0], 3) + std::pow(x[1], 3) + 1, 2));
  }
};
struct constraint1Error {
  Vector operator()(const Vector5 &x,
      boost::optional<Matrix &> H1 = boost::none) const {
    if (H1) {
      /**
       * [2x1, 2x2, 2x3, 2x4, 2x5]
       */
      *H1 =
          (Matrix(1, 5) << 2 * x[0], 2 * x[1], 2 * x[2], 2 * x[3], 2 * x[4]).finished();
    }
    return I_1x1 * (x.cwiseProduct(x).sum() - 10);
  }
};
struct constraint2Error {
  Vector operator()(const Vector5 &x,
      boost::optional<Matrix &> H1 = boost::none) const {
    if (H1) {
      /**
       * [0, x3, x2, -5x5, -5x4]
       */
      *H1 = (Matrix(1, 5) << 0, x[2], x[1], -5 * x[4], -5 * x[3]).finished();
    }
    return I_1x1 * (x[1] * x[2] - 5 * x[3] * x[4]);
  }
};
struct constraint3Error {
  Vector operator()(const Vector5 &x,
      boost::optional<Matrix &> H1 = boost::none) const {
    if (H1) {
      /**
       * [2x1^2, 2x2^2, 0, 0, 0]
       */
      *H1 =
          (Matrix(1, 5) << 3 * x[0] * x[0], 3 * x[1] * x[1], 0, 0, 0).finished();
    }
    return I_1x1 * (std::pow(x[0], 3) + std::pow(x[1], 3) + 1);
  }
};

typedef NonlinearEqualityConstraint1<Vector5, costError> cost;
typedef NonlinearEqualityConstraint1<Vector5, constraint1Error> constraint1;
typedef NonlinearEqualityConstraint1<Vector5, constraint2Error> constraint2;
typedef NonlinearEqualityConstraint1<Vector5, constraint3Error> constraint3;

}

TEST(SQPLineSearch2, CheckErrorMargin) {
  Key X(Symbol('X',1)), D(Symbol('D',0)), D1(Symbol('D',1)), D2(Symbol('D',2)), D3(Symbol('D',3));
  Values validValues;
  Vector5 validVector;
  validVector << -1.71, 1.59, 1.82, -0.763, -0.763;
  validValues.insert(X, validVector);
  CHECK(assert_equal(-0.071062, Nocedal183::constraint1(X,D1).unwhitenedError(validValues).sum()));
  CHECK(assert_equal(-0.017045, Nocedal183::constraint2(X,D2).unwhitenedError(validValues).sum()));
  CHECK(assert_equal(0.019467999999999999, Nocedal183::constraint3(X,D3).unwhitenedError(validValues).sum()));
  NonlinearCostFactorGraph fg;
  fg.push_back(Nocedal183::constraint1(X,D1));
  fg.push_back(Nocedal183::constraint2(X,D2));
  fg.push_back(Nocedal183::constraint3(X,D3));
  CHECK(assert_equal(0.107575, fg.error(validValues)));

}

TEST_DISABLED(SQPLineSearch2, NonlinearConstraintWithEqualities) {
  Key X(Symbol('X',1)), D(Symbol('D',0)), D1(Symbol('D',1)), D2(Symbol('D',2)), D3(Symbol('D',3));
  NP problem;
  problem.cost->push_back(Nocedal183::cost(noiseModel::Diagonal::Sigmas(Vector::Ones(1)),X,D));
  problem.equalities->push_back(Nocedal183::constraint1(X,D1));
  problem.equalities->push_back(Nocedal183::constraint2(X,D2));
  problem.equalities->push_back(Nocedal183::constraint3(X,D3));
  SQPLineSearch2 solver(problem);
  Values expected, initial;
  Vector5 expectedVector, initialVector;
  expectedVector << -1.8 , 1.7, 1.9 , -0.8 , -0.8;
  initialVector << -1.71, 1.59, 1.82, -0.763, -0.763;
  expected.insert(X, expectedVector);
  initial.insert(X, initialVector);
  double equalityError;
  CHECK(!solver.checkFeasibility(initial, equalityError));
  std::cout << "Equality Error: " << equalityError << std::endl;
  Values actuals = solver.optimize(initial);
  CHECK(assert_equal(expected,actuals, 1e-7));
}

TEST_DISABLED(SQPLineSearch2, FeasabilityEqualities) {
  Key X(Symbol('X',1)), D(Symbol('D',1));
  NP problem;
  problem.cost->push_back(Nocedal183::cost(X,D));
  problem.equalities->push_back(Nocedal183::constraint1(X,D));
  problem.equalities->push_back(Nocedal183::constraint2(X,D));
  problem.equalities->push_back(Nocedal183::constraint3(X,D));
  SQPLineSearch2 solver(problem);
  Values initial;
  Vector5 initialVector;
  initialVector << -1.71, 1.59, 1.82, -0.763, -0.763;
  initial.insert(X, initialVector);
  CHECK(solver.checkFeasibility(initial));
}

/**
 * Betts10 sample problem page 30:
 * F(X) = x1^2 + x2^2 + log(x1*x2)
 * c1(x) = x1*x2 >= 1
 * 0 <= x1 <= 10
 * 0 <= x2 <= 10
 * starting point on (0.5,2) with solution at (1,1)
 */

namespace Betts10_30 {
struct costError {
  Vector operator()(const double &x1, const double &x2,
      boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 =
          boost::none) const {
    return I_1x1 * (std::pow(x1, 2) + std::pow(x2, 2) + std::log(x1 * x2));
  }
};

struct constraintError {
  Vector operator()(const double &x1, const double &x2,
      boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 =
          boost::none) const {
    return I_1x1 * (1 - x1 * x2);
  }
};

struct lowerBoundXError {
  Vector operator()(const double &x,
      boost::optional<Matrix &> H1 = boost::none) const {
    return I_1x1 * (-x);
  }
};

struct upperBoundXError {
  Vector operator()(const double &x,
      boost::optional<Matrix &> H1 = boost::none) const {
    return I_1x1 * (x - 10);
  }
};

typedef NonlinearEqualityConstraint2<double, double, costError> cost;
typedef NonlinearInequalityConstraint2<double, double, constraintError> constraint;
typedef NonlinearInequalityConstraint1<double, lowerBoundXError> lowerBoundX;
typedef NonlinearInequalityConstraint1<double, upperBoundXError> upperBoundX;
}

TEST_DISABLED(SQPLineSearch2, NonlinearConstraintWithInequalities) {
  Key X(Symbol('X',1)), Y(Symbol('Y',1)), dk(Symbol('D', 1));
  NP problem;
  problem.cost->push_back(Betts10_30::cost(X, Y, dk));
  problem.inequalities->push_back(Betts10_30::constraint(X,Y,dk));
  problem.inequalities->push_back(Betts10_30::lowerBoundX(X, dk));
  problem.inequalities->push_back(Betts10_30::lowerBoundX(Y, dk));
  problem.inequalities->push_back(Betts10_30::upperBoundX(X, dk));
  problem.inequalities->push_back(Betts10_30::upperBoundX(Y, dk));
  SQPLineSearch2 solver(problem);
  Values vals;
  vals.insert(X, 0.5);
  vals.insert(Y, 2.0);
  Values result = solver.optimize(vals);
  Values expectedResult;
  expectedResult.insert(X, 1.0);
  expectedResult.insert(Y, 1.0);
  CHECK(assert_equal(expectedResult, result, 1e-7));
}

/**
 * Betts10  Example 1.5 page 32 (changed to fit the internal code conventions)
 * F(X) = x1^2 + x2^2
 * s.t.
 * c1(x) = 2 - x1 - x2 <= 0
 * c2(x) = x1 + (2/3)x2 - 4 <= 0
 * starting point on (4,0) solution (1,1) and optimal lambda of (2,0)
 * Optimal active set is {c1}
 */

namespace Betts10_32 {
struct costError {
  Vector operator()(const double &x1, const double &x2,
      boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 =
          boost::none) const {
    return I_1x1 * (std::pow(x1, 2) + std::pow(x2, 2));
  }
};
struct constraint1Error {
  Vector operator()(const double &x1, const double &x2,
      boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 =
          boost::none) const {
    return I_1x1 * (2 - x1 - x2);
  }
};
struct constraint2Error {
  Vector operator()(const double &x1, const double &x2,
      boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 =
          boost::none) const {
    return I_1x1 * (x1 + (2.0 / 3.0) * x2 - 4);
  }
};

typedef NonlinearEqualityConstraint2<double, double, costError> cost;
typedef NonlinearInequalityConstraint2<double, double, constraint1Error> constraint1;
typedef NonlinearInequalityConstraint2<double, double, constraint2Error> constraint2;
}

TEST_DISABLED(SQPLineSearch2, NonlinearConstraintWithEqualitiesOnly) {
  Key X(Symbol('X',1)), Y(Symbol('Y', 1)), D(Symbol('D',1));
  NP problem;
  problem.cost->push_back(Betts10_32::cost(X,Y,D));
  problem.inequalities->push_back(Betts10_32::constraint1(X,Y,D));
  problem.inequalities->push_back(Betts10_32::constraint2(X,Y,D));

  SQPLineSearch2 solver(problem);
  Values inits;
  inits.insert(X, 4.0);
  inits.insert(Y, 0.0);
  Values result = solver.optimize(inits);
  Values expectedResult;
  expectedResult.insert(X, 1.0);
  expectedResult.insert(Y, 1.0);
  CHECK(assert_equal(expectedResult, result, 1e-7));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
