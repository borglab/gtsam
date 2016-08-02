#include <CppUnitLite/TestHarness.h>
#include <gtsam_unstable/nonlinear/SQPLineSearch2.h>
#include <gtsam_unstable/nonlinear/NonlinearEqualityConstraint.h>
#include <gtsam_unstable/nonlinear/NonlinearInequalityConstraint.h>
#include <gtsam/inference/Symbol.h>

using namespace gtsam;
/**
 * Nocedal06 Example 15.2 page 427
 * F(X) = x^2 + y^2
 * s.t. (x-1)^3 - y^2 = 0
 * with only solution and feasible point at (1,0)
 */

namespace Nocedal152 {
class cost: public NonlinearEqualityConstraint2<double, double> {
public:
  cost(Key j1, Key j2, Key dualKey) :
      NonlinearEqualityConstraint2(j1, j2, dualKey) {
  }

private:
  virtual Vector evaluateError(const X1 &x1, const X2 &x2,
      boost::optional<Matrix &> H1, boost::optional<Matrix &> H2) const
          override {
    return Vector1(std::pow(x1, 2) + std::pow(x2, 2));
  }
};

class constraint: public NonlinearEqualityConstraint2<double, double> {
public:
  constraint(Key j1, Key j2, Key dualKey) :
      NonlinearEqualityConstraint2(j1, j2, dualKey) {
  }

private:
  virtual Vector evaluateError(const X1 &x1, const X2 &x2,
      boost::optional<Matrix &> H1, boost::optional<Matrix &> H2) const
          override {
    return Vector1(std::pow(x1 - 1, 2) + std::pow(x2, 2));
  }
};
}

TEST(SQPLineSearch2, TrivialNonlinearConstraintWithEqualities) {
  Key X(Symbol('X',1)) , Y(Symbol('Y',1)), D(Symbol('D',1));
  NP problem;
  problem.cost.push_back(Nocedal152::cost(X,Y,D));
  problem.equalities.push_back(Nocedal152::constraint(X,Y,D));
  Values expected;
  expected.insert(X, 1.0);
  expected.insert(Y, 0.0);
  SQPLineSearch2 solver(problem);
  Values actual = solver.optimize(expected);
  CHECK(assert_equal(expected, actual));
}

/**
 * Nocedal06 Problem 18.3 page 562
 * F(X) = e^(x1x2x3x4x5) + 0.5(x1^3 + x2^3 + 1)^2
 * s.t
 * x1^2 + x2^2 + x3^2 + x4^2 + x5^2 − 10 = 0
 * x2x3 − 5x4x5 = 0
 * x1^3 + x2^3 + 1 = 0
 * starting point on (−1.71, 1.59, 1.82, −0.763, −0.763)
 * with solution at (−1.8, 1.7, 1.9, −0.8, −0.8)
 */

namespace Nocedal183 {
class cost: public NonlinearEqualityConstraint1<Vector5> {
public:
  cost(Key key, Key dualKey) :
      NonlinearEqualityConstraint1(key, dualKey) {
  }

private:
  virtual Vector evaluateError(const X &x, boost::optional<Matrix &> H1) const
      override {
    return Vector1(
        std::exp(x[0] * x[1] * x[2] * x[3] * x[4])
            + 0.5 * std::pow(std::pow(x[0], 3) + std::pow(x[1], 3) + 1, 2));
  }
};

class constraint1: public NonlinearEqualityConstraint1<Vector5> {
public:
  constraint1(Key key, Key dualKey) :
      NonlinearEqualityConstraint1(key, dualKey) {
  }

private:
  virtual Vector evaluateError(const X &x, boost::optional<Matrix &> H1) const
      override {
    return Vector1(x.cwiseProduct(x).sum() - 10);
  }
};

class constraint2: public NonlinearEqualityConstraint1<Vector5> {
public:
  constraint2(Key key, Key dualKey) :
      NonlinearEqualityConstraint1(key, dualKey) {
  }

private:
  virtual Vector evaluateError(const X &x, boost::optional<Matrix &> H1) const
      override {
    return Vector1(x[1] * x[2] - 5 * x[3] * x[4]);
  }
};

class constraint3: public NonlinearEqualityConstraint1<Vector5> {
public:
  constraint3(Key key, Key dualKey) :
      NonlinearEqualityConstraint1(key, dualKey) {
  }

private:
  virtual Vector evaluateError(const X &x, boost::optional<Matrix &> H1) const
      override {
    return Vector1(std::pow(x[0], 3) + std::pow(x[1], 3) + 1);
  }
};
}

TEST(SQPLineSearch2, NonlinearConstraintWithEqualities) {
  Key X(Symbol('X',1)), D(Symbol('D',1));
  NP problem;
  problem.cost.push_back(Nocedal183::cost(X,D));
  problem.equalities.push_back(Nocedal183::constraint1(X,D));
  problem.equalities.push_back(Nocedal183::constraint2(X,D));
  problem.equalities.push_back(Nocedal183::constraint3(X,D));
  SQPLineSearch2 solver(problem);
  Values expected, initial;
  Vector5 expectedVector, initialVector;
  expectedVector << -1.8 , 1.7, 1.9 , -0.8 , -0.8;
  initialVector << -1.71, 1.59, 1.82, -0.763, -0.763;
  expected.insert(X, expectedVector);
  initial.insert(X, initialVector);
  Values actuals = solver.optimize(initial);
  CHECK(assert_equal(expected,actuals, 1e-7));
}

/**
 * Betts10 sample problem page 30:
 * F(X) = x1^2 + x2^2 + log(x1*x2)
 * c1(x) = x1*x2 >= 1
 * 0 <= x1 <= 10
 * 0 <= x2 <= 10
 * starting point on (0.5,2) with solution at (1,1)
 */

class cost: public NonlinearEqualityConstraint2<double, double> {
public:
  cost(Key j1, Key j2, Key dualKey) :
      NonlinearEqualityConstraint2(j1, j2, dualKey, 1) {
  }

private:
  virtual Vector evaluateError(const X1 &x1, const X2 &x2,
      boost::optional<Matrix &> H1, boost::optional<Matrix &> H2) const
          override {
    return Vector1(std::pow(x1, 2) + std::pow(x2, 2) + std::log(x1 * x2));
  }
};

class constraint: public NonlinearInequalityConstraint2<double, double> {
public:
  constraint(Key j1, Key j2, Key dualKey) :
      NonlinearInequalityConstraint2(j1, j2, dualKey) {
  }

private:
  virtual double computeError(const X1 &x1, const X2 &x2,
      boost::optional<Matrix &> H1, boost::optional<Matrix &> H2) const
          override {
    return 1 - x1 * x2;
  }
};

class lowerBoundX: public NonlinearInequalityConstraint1<double> {
public:
  lowerBoundX(Key key, Key dualKey) :
      NonlinearInequalityConstraint1(key, dualKey) {
  }

private:
  virtual double computeError(const X &x, boost::optional<Matrix &> H1) const
      override {
    return -x;
  }
};

class upperBoundX: public NonlinearInequalityConstraint1<double> {
public:
  upperBoundX(Key key, Key dualKey) :
      NonlinearInequalityConstraint1(key, dualKey) {
  }

private:
  virtual double computeError(const X &x, boost::optional<Matrix &> H1) const
      override {
    return x - 10;
  }
};

TEST(SQPLineSearch2, NonlinearConstraintWithInequalities) {
  Key X(Symbol('X',1)), Y(Symbol('Y',1)), dk(Symbol('D', 1));
  NP problem;
  problem.cost.push_back(cost(X, Y, dk));
  problem.inequalities.push_back(constraint(X,Y,dk));
  problem.inequalities.push_back(lowerBoundX(X, dk));
  problem.inequalities.push_back(lowerBoundX(Y, dk));
  problem.inequalities.push_back(upperBoundX(X, dk));
  problem.inequalities.push_back(upperBoundX(Y, dk));
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
class cost2: public NonlinearEqualityConstraint2<double, double> {
public:
  cost2(Key j1, Key j2, Key dualKey) :
      NonlinearEqualityConstraint2(j1, j2, dualKey, 1) {
  }

  virtual Vector evaluateError(const X1 &x1, const X2 &x2,
      boost::optional<Matrix &> H1, boost::optional<Matrix &> H2) const
          override {
    return Vector1(std::pow(x1, 2) + std::pow(x2, 2));
  }
};

class c1: public NonlinearInequalityConstraint2<double, double> {
public:
  c1(Key j1, Key j2, Key dualKey) :
      NonlinearInequalityConstraint2(j1, j2, dualKey) {
  }

  virtual double computeError(const X1 &x1, const X2 &x2,
      boost::optional<Matrix &> H1, boost::optional<Matrix &> H2) const
          override {
    return 2 - x1 - x2;
  }
};

class c2: public NonlinearInequalityConstraint2<double, double> {
public:
  c2(Key j1, Key j2, Key dualKey) :
      NonlinearInequalityConstraint2(j1, j2, dualKey) {
  }

  virtual double computeError(const X1 &x1, const X2 &x2,
      boost::optional<Matrix &> H1, boost::optional<Matrix &> H2) const
          override {
    return x1 + (2.0 / 3.0) * x2 - 4;
  }
};
TEST(SQPLineSearch2, NonlinearConstraintWithEqualitiesOnly) {
  Key X(Symbol('X',1)), Y(Symbol('Y', 1)), D(Symbol('D',1));
  NP problem;
  problem.cost.push_back(cost2(X,Y,D));
  problem.inequalities.push_back(c1(X,Y,D));
  problem.inequalities.push_back(c2(X,Y,D));

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
