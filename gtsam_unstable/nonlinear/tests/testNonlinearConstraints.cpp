/**
 * @file 	 testNonlinearConstraints.cpp
 * @brief  Tests Nonlinear Equality and Inequality Constraints
 * @author Duy-Nguyen Ta
 * @author Ivan Dario Jimenez
 * @date 	 Oct 26, 2013
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_unstable/nonlinear/NonlinearConstraint.h>
#include <gtsam_unstable/nonlinear/NonlinearInequalityConstraint.h>

using namespace gtsam;
using namespace gtsam::symbol_shorthand;
using namespace std;

/**
 * 2x1 <= 33
 */
struct TrivialInequalityError1 {
  Vector operator() (const double &x, boost::optional<Matrix &> H1) const{
    if(H1){
      *H1 = (Matrix(1,1) << 2.0).finished();
    }
    return I_1x1 * (2 * x - 33);
  }
};

typedef NonlinearInequalityConstraint1<double, TrivialInequalityError1> TrivialInequality1;

TEST(NonlinearINequalityConstraint, TrivialInequality1){
  Key X(Symbol('X',1)), D(Symbol('D', 1));
  
  TrivialInequality1 inequality(X,D);
  CHECK(0.0 == inequality.evaluateError(2.0)[0]);
  CHECK(-29 == inequality.equality_.evaluateError(2.0)[0]);
}


/**
 * 2x1 + sin(x2) <= 11
 */

struct TrivialInequalityError2{
  Vector operator() (const double &x1, const double &x2, boost::optional<Matrix &> H1, boost::optional<Matrix &> H2){
    if(H1){
      *H1 = (Matrix(1,1) << 2.0).finished();
    }
    
    if(H2){
      *H2 = (Matrix(1,1) << std::cos(x2)).finished();
    }
    
    return I_1x1 * (2*x1 + std::sin(x2) - 11);
  }
};

typedef NonlinearInequalityConstraint2<double, double, TrivialInequalityError2> TrivialInequality2;

TEST(NonlinearInequalityConstraint, TrivialInequality2){
  Key X1(Symbol('X',1)), X2(Symbol('X',2)), D(Symbol('D', 1));
  
  TrivialInequality2 inequality(X1, X2, D);
  CHECK(0.0 == inequality.evaluateError(5.5,0)[0]);
  CHECK(0.0 == inequality.equality_.evaluateError(5.5,0)[0]);
}

/**
 * 2x1 + 3x2^2 + x3^3 <= 123
 */

struct TrivialInequalityError3{
  Vector operator()(const double &x1, const double &x2, const double &x3,
      boost::optional<Matrix&> H1, boost::optional<Matrix&> H2,
      boost::optional<Matrix&> H3) {
    if(H1){
      *H1 = (Matrix(1,1) << 2).finished();
    }
    
    if(H2){
      *H2 = (Matrix(1,1) << 6 * x2).finished();
    }
    
    if(H3){
      *H3 = (Matrix(1,1) << 3 * x3 * x3).finished();
    }
    
    return I_1x1 * (2*x1 + 3*std::pow(x2,2) + std::pow(x3,3) - 123);
  }
};

typedef NonlinearInequalityConstraint3<double, double, double, TrivialInequalityError3> TrivialInequality3;

TEST(NonlinearInequalityConstraint, TrivialInequality3){
  Key X1(Symbol('X',1)), X2(Symbol('X',2)), X3(Symbol('X',3)), D(Symbol('D', 1));
   
  TrivialInequality3 inequality(X1, X2, X3, D);
  CHECK(666 == inequality.evaluateError(6.0, 4.0, 9.0)[0]);
  CHECK(666 == inequality.equality_.evaluateError(6.0, 4.0, 9.0)[0]);
}

TEST(Vector, vectorize) {
  Matrix m = (Matrix(4,3) << 1,2,3,4,5,6,7,8,9,10,11,12).finished();
  Matrix m2 = m.transpose();
  m2.resize(12,1);
  Vector v = Vector(m2);
  Vector expectedV = (Vector(12) << 1,2,3,4,5,6,7,8,9,10,11,12).finished();
  EXPECT(assert_equal(expectedV, v));
}

/* ************************************************************************* */
/**
 * Test 3-way equality nonlinear constraint
 *  x(1)      + x(2)^2 + x(3)^3                 = 3
 */
struct ConstraintError{
  Vector operator() (const double& x1, const double& x2, const double& x3,
                       boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 =
  boost::none, boost::optional<Matrix&> H3 = boost::none) const {
    if (H1) {
      *H1 = (Matrix(1, 1) << 1.0).finished();
    }
    if (H2) {
      *H2 = (Matrix(1, 1) << 2.0 * x2).finished();
    }
    if (H3) {
      *H3 = (Matrix(1, 1) << 3.0 * x3 * x3).finished();
    }
    
    return (Vector(1) << x1 + x2 * x2 + x3 * x3 * x3 - 3.0).finished();
  }
};

class Constraint: public NonlinearEqualityConstraint3<double, double, double, ConstraintError> {
  typedef NonlinearEqualityConstraint3<double, double, double, ConstraintError> Base;
public:
  Constraint(Key key1, Key key2, Key key3, Key dualKey) :
      Base(key1, key2, key3, dualKey, 1) {
  }

  void expectedHessians(const double& x1, const double& x2, const double& x3,
      std::vector<Matrix>& G11, std::vector<Matrix>& G12,
      std::vector<Matrix>& G13, std::vector<Matrix>& G22,
      std::vector<Matrix>& G23, std::vector<Matrix>& G33) const {
    G11.push_back(zeros(1, 1));
    G12.push_back(zeros(1, 1));
    G13.push_back(zeros(1, 1));
    G22.push_back((Matrix(1, 1) << 2.0).finished());
    G23.push_back(zeros(1, 1));
    G33.push_back((Matrix(1, 1) << 6.0 * x3).finished());
  }
};

/* ************************************************************************* */

TEST(NonlinearConstraint3, Hessian) {
  double x1(2.0), x2(sqrt(2) - 1), x3(sqrt(2) - 1);
  Constraint constraint(X(1), X(2), X(3), 0);

  std::vector<Matrix> expectedG11, expectedG12, expectedG13, expectedG22,
  expectedG23, expectedG33;
  constraint.expectedHessians(x1, x2, x3, expectedG11, expectedG12, expectedG13,
      expectedG22, expectedG23, expectedG33);
  std::vector<Matrix> G11, G12, G13, G22, G23, G33;
  constraint.evaluateHessians(x1, x2, x3, G11, G12, G13, G22, G23, G33);
  EXPECT(assert_equal(expectedG11[0], G11[0], 1e-5));
  EXPECT(assert_equal(expectedG12[0], G12[0], 1e-5));
  EXPECT(assert_equal(expectedG13[0], G13[0], 1e-5));
  EXPECT(assert_equal(expectedG22[0], G22[0], 1e-5));
  EXPECT(assert_equal(expectedG23[0], G23[0], 1e-5));
  EXPECT(assert_equal(expectedG33[0], G33[0], 1e-5));
}

/* ************************************************************************* */
/**
 * Test 3-way nonlinear equality multi-values constraint
 *  p(1).x      + p(2).x^2 + p(3).y^3                 = 3
 *  p(1).x^2    + p(2).y   + p(3).x^3                 = 5
 */
struct Constraint2dError{
  Vector operator() (const Point2& p1, const Point2& p2, const Point2& p3,
                       boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 =
  boost::none, boost::optional<Matrix&> H3 = boost::none) const {
    if (H1) {
      *H1 = (Matrix(2, 2) << 1.0, 0.0, 2 * p1.x(), 0.0).finished();
    }
    if (H2) {
      *H2 = (Matrix(2, 2) << 2.0 * p2.x(), 0.0, 0.0, 1.0).finished();
    }
    if (H3) {
      *H3 =
        (Matrix(2, 2) << 0.0, 3.0 * p3.y() * p3.y(), 3.0 * p3.x() * p3.x(), 0.0).finished();
    }
    
    return (Vector(2)
      << p1.x() + p2.x() * p2.x() + p3.y() * p3.y() * p3.y() - 3.0, p1.x()
                                                                    * p1.x() + p2.y() + p3.x() * p3.x() * p3.x() - 5.0).finished();
  }
};

class Constraint2d: public NonlinearEqualityConstraint3<Point2, Point2, Point2, Constraint2dError> {
  typedef NonlinearEqualityConstraint3<Point2, Point2, Point2, Constraint2dError> Base;

public:
  Constraint2d(Key key1, Key key2, Key key3, Key dualKey) :
      Base(key1, key2, key3, dualKey, 1) {
  }

  void expectedHessians(const Point2& x1, const Point2& x2, const Point2& x3,
      std::vector<Matrix>& G11, std::vector<Matrix>& G12,
      std::vector<Matrix>& G13, std::vector<Matrix>& G22,
      std::vector<Matrix>& G23, std::vector<Matrix>& G33) const {
    G11.push_back(zeros(2, 2));
    G11.push_back((Matrix(2, 2) << 2.0, 0.0, 0.0, 0.0).finished());

    G12.push_back(zeros(2, 2));
    G12.push_back(zeros(2, 2));

    G13.push_back(zeros(2, 2));
    G13.push_back(zeros(2, 2));

    G22.push_back((Matrix(2, 2) << 2.0, 0.0, 0.0, 0.0).finished());
    G22.push_back(zeros(2, 2));

    G23.push_back(zeros(2, 2));
    G23.push_back(zeros(2, 2));

    G33.push_back((Matrix(2, 2) << 0.0, 0.0, 0.0, 6.0 * x3.y()).finished());
    G33.push_back((Matrix(2, 2) << 6.0 * x3.x(), 0.0, 0.0, 0.0).finished());
  }
};

/* ************************************************************************* */

TEST(NonlinearConstraint3, Hessian2) {
  Point2 x1(2.0, 2.0), x2(sqrt(2) - 1, 3.0), x3(4.0, sqrt(2) - 2);
  Constraint2d constraint(X(1), X(2), X(3), 0);

  std::vector<Matrix> expectedG11, expectedG12, expectedG13, expectedG22,
  expectedG23, expectedG33;
  constraint.expectedHessians(x1, x2, x3, expectedG11, expectedG12, expectedG13,
      expectedG22, expectedG23, expectedG33);
  std::vector<Matrix> G11, G12, G13, G22, G23, G33;
  constraint.evaluateHessians(x1, x2, x3, G11, G12, G13, G22, G23, G33);
  for (size_t i = 0; i<G11.size(); ++i) {
    EXPECT(assert_equal(expectedG11[i], G11[i], 1e-5));
    EXPECT(assert_equal(expectedG12[i], G12[i], 1e-5));
    EXPECT(assert_equal(expectedG13[i], G13[i], 1e-5));
    EXPECT(assert_equal(expectedG22[i], G22[i], 1e-5));
    EXPECT(assert_equal(expectedG23[i], G23[i], 1e-5));
    EXPECT(assert_equal(expectedG33[i], G33[i], 1e-5));
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
