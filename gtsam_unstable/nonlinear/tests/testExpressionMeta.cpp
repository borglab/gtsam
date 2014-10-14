/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testExpressionMeta.cpp
 * @date October 14, 2014
 * @author Frank Dellaert
 * @brief Test meta-programming constructs for Expressions
 */

#include <gtsam_unstable/nonlinear/ExpressionFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/base/Testable.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
namespace mpl = boost::mpl;

#include <boost/mpl/assert.hpp>
#include <boost/mpl/equal.hpp>
template<class T> struct Incomplete;

// Check generation of FunctionalNode
typedef mpl::vector<Pose3, Point3> MyTypes;
typedef FunctionalNode<Point2, MyTypes>::type Generated;
//Incomplete<Generated> incomplete;
BOOST_MPL_ASSERT((boost::is_same< Matrix2, Generated::Record::Jacobian2T >));

// Try generating vectors of ExecutionTrace
typedef mpl::vector<ExecutionTrace<Pose3>, ExecutionTrace<Point3> > ExpectedTraces;

typedef mpl::transform<MyTypes, ExecutionTrace<MPL::_1> >::type MyTraces;
BOOST_MPL_ASSERT((mpl::equal< ExpectedTraces, MyTraces >));

template<class T>
struct MakeTrace {
  typedef ExecutionTrace<T> type;
};
typedef mpl::transform<MyTypes, MakeTrace<MPL::_1> >::type MyTraces1;
BOOST_MPL_ASSERT((mpl::equal< ExpectedTraces, MyTraces1 >));

// Try generating vectors of Expression types
typedef mpl::vector<Expression<Pose3>, Expression<Point3> > ExpectedExpressions;
typedef mpl::transform<MyTypes, Expression<MPL::_1> >::type Expressions;
BOOST_MPL_ASSERT((mpl::equal< ExpectedExpressions, Expressions >));

// Try generating vectors of Jacobian types
typedef mpl::vector<Matrix26, Matrix23> ExpectedJacobians;
typedef mpl::transform<MyTypes, Jacobian<Point2, MPL::_1> >::type Jacobians;
BOOST_MPL_ASSERT((mpl::equal< ExpectedJacobians, Jacobians >));

// Try accessing a Jacobian
typedef mpl::at_c<Jacobians, 1>::type Jacobian23; // base zero !
BOOST_MPL_ASSERT((boost::is_same< Matrix23, Jacobian23>));

/* ************************************************************************* */
// Boost Fusion includes allow us to create/access values from MPL vectors
#include <boost/fusion/include/mpl.hpp>
#include <boost/fusion/include/at_c.hpp>

// Create a value and access it
TEST(ExpressionFactor, JacobiansValue) {
  using boost::fusion::at_c;
  Matrix23 expected;
  Jacobians jacobians;

  at_c<1>(jacobians) << 1, 2, 3, 4, 5, 6;

  Matrix23 actual = at_c<1>(jacobians);
  CHECK(actual.cols() == expected.cols());
  CHECK(actual.rows() == expected.rows());
}

/* ************************************************************************* */
// Test out polymorphic transform

#include <boost/fusion/include/make_vector.hpp>
#include <boost/fusion/include/transform.hpp>
#include <boost/utility/result_of.hpp>

struct triple {
  template<class> struct result; // says we will provide result

  template<class F>
  struct result<F(int)> {
      typedef double type; // result for int argument
  };

  template<class F>
  struct result<F(const int&)> {
      typedef double type; // result for int argument
  };

  template<class F>
  struct result<F(const double &)> {
      typedef double type; // result for double argument
  };

  template<class F>
  struct result<F(double)> {
      typedef double type; // result for double argument
  };

  // actual function
  template<typename T>
  typename result<triple(T)>::type operator()(const T& x) const {
    return (double) x;
  }
};

TEST(ExpressionFactor, Triple) {
  typedef boost::fusion::vector<int, double> IntDouble;
  IntDouble H = boost::fusion::make_vector(1, 2.0);

  // Only works if I use Double2
  typedef boost::fusion::result_of::transform<IntDouble, triple>::type Result;
  typedef boost::fusion::vector<double, double> Double2;
  Double2 D = boost::fusion::transform(H, triple());

  using boost::fusion::at_c;
  DOUBLES_EQUAL(1.0, at_c<0>(D), 1e-9);
  DOUBLES_EQUAL(2.0, at_c<1>(D), 1e-9);
}

/* ************************************************************************* */
#include <boost/fusion/include/invoke.hpp>
#include <boost/functional/value_factory.hpp>

// Test out polymorphic transform
TEST(ExpressionFactor, Invoke) {
  std::plus<int> add;
  assert(invoke(add,boost::fusion::make_vector(1,1)) == 2);

  // Creating a Pose3 (is there another way?)
  boost::fusion::vector<Rot3, Point3> pair;
  Pose3 pose = boost::fusion::invoke(boost::value_factory<Pose3>(), pair);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

