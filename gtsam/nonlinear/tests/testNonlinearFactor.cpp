/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testDynamicValues.cpp
 * @author Duy-Nguyen Ta
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/LieVector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/Key.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
typedef TypedSymbol<Pose2, 'p'> PoseKey;

/* ************************************************* */
class TestFactor1: public NonlinearFactor1<PoseKey> {
	typedef NonlinearFactor1<PoseKey> Base;
public:
	TestFactor1() : Base(sharedSigmas(Vector_(1, 1.0)), 1) {}
	virtual ~TestFactor1() {}

	virtual Vector evaluateError(const Pose2& pose, boost::optional<Matrix&> H = boost::none) const {
		if (H) *H = zeros(1,3);
		return zero(1);
	}
};

/* ************************************************************************* */
TEST(NonlinearFactor, TestFactor1) {
	TestFactor1 factor1;
	Vector actualError = factor1.evaluateError(Pose2());
	Vector expectedError = zero(1);
	CHECK(assert_equal(expectedError, actualError));
}

/* ************************************************************************* */
typedef TypedSymbol<LieVector, 'x'> TestKey;

/* ************************************************************************* */
class TestFactor4 : public NonlinearFactor4<TestKey, TestKey, TestKey, TestKey> {
public:
  typedef NonlinearFactor4<TestKey, TestKey, TestKey, TestKey> Base;
  TestFactor4() : Base(sharedSigmas(Vector_(1, 2.0)), 1, 2, 3, 4) {}
  virtual ~TestFactor4() {}

  virtual Vector
    evaluateError(const LieVector& x1, const LieVector& x2, const LieVector& x3, const LieVector& x4,
        boost::optional<Matrix&> H1 = boost::none,
        boost::optional<Matrix&> H2 = boost::none,
        boost::optional<Matrix&> H3 = boost::none,
        boost::optional<Matrix&> H4 = boost::none) const {
    if(H1) {
      *H1 = Matrix_(1,1, 1.0);
      *H2 = Matrix_(1,1, 2.0);
      *H3 = Matrix_(1,1, 3.0);
      *H4 = Matrix_(1,1, 4.0);
    }
    return (Vector(1) << x1 + x2 + x3 + x4).finished();
  }
};

/* ************************************ */
TEST(NonlinearFactor, NonlinearFactor4) {
  TestFactor4 tf;
  Values tv;
  tv.insert(TestKey(1), LieVector(1, 1.0));
  tv.insert(TestKey(2), LieVector(1, 2.0));
  tv.insert(TestKey(3), LieVector(1, 3.0));
  tv.insert(TestKey(4), LieVector(1, 4.0));
  EXPECT(assert_equal(Vector_(1, 10.0), tf.unwhitenedError(tv)));
  DOUBLES_EQUAL(25.0/2.0, tf.error(tv), 1e-9);
  Ordering ordering; ordering += TestKey(1), TestKey(2), TestKey(3), TestKey(4);
  JacobianFactor jf(*boost::dynamic_pointer_cast<JacobianFactor>(tf.linearize(tv, ordering)));
  LONGS_EQUAL(jf.keys()[0], 0);
  LONGS_EQUAL(jf.keys()[1], 1);
  LONGS_EQUAL(jf.keys()[2], 2);
  LONGS_EQUAL(jf.keys()[3], 3);
  EXPECT(assert_equal(Matrix_(1,1, 0.5), jf.getA(jf.begin())));
  EXPECT(assert_equal(Matrix_(1,1, 1.0), jf.getA(jf.begin()+1)));
  EXPECT(assert_equal(Matrix_(1,1, 1.5), jf.getA(jf.begin()+2)));
  EXPECT(assert_equal(Matrix_(1,1, 2.0), jf.getA(jf.begin()+3)));
  EXPECT(assert_equal(Vector_(1, -5.0), jf.getb()));
}

/* ************************************************************************* */
class TestFactor5 : public NonlinearFactor5<TestKey, TestKey, TestKey, TestKey, TestKey> {
public:
  typedef NonlinearFactor5<TestKey, TestKey, TestKey, TestKey, TestKey> Base;
  TestFactor5() : Base(sharedSigmas(Vector_(1, 2.0)), 1, 2, 3, 4, 5) {}
  virtual ~TestFactor5() {}

  virtual Vector
    evaluateError(const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5,
        boost::optional<Matrix&> H1 = boost::none,
        boost::optional<Matrix&> H2 = boost::none,
        boost::optional<Matrix&> H3 = boost::none,
        boost::optional<Matrix&> H4 = boost::none,
        boost::optional<Matrix&> H5 = boost::none) const {
    if(H1) {
      *H1 = Matrix_(1,1, 1.0);
      *H2 = Matrix_(1,1, 2.0);
      *H3 = Matrix_(1,1, 3.0);
      *H4 = Matrix_(1,1, 4.0);
      *H5 = Matrix_(1,1, 5.0);
    }
    return (Vector(1) << x1 + x2 + x3 + x4 + x5).finished();
  }
};

/* ************************************ */
TEST(NonlinearFactor, NonlinearFactor5) {
  TestFactor5 tf;
  Values tv;
  tv.insert(TestKey(1), LieVector(1, 1.0));
  tv.insert(TestKey(2), LieVector(1, 2.0));
  tv.insert(TestKey(3), LieVector(1, 3.0));
  tv.insert(TestKey(4), LieVector(1, 4.0));
  tv.insert(TestKey(5), LieVector(1, 5.0));
  EXPECT(assert_equal(Vector_(1, 15.0), tf.unwhitenedError(tv)));
  DOUBLES_EQUAL(56.25/2.0, tf.error(tv), 1e-9);
  Ordering ordering; ordering += TestKey(1), TestKey(2), TestKey(3), TestKey(4), TestKey(5);
  JacobianFactor jf(*boost::dynamic_pointer_cast<JacobianFactor>(tf.linearize(tv, ordering)));
  LONGS_EQUAL(jf.keys()[0], 0);
  LONGS_EQUAL(jf.keys()[1], 1);
  LONGS_EQUAL(jf.keys()[2], 2);
  LONGS_EQUAL(jf.keys()[3], 3);
  LONGS_EQUAL(jf.keys()[4], 4);
  EXPECT(assert_equal(Matrix_(1,1, 0.5), jf.getA(jf.begin())));
  EXPECT(assert_equal(Matrix_(1,1, 1.0), jf.getA(jf.begin()+1)));
  EXPECT(assert_equal(Matrix_(1,1, 1.5), jf.getA(jf.begin()+2)));
  EXPECT(assert_equal(Matrix_(1,1, 2.0), jf.getA(jf.begin()+3)));
  EXPECT(assert_equal(Matrix_(1,1, 2.5), jf.getA(jf.begin()+4)));
  EXPECT(assert_equal(Vector_(1, -7.5), jf.getb()));
}

/* ************************************************************************* */
class TestFactor6 : public NonlinearFactor6<TestKey, TestKey, TestKey, TestKey, TestKey, TestKey> {
public:
  typedef NonlinearFactor6<TestKey, TestKey, TestKey, TestKey, TestKey, TestKey> Base;
  TestFactor6() : Base(sharedSigmas(Vector_(1, 2.0)), 1, 2, 3, 4, 5, 6) {}
  virtual ~TestFactor6() {}

  virtual Vector
    evaluateError(const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6,
        boost::optional<Matrix&> H1 = boost::none,
        boost::optional<Matrix&> H2 = boost::none,
        boost::optional<Matrix&> H3 = boost::none,
        boost::optional<Matrix&> H4 = boost::none,
        boost::optional<Matrix&> H5 = boost::none,
        boost::optional<Matrix&> H6 = boost::none) const {
    if(H1) {
      *H1 = Matrix_(1,1, 1.0);
      *H2 = Matrix_(1,1, 2.0);
      *H3 = Matrix_(1,1, 3.0);
      *H4 = Matrix_(1,1, 4.0);
      *H5 = Matrix_(1,1, 5.0);
      *H6 = Matrix_(1,1, 6.0);
    }
    return (Vector(1) << x1 + x2 + x3 + x4 + x5 + x6).finished();
  }
};

/* ************************************ */
TEST(NonlinearFactor, NonlinearFactor6) {
  TestFactor6 tf;
  Values tv;
  tv.insert(TestKey(1), LieVector(1, 1.0));
  tv.insert(TestKey(2), LieVector(1, 2.0));
  tv.insert(TestKey(3), LieVector(1, 3.0));
  tv.insert(TestKey(4), LieVector(1, 4.0));
  tv.insert(TestKey(5), LieVector(1, 5.0));
  tv.insert(TestKey(6), LieVector(1, 6.0));
  EXPECT(assert_equal(Vector_(1, 21.0), tf.unwhitenedError(tv)));
  DOUBLES_EQUAL(110.25/2.0, tf.error(tv), 1e-9);
  Ordering ordering; ordering += TestKey(1), TestKey(2), TestKey(3), TestKey(4), TestKey(5), TestKey(6);
  JacobianFactor jf(*boost::dynamic_pointer_cast<JacobianFactor>(tf.linearize(tv, ordering)));
  LONGS_EQUAL(jf.keys()[0], 0);
  LONGS_EQUAL(jf.keys()[1], 1);
  LONGS_EQUAL(jf.keys()[2], 2);
  LONGS_EQUAL(jf.keys()[3], 3);
  LONGS_EQUAL(jf.keys()[4], 4);
  LONGS_EQUAL(jf.keys()[5], 5);
  EXPECT(assert_equal(Matrix_(1,1, 0.5), jf.getA(jf.begin())));
  EXPECT(assert_equal(Matrix_(1,1, 1.0), jf.getA(jf.begin()+1)));
  EXPECT(assert_equal(Matrix_(1,1, 1.5), jf.getA(jf.begin()+2)));
  EXPECT(assert_equal(Matrix_(1,1, 2.0), jf.getA(jf.begin()+3)));
  EXPECT(assert_equal(Matrix_(1,1, 2.5), jf.getA(jf.begin()+4)));
  EXPECT(assert_equal(Matrix_(1,1, 3.0), jf.getA(jf.begin()+5)));
  EXPECT(assert_equal(Vector_(1, -10.5), jf.getb()));

}
/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
