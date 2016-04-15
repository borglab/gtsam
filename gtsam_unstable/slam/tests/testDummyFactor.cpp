/**
 * @file testDummyFactor.cpp
 *
 * @brief A simple dummy nonlinear factor that can be used to enforce connectivity
 * without adding any real information
 *
 * @date Sep 10, 2012
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam_unstable/slam/DummyFactor.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/base/TestableAssertions.h>

using namespace gtsam;

const double tol = 1e-9;

/* ************************************************************************* */
TEST( testDummyFactor, basics ) {

  gtsam::Key key1 = 7, key2 = 9;
  size_t dim1 = 3, dim2 = 3;
  DummyFactor dummyfactor(key1, dim1, key2, dim2);

  // verify contents
  LONGS_EQUAL(2, dummyfactor.size());
  EXPECT_LONGS_EQUAL(key1, dummyfactor.keys()[0]);
  EXPECT_LONGS_EQUAL(key2, dummyfactor.keys()[1]);

  LONGS_EQUAL(2, dummyfactor.dims().size());
  EXPECT_LONGS_EQUAL(dim1, dummyfactor.dims()[0]);
  EXPECT_LONGS_EQUAL(dim2, dummyfactor.dims()[1]);

  Values c;
  c.insert(key1, Point3(1.0, 2.0, 3.0));
  c.insert(key2, Point3(4.0, 5.0, 6.0));

  // errors - all zeros
  DOUBLES_EQUAL(0.0, dummyfactor.error(c), tol);

  // linearization: all zeros
  GaussianFactor::shared_ptr actLinearization = dummyfactor.linearize(c);
  CHECK(actLinearization);
  noiseModel::Diagonal::shared_ptr model3 = noiseModel::Unit::Create(3);
  GaussianFactor::shared_ptr expLinearization(new JacobianFactor(
      key1, Matrix::Zero(3,3), key2, Matrix::Zero(3,3), Z_3x1, model3));
  EXPECT(assert_equal(*expLinearization, *actLinearization, tol));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
