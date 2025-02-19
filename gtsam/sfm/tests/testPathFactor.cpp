
//------------------------------------------------------------------------------
// Unit tests for PathFactor using Rot3 as the Lie group.
//------------------------------------------------------------------------------

#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/types.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/PathFactor.h>

#include <CppUnitLite/TestHarness.h>

using namespace gtsam;

using namespace std::placeholders;
//--------------------------------------------------------------------------
// Helper Functions
//--------------------------------------------------------------------------

/**
 * Helper function to populate a Values object with the measurements for a path.
 * For an EdgeKey, if the forward measurement (g_{ij}) exists then it is used;
 * otherwise, the reversed measurement (g_{ji}) is expected, so we store that value.
 *
 * In our tests:
 * - For edge e1, we assume the forward measurement is available.
 * - For edge e2, we assume the measurement is stored under e2.reversed().
 */
void populateValues(const EdgeKey& e1, const EdgeKey& e2,
                      const Rot3& R12, const Rot3& R23, Values& values) {
  values.insert(static_cast<Key>(e1), R12);
  values.insert(static_cast<Key>(e2.reversed()), R23.inverse());
}


//--------------------------------------------------------------------------
// Test: Factor Error
//--------------------------------------------------------------------------

TEST(PathFactor, Error) {
  // Create a simple path: from node 1->2 and 2->3.
  EdgeKey e1(1, 2), e2(2, 3);
  std::vector<EdgeKey> pathKeys = { e1, e2 };

  // Define rotations.
  // R12: rotation about Z-axis by 30 degrees.
  Rot3 R12 = Rot3::Rz(30 * M_PI / 180.0);
  // R23: rotation about Z-axis by 45 degrees.
  Rot3 R23 = Rot3::Rz(45 * M_PI / 180.0);

  // The predicted overall rotation is R12 ∘ R23.
  Rot3 predicted = R12.compose(R23);
  
  // Create a PathFactor with measured rotation equal to the predicted one.
  PathFactor<Rot3> factor(pathKeys, predicted);

  // Populate a Values object with the appropriate measurements.
  Values values;
  populateValues(e1, e2, R12, R23, values);

  // Compute the factor error.
  double error_val = factor.error(values);
  std::cout << "Error (should be near 0): " << error_val << std::endl;
  EXPECT(abs(error_val) < 1e-6);
}

Vector3 residualFunc(const Rot3& R12, const Rot3& R32, const Rot3& R13) {
  Rot3 prediction = R12.compose(R32.inverse());
  Rot3 residual = R13.inverse().compose(prediction);
  return Rot3::Logmap(residual);
};


//--------------------------------------------------------------------------
// Test: Jacobian (Analytical vs. Numerical)
//--------------------------------------------------------------------------
TEST(PathFactor, Jacobian) {
  // Create a simple path: from node 1->2 and 2->3.
  EdgeKey e1(1, 2), e2(2, 3);
  std::vector<EdgeKey> pathKeys = { e1, e2 };

  // Define rotations.
  Rot3 R12 = Rot3::Rz(30 * M_PI / 180.0);  // for edge e1 (forward)
  Rot3 R23 = Rot3::Rz(45 * M_PI / 180.0);  // for edge e2 (measurement stored reversed)
  
  // The predicted overall rotation is R12 ∘ R23.
  Rot3 R13 = R12.compose(R23);
  
  // Create the PathFactor.
  PathFactor<Rot3> factor(pathKeys, R13);
  
  // Populate a Values object.
  Values values;
  populateValues(e1, e2, R12, R23, values);
  
  // Linearize the factor.
  auto linearFactor = factor.linearize(values); 
  auto jacFactor = std::dynamic_pointer_cast<JacobianFactor>(linearFactor);
  EXPECT(jacFactor);
  
  // Obtain the overall Jacobian matrix A and residual vector b.
  std::pair<Matrix, Vector> A_b = jacFactor->jacobian();
  Matrix A_total = A_b.first;
  EXPECT(A_total.cols() == 6);  // Two keys, each 3-dimensional.
  EXPECT(A_total.rows() == 3);  // Three dimensional error.
  
  
  Matrix numJacobian1 = numericalDerivative11<Vector3, Rot3>(
    std::bind(&residualFunc, std::placeholders::_1, R23.inverse(), R13), R12);
  Matrix numJacobian2 = numericalDerivative11<Vector3, Rot3>(
    std::bind(&residualFunc, R12, std::placeholders::_1, R13), R23.inverse());

  // Extract the corresponding block for key e1 from the analytical Jacobian.
  Matrix A_block1 = A_total.block(0, 0, A_total.rows(), 3);
  EXPECT(assert_equal(numJacobian1, A_block1, 1e-6));
  
  Matrix A_block2 = A_total.block(0, 3, A_total.rows(), 3);
  EXPECT(assert_equal(numJacobian2, A_block2, 1e-6));
}


/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
