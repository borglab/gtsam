/**
 * @file    timeFrobeniusFactor.cpp
 * @brief   Time FrobeniusBetweenFactor::evaluateError with derivatives for various Lie groups.
 * @author  Frank Dellaert
 * @date    April 2024
 */

#include <gtsam/slam/FrobeniusFactor.h>
#include <gtsam/base/timing.h>

 // Include all Lie Groups to be timed
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/SOn.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/geometry/SO4.h>
#include <gtsam/geometry/Gal3.h>
#include <gtsam/navigation/NavState.h>

#include <iostream>
#include <string>

using namespace gtsam;

// Define a sufficiently large number of iterations for stable timing.
static constexpr size_t NUM_ITERATIONS = 10000000;

// Create CSV file for results
std::ofstream os("timeFrobeniusFactor.csv");

/**
 * @brief Time evaluateError for a given Lie group type.
 * This function creates a FrobeniusBetweenFactor for a given Lie group type `T`,
 * and times the `evaluateError` method including Jacobian calculations.
 * @tparam T The Lie group type (e.g., Rot3, Pose3). It must be a MatrixLieGroup.
 * @param name A string identifier for the type, used for printing results.
 */
template <typename T>
void timeOne(const std::string& name) {
  Key key1(1), key2(2);

  // Use random values to avoid any special cases (e.g. identity).
  // Using ChartAtOrigin::Retract is the modern GTSAM way.
  T T1 = T::ChartAtOrigin::Retract(Vector::Random(T::dimension));
  T T2 = T::ChartAtOrigin::Retract(Vector::Random(T::dimension));
  T T12 = T::ChartAtOrigin::Retract(Vector::Random(T::dimension));

  // Create the factor. The noise model is not important for timing evaluateError.
  FrobeniusBetweenFactor<T> factor(key1, key2, T12, nullptr);

  // Get dimensions for Jacobian matrices.
  // N is the matrix dimension (e.g., 3 for SO(3)), Dim is N*N.
  // D is the dimension of the tangent space.
  constexpr auto N = T::LieAlgebra::RowsAtCompileTime;
  constexpr auto Dim = N * N;
  constexpr auto D = T::dimension;
  Matrix H1(Dim, D), H2(Dim, D);

  // Warmup call to make sure code and data are in cache.
  for (int i = 0; i < 100; ++i) {
    factor.evaluateError(T1, T2, &H1, &H2);
  }

  // Timed loop.
  static const size_t id_tic = ::gtsam::internal::getTicTocID(name.c_str());
  ::gtsam::internal::AutoTicToc obj(id_tic, name.c_str());
  for (size_t t = 0; t < NUM_ITERATIONS; t++) {
    // We pass H1 and H2 to ensure Jacobians are computed.
    factor.evaluateError(T1, T2, &H1, &H2);
  }
  obj.stop();
  auto timer = ::gtsam::internal::gCurrentTimer.lock()->child(id_tic, name.c_str(), ::gtsam::internal::gCurrentTimer);
  os << timer->secs() / NUM_ITERATIONS << ", \n";
  std::cout << name << ":\t" << timer->secs()*1e9/NUM_ITERATIONS << " ns" << std::endl;
}

/*************************************************************************************/
int main(void) {
  // Announce the test and number of iterations.
  std::cout << "Timing FrobeniusBetweenFactor::evaluateError with derivatives for " << NUM_ITERATIONS
    << " iterations.\n" << std::endl;

  // Time all the specified MatrixLieGroup classes.
  timeOne<Rot2>("Rot2");
  timeOne<SO3>("SO3");
  timeOne<Rot3>("Rot3");
  timeOne<Pose2>("Pose2");
  timeOne<SO4>("SO4");
  timeOne<Pose3>("Pose3");
  timeOne<Gal3>("Gal3");
  timeOne<NavState>("NavState");

  return 0;
}
/*************************************************************************************/