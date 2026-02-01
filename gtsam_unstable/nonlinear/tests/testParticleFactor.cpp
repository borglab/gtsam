/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testParticleFactor.cpp
 * @brief   Unit tests for the Particle factor
 * @author  Frank Dellaert
 * @date    Dec 9, 2013
 */

#include <gtsam/linear/NoiseModel.h>

namespace gtsam {

/**
 * A factor approximating a density on a variable as a set of weighted particles
 */
template<class X>
class ParticleFactor {

public:
  typedef ParticleFactor This; ///< Typedef to this class
  typedef std::shared_ptr<This> shared_ptr; ///< shared_ptr to this class

};

/**
 * A particle filter class, styled on functional KalmanFilter
 * A state is created, which is functionally updates through [predict] and [update]
 */
template<class X>
class ParticleFilter {

public:

  /**
   * The Particle filter state is simply a ParticleFactor
   */
  typedef typename ParticleFactor<X>::shared_ptr State;

  /**
   * Create initial state, i.e., prior density at time k=0
   * In Bayes Filter notation, these are x_{0|0} and P_{0|0}
   * @param x0 estimate at time 0
   * @param P0 covariance at time 0, given as a diagonal Gaussian 'model'
   */
  State init(const Vector& x0, const SharedDiagonal& P0) {
    return std::make_shared<ParticleFactor<X> >();
  }

};
// ParticleFilter

}// namespace gtsam

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/linear/KalmanFilter.h>
#include <gtsam/geometry/Pose2.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

//******************************************************************************

TEST( particleFactor, constructor ) {
//  ParticleFactor<Pose2> pf;
  //CHECK(assert_equal(expected, actual));
}

//******************************************************************************
// Tests to do:
// Take two variables pf-x-*-y, eliminate x, multiply and sample then marginalize
TEST( particleFactor, eliminate) {
//  ParticleFactor<Pose2> fx;
  BetweenFactor<Pose2> fxy;

}

//******************************************************************************

/** Small 2D point class implemented as a Vector */
struct State: Vector {
  State(double x, double y) :
      Vector((Vector(2) << x, y).finished()) {
  }
};

//******************************************************************************
TEST( ParticleFilter, constructor) {

// Create a Kalman filter of dimension 2
  ParticleFilter<Pose2> pf1;

// Create inital mean/covariance
  State x_initial(0.0, 0.0);
  SharedDiagonal P1 = noiseModel::Isotropic::Sigma(2, 0.1);

// Get initial state by passing initial mean/covariance to the filter
  ParticleFilter<Pose2>::State p1 = pf1.init(x_initial, P1);

//  // Assert it has the correct mean, covariance and information
//  EXPECT(assert_equal(x_initial, p1->mean()));
//  Matrix Sigma = (Mat(2, 2) << 0.01, 0.0, 0.0, 0.01);
//  EXPECT(assert_equal(Sigma, p1->covariance()));
//  EXPECT(assert_equal(inverse(Sigma), p1->information()));
//
//  // Create one with a sharedGaussian
//  KalmanFilter::State p2 = pf1.init(x_initial, Sigma);
//  EXPECT(assert_equal(Sigma, p2->covariance()));
//
//  // Now make sure both agree
//  EXPECT(assert_equal(p1->covariance(), p2->covariance()));
}

//******************************************************************************
TEST( ParticleFilter, linear1 ) {

  // Create the controls and measurement properties for our example
  Matrix F = I_2x2;
  Matrix B = I_2x2;
  Vector u = Vector2(1.0, 0.0);
  SharedDiagonal modelQ = noiseModel::Isotropic::Sigma(2, 0.1);
  Matrix Q = 0.01 * I_2x2;
  Matrix H = I_2x2;
  State z1(1.0, 0.0);
  State z2(2.0, 0.0);
  State z3(3.0, 0.0);
  SharedDiagonal modelR = noiseModel::Isotropic::Sigma(2, 0.1);
  Matrix R = 0.01 * I_2x2;

// Create the set of expected output TestValues
  State expected0(0.0, 0.0);
  Matrix P00 = 0.01 * I_2x2;

  State expected1(1.0, 0.0);
  Matrix P01 = P00 + Q;
  Matrix I11 = P01.inverse() + R.inverse();

  State expected2(2.0, 0.0);
  Matrix P12 = I11.inverse() + Q;
  Matrix I22 = P12.inverse() + R.inverse();

  State expected3(3.0, 0.0);
  Matrix P23 = I22.inverse() + Q;
  Matrix I33 = P23.inverse() + R.inverse();

// Create a Kalman filter of dimension 2
  KalmanFilter kf(2);

// Create the Kalman Filter initialization point
  State x_initial(0.0, 0.0);
  SharedDiagonal P_initial = noiseModel::Isotropic::Sigma(2, 0.1);

// Create initial KalmanFilter object
  KalmanFilter::State p0 = kf.init(x_initial, P_initial);
  EXPECT(assert_equal(expected0, p0->mean()));
  EXPECT(assert_equal(P00, p0->covariance()));

// Run iteration 1
  KalmanFilter::State p1p = kf.predict(p0, F, B, u, modelQ);
  EXPECT(assert_equal(expected1, p1p->mean()));
  EXPECT(assert_equal(P01, p1p->covariance()));

  KalmanFilter::State p1 = kf.update(p1p, H, z1, modelR);
  EXPECT(assert_equal(expected1, p1->mean()));
  EXPECT(assert_equal(I11, p1->information()));

// Run iteration 2 (with full covariance)
  KalmanFilter::State p2p = kf.predictQ(p1, F, B, u, Q);
  EXPECT(assert_equal(expected2, p2p->mean()));

  KalmanFilter::State p2 = kf.update(p2p, H, z2, modelR);
  EXPECT(assert_equal(expected2, p2->mean()));

// Run iteration 3
  KalmanFilter::State p3p = kf.predict(p2, F, B, u, modelQ);
  EXPECT(assert_equal(expected3, p3p->mean()));
  LONGS_EQUAL(3, (long)KalmanFilter::step(p3p));

  KalmanFilter::State p3 = kf.update(p3p, H, z3, modelR);
  EXPECT(assert_equal(expected3, p3->mean()));
  LONGS_EQUAL(3, (long)KalmanFilter::step(p3));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

