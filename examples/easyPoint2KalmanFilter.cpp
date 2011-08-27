/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * easyPoint2KalmanFilter.cpp
 *
 * simple linear Kalman filter on a moving 2D point, but done using factor graphs
 * This example uses the templated ExtendedKalmanFilter class to perform the same
 * operations as in elaboratePoint2KalmanFilter
 *
 *  Created on: Aug 19, 2011
 *  @Author: Frank Dellaert
 *  @Author: Stephen Williams
 */

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testExtendedKalmanFilter
 * @author Stephen Williams
 */

#include <gtsam/nonlinear/ExtendedKalmanFilter-inl.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LieValues-inl.h>
#include <gtsam/geometry/Point2.h>

using namespace std;
using namespace gtsam;

// Define Types for Linear System Test
typedef TypedSymbol<Point2, 'x'> LinearKey;
typedef LieValues<LinearKey> LinearValues;
typedef Point2 LinearMeasurement;

int main() {

  // Create the Kalman Filter initialization point
  Point2 x_initial(0.0, 0.0);
  SharedDiagonal P_initial = noiseModel::Diagonal::Sigmas(Vector_(2, 0.1, 0.1));

  // Create an ExtendedKalmanFilter object
  ExtendedKalmanFilter<LinearValues,LinearKey> ekf(x_initial, P_initial);



  // Now predict the state at t=1, i.e. argmax_{x1} P(x1) = P(x1|x0) P(x0)
  // In Kalman Filter notation, this is x_{t+1|t} and P_{t+1|t}
  // For the Kalman Filter, this requires a motion model, f(x_{t}) = x_{t+1|t)
  // Assuming the system is linear, this will be of the form f(x_{t}) = F*x_{t} + B*u_{t} + w
  // where F is the state transition model/matrix, B is the control input model,
  // and w is zero-mean, Gaussian white noise with covariance Q
  // Note, in some models, Q is actually derived as G*w*G^T where w models uncertainty of some
  // physical property, such as velocity or acceleration, and G is derived from physics
  //
  // For the purposes of this example, let us assume we are using a constant-position model and
  // the controls are driving the point to the right at 1 m/s. Then, F = [1 0 ; 0 1], B = [1 0 ; 0 1]
  // and u = [1 ; 0]. Let us also assume that the process noise Q = [0.1 0 ; 0 0.1].
  Vector u = Vector_(2, 1.0, 0.0);
  SharedGaussian Q = noiseModel::Diagonal::Sigmas(Vector_(2, 0.1, 0.1), true);

  // This simple motion can be modeled with a BetweenFactor
  // Create Keys
  LinearKey x0(0), x1(1);
  // Predict delta based on controls
  Point2 difference(1,0);
  // Create Factor
  BetweenFactor<LinearValues,LinearKey> factor1(x0, x1, difference, Q);

  // Predict the new value with the EKF class
  Point2 x1_predict = ekf.predict(factor1);
  x1_predict.print("X1 Predict");



  // Now, a measurement, z1, has been received, and the Kalman Filter should be "Updated"/"Corrected"
  // This is equivalent to saying P(x1|z1) ~ P(z1|x1)*P(x1)
  // For the Kalman Filter, this requires a measurement model h(x_{t}) = \hat{z}_{t}
  // Assuming the system is linear, this will be of the form h(x_{t}) = H*x_{t} + v
  // where H is the observation model/matrix, and v is zero-mean, Gaussian white noise with covariance R
  //
  // For the purposes of this example, let us assume we have something like a GPS that returns
  // the current position of the robot. Then H = [1 0 ; 0 1]. Let us also assume that the measurement noise
  // R = [0.25 0 ; 0 0.25].
  SharedGaussian R = noiseModel::Diagonal::Sigmas(Vector_(2, 0.25, 0.25), true);

  // This simple measurement can be modeled with a PriorFactor
  Point2 z1(1.0, 0.0);
  PriorFactor<LinearValues,LinearKey> factor2(x1, z1, R);

  // Update the Kalman Filter with the measurement
  Point2 x1_update = ekf.update(factor2);
  x1_update.print("X1 Update");



  // Do the same thing two more times...
  // Predict
  LinearKey x2(2);
  difference = Point2(1,0);
  BetweenFactor<LinearValues,LinearKey> factor3(x1, x2, difference, Q);
  Point2 x2_predict = ekf.predict(factor1);
  x2_predict.print("X2 Predict");

  // Update
  Point2 z2(2.0, 0.0);
  PriorFactor<LinearValues,LinearKey> factor4(x2, z2, R);
  Point2 x2_update = ekf.update(factor4);
  x2_update.print("X2 Update");



  // Do the same thing one more time...
  // Predict
  LinearKey x3(3);
  difference = Point2(1,0);
  BetweenFactor<LinearValues,LinearKey> factor5(x2, x3, difference, Q);
  Point2 x3_predict = ekf.predict(factor5);
  x3_predict.print("X3 Predict");

  // Update
  Point2 z3(3.0, 0.0);
  PriorFactor<LinearValues,LinearKey> factor6(x3, z3, R);
  Point2 x3_update = ekf.update(factor6);
  x3_update.print("X3 Update");

  return 0;
}
