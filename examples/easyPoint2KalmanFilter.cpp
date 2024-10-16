/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file easyPoint2KalmanFilter.cpp
 *
 * simple linear Kalman filter on a moving 2D point, but done using factor graphs
 * This example uses the templated ExtendedKalmanFilter class to perform the same
 * operations as in elaboratePoint2KalmanFilter
 *
 * @date Aug 19, 2011
 * @author Frank Dellaert
 * @author Stephen Williams
 */

#include <gtsam/nonlinear/ExtendedKalmanFilter.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Point2.h>

using namespace std;
using namespace gtsam;

// Define Types for Linear System Test
typedef Point2 LinearMeasurement;

int main() {

  // Create the Kalman Filter initialization point
  Point2 x_initial(0.0, 0.0);
  SharedDiagonal P_initial = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1));

  // Create Key for initial pose
  Symbol x0('x',0);

  // Create an ExtendedKalmanFilter object
  ExtendedKalmanFilter<Point2> ekf(x0, x_initial, P_initial);

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
  Vector u = Vector2(1.0, 0.0);
  SharedDiagonal Q = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1), true);

  // This simple motion can be modeled with a BetweenFactor
  // Create Key for next pose
  Symbol x1('x',1);
  // Predict delta based on controls
  Point2 difference(1,0);
  // Create Factor
  BetweenFactor<Point2> factor1(x0, x1, difference, Q);

  // Predict the new value with the EKF class
  Point2 x1_predict = ekf.predict(factor1);
  traits<Point2>::Print(x1_predict, "X1 Predict");



  // Now, a measurement, z1, has been received, and the Kalman Filter should be "Updated"/"Corrected"
  // This is equivalent to saying P(x1|z1) ~ P(z1|x1)*P(x1)
  // For the Kalman Filter, this requires a measurement model h(x_{t}) = \hat{z}_{t}
  // Assuming the system is linear, this will be of the form h(x_{t}) = H*x_{t} + v
  // where H is the observation model/matrix, and v is zero-mean, Gaussian white noise with covariance R
  //
  // For the purposes of this example, let us assume we have something like a GPS that returns
  // the current position of the robot. Then H = [1 0 ; 0 1]. Let us also assume that the measurement noise
  // R = [0.25 0 ; 0 0.25].
  SharedDiagonal R = noiseModel::Diagonal::Sigmas(Vector2(0.25, 0.25), true);

  // This simple measurement can be modeled with a PriorFactor
  Point2 z1(1.0, 0.0);
  PriorFactor<Point2> factor2(x1, z1, R);

  // Update the Kalman Filter with the measurement
  Point2 x1_update = ekf.update(factor2);
  traits<Point2>::Print(x1_update, "X1 Update");



  // Do the same thing two more times...
  // Predict
  Symbol x2('x',2);
  difference = Point2(1,0);
  BetweenFactor<Point2> factor3(x1, x2, difference, Q);
  Point2 x2_predict = ekf.predict(factor3);
  traits<Point2>::Print(x2_predict, "X2 Predict");

  // Update
  Point2 z2(2.0, 0.0);
  PriorFactor<Point2> factor4(x2, z2, R);
  Point2 x2_update = ekf.update(factor4);
  traits<Point2>::Print(x2_update, "X2 Update");



  // Do the same thing one more time...
  // Predict
  Symbol x3('x',3);
  difference = Point2(1,0);
  BetweenFactor<Point2> factor5(x2, x3, difference, Q);
  Point2 x3_predict = ekf.predict(factor5);
  traits<Point2>::Print(x3_predict, "X3 Predict");

  // Update
  Point2 z3(3.0, 0.0);
  PriorFactor<Point2> factor6(x3, z3, R);
  Point2 x3_update = ekf.update(factor6);
  traits<Point2>::Print(x3_update, "X3 Update");

  return 0;
}
